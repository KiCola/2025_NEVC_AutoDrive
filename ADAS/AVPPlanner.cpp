#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

#include "Eigen/Core"
#include "Eigen/QR"
#include "UtilMath.hpp"
#include "AVPLog.hpp"
#include "AVPPlanner.hpp"

AVPPlanner::AVPPlanner(const HDMapStandalone::MParkingSpace& parkingSpace,
	const SSD::SimPoint3D& initPoint,
	const SSD::SimPoint3D& terminalPoint,
	const VehicleParam& veh,
	const bool leaveAfterParked,
	const double safetyDistance) :
	mParkingSpace(parkingSpace),
	mInitPoint(initPoint),
	mTerminalPoint(terminalPoint),
	mVeh(veh),
	mLeaveAfterParked(leaveAfterParked),
	mSafetyDistance(safetyDistance),
	mReversePointLocal({ 7., 4. }),
	mReversePoint(),
	mParkingEndPoint(),
	mTurningCompensation(0.4),
	mReverseTrajectoryStepSize(0.2),
	mReverseTrajectory(),
	mForwardTrajectoryStepSize(0.5),
	mForwardTrajectory(),
	mLeavingTrajectoryStepSize(0.5),
	mLeavingTrajectory()
{
	mOrigin.x = mParkingSpace.boundaryKnots[0].x;
	mOrigin.y = mParkingSpace.boundaryKnots[0].y;
	mAxis.x = mParkingSpace.boundaryKnots[3].x - mParkingSpace.boundaryKnots[0].x;
	mAxis.y = mParkingSpace.boundaryKnots[3].y - mParkingSpace.boundaryKnots[0].y;
	mOrientation = std::atan2(mAxis.y, mAxis.x);
	std::cout << "Parking space origin: [" << mOrigin.x << ", " << mOrigin.y << "]" << std::endl;
	std::cout << "Parking space axis: [" << mAxis.x << ", " << mAxis.y << "]" << std::endl;
	std::cout << "Parking space orientation: " << mOrientation << std::endl;
}

const SSD::SimPoint3DVector AVPPlanner::ReverseTrajectory() const
{
	return mReverseTrajectory;
}

const SSD::SimPoint3DVector AVPPlanner::ForwardTrajectory() const
{
	return mForwardTrajectory;
}

const SSD::SimPoint2D AVPPlanner::ParkingEndPoint() const
{
	return mParkingEndPoint;
}

const SSD::SimPoint3DVector AVPPlanner::LeavingTrajectory() const
{
	return mLeavingTrajectory;
}

void AVPPlanner::planReverseVerTrajectory()
{
	/* 1. Calculate control points */
	// length and width of the parking space
	SSD::SimPoint3D p0 { mParkingSpace.boundaryKnots[0].x, mParkingSpace.boundaryKnots[0].y, mParkingSpace.boundaryKnots[0].z };
	SSD::SimPoint3D p1 { mParkingSpace.boundaryKnots[1].x, mParkingSpace.boundaryKnots[1].y, mParkingSpace.boundaryKnots[1].z };
	SSD::SimPoint3D p2 { mParkingSpace.boundaryKnots[2].x, mParkingSpace.boundaryKnots[2].y, mParkingSpace.boundaryKnots[2].z };
	SSD::SimPoint3D p3 { mParkingSpace.boundaryKnots[3].x, mParkingSpace.boundaryKnots[3].y, mParkingSpace.boundaryKnots[3].z };
	double pL = UtilMath::PlanarDistance(p0, p3); // 横向距离
	double pW = UtilMath::PlanarDistance(p0, p1); // 纵向距离

	double x0_ori = mReversePointLocal.x;
	double y0_ori = mReversePointLocal.y;
	// rotate the coordinate frame by pi/4 for computational convenience
	SSD::SimPoint2D C0 = UtilMath::Rotate({ x0_ori, y0_ori }, M_PI_4);
	mReversePoint = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, C0);
	double x0 = C0.x;
	double y0 = C0.y;
	double kc0 = -1.;
	double xc2_ori = pL / 2 + mTurningCompensation;
	double yc2_ori = -pW + (mSafetyDistance + mVeh.Lr) + mVeh.L;
	SSD::SimPoint2D C2 = UtilMath::Rotate({ xc2_ori, yc2_ori }, M_PI_4);
	double xc2 = C2.x;
	double yc2 = C2.y;
	double xc3_ori = xc2_ori;
	double yc3_ori = -pW + (mSafetyDistance + mVeh.Lr);
	SSD::SimPoint2D C3 = UtilMath::Rotate({ xc3_ori, yc3_ori }, M_PI_4);
	double xc3 = C3.x;
	double yc3 = C3.y;
	double kc3 = 1.;

	double R = y0_ori - yc2_ori;
	double xc1_ori = xc2_ori + R;
	double yc1_ori = y0_ori;
	SSD::SimPoint2D C1 = UtilMath::Rotate({ xc1_ori, yc1_ori }, M_PI_4);
	double xc1 = C1.x;
	double yc1 = C1.y;

	// end point
	double xe_ori = pL / 2.;
	double vehicleLength = mVeh.Lf + mVeh.L + mVeh.Lr;
	double ye_ori = -pW + mVeh.Lr + (pW - vehicleLength) / 2.;
	SSD::SimPoint2D Ce = UtilMath::Rotate({ xe_ori, ye_ori }, M_PI_4);
	mParkingEndPoint = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, Ce);

	/* 2. Fit reverse trajectory using Eigen */
	Eigen::MatrixXd A(5, 5);
	A << pow(x0, 4), pow(x0, 3), pow(x0, 2), pow(x0, 1), 1,
		pow(xc1, 4), pow(xc1, 3), pow(xc1, 2), pow(xc1, 1), 1,
		pow(xc3, 4), pow(xc3, 3), pow(xc3, 2), pow(xc3, 1), 1,
		4 * pow(xc3, 3), 3 * pow(xc3, 2), 2 * pow(xc3, 1), 1, 0,
		pow(xc2, 4), pow(xc2, 3), pow(xc2, 2), pow(xc2, 1), 1;
	Eigen::VectorXd b(5);
	b << y0, yc1, yc3, kc3, yc2;
	auto xCoeff = A.householderQr().solve(b);

	// need to save the coeffs since other attempts to access coefficients from xCoeff
	// will change their values
	std::vector<double> coeffs{ xCoeff[4], xCoeff[3], xCoeff[2], xCoeff[1], xCoeff[0] };
	while (std::abs(coeffs[0]) > 5 || isnan(coeffs[0])
		|| std::abs(coeffs[1]) > 5 || isnan(coeffs[1])
		|| std::abs(coeffs[2]) > 5 || isnan(coeffs[2])
		|| std::abs(coeffs[3]) > 5 || isnan(coeffs[3])
		|| std::abs(coeffs[4]) > 5 || isnan(coeffs[4]))
	{
		auto xCoeffNew = A.householderQr().solve(b);
		coeffs[0] = xCoeffNew[4];
		coeffs[1] = xCoeffNew[3];
		coeffs[2] = xCoeffNew[2];
		coeffs[3] = xCoeffNew[1];
		coeffs[4] = xCoeffNew[0];
	}
	std::cout << "Fitted coefficients: [" << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2]
		<< ", " << coeffs[3] << ", " << coeffs[4] << "]" << std::endl;
	std::cout << std::endl;

	std::vector<SSD::SimPoint2D> reverseTrajLocal;
	double reverseStepSize = mReverseTrajectoryStepSize * (xc3 - x0) / abs(xc3 - x0);
	int reverseStepCount = (int)((xc3 - x0) / reverseStepSize);
	for (size_t i = 0; i <= reverseStepCount; ++i)
	{
		double x = x0 + i * reverseStepSize;
		double y = coeffs[4] * pow(x, 4) + coeffs[3] * pow(x, 3) + coeffs[2] * pow(x, 2)
			+ coeffs[1] * pow(x, 1) + coeffs[0];
		reverseTrajLocal.push_back({ x, y });
	}
	reverseTrajLocal.push_back({ xc3, yc3 });


	/* 3. Convert the local trajectory to a global one */
	mReverseTrajectory.resize(reverseTrajLocal.size());
	for (size_t i = 0; i < reverseTrajLocal.size(); ++i)
	{
		SSD::SimPoint2D trajPt = UtilMath::LocalToGlobal(mOrigin, mOrientation + M_PI_4, reverseTrajLocal[i]);
		mReverseTrajectory[i] = { trajPt.x, trajPt.y, 0. };
	}
}

void AVPPlanner::planForwardVerTrajectory()
{
	// 垂直停车
	double xDiff = mReversePoint.x - mInitPoint.x;
	double yDiff = mReversePoint.y - mInitPoint.y;
	if (std::abs(xDiff) > std::abs(yDiff)) {
		double forwardStepSize = mForwardTrajectoryStepSize * UtilMath::Sign(xDiff);
		int forwardStepCount = (int)(xDiff / forwardStepSize);
		for (size_t i = 0; i <= forwardStepCount; ++i)
		{
			double x = mInitPoint.x + i * forwardStepSize;
			double y = mInitPoint.y + i * forwardStepSize / xDiff * yDiff;
			mForwardTrajectory.push_back({ x, y, 0. });
		}
	}
	else {
		double forwardStepSize = mForwardTrajectoryStepSize * UtilMath::Sign(yDiff);
		int forwardStepCount = (int)(yDiff / forwardStepSize);
		for (size_t i = 0; i <= forwardStepCount; ++i)
		{
			double y = mInitPoint.y + i * forwardStepSize;
			double x = mInitPoint.x + i * forwardStepSize / yDiff * xDiff;
			mForwardTrajectory.push_back({ x, y, 0. });
		}
	}
}

void AVPPlanner::planLeavingVerTrajectory()
{
	// 垂直停车
	size_t reverseTrajSize = mReverseTrajectory.size();
	mLeavingTrajectory.resize(reverseTrajSize);
	for (size_t i = 0; i < reverseTrajSize/5; ++i) {
		mLeavingTrajectory[i] = mReverseTrajectory[reverseTrajSize - i - 1];
	}

	SSD::SimPoint3D StartPoint = { mReverseTrajectory.back().x, mReverseTrajectory.back().y, mReverseTrajectory.back().z };
	SSD::SimPoint3D EndPoint = { mTerminalPoint.x, mTerminalPoint.y, mTerminalPoint.z };
	SSD::SimPoint3DVector inputWayPoints;
	inputWayPoints.push_back(StartPoint);
	inputWayPoints.push_back(EndPoint);
    
    // 使用 generateRoute 规划从停车位出口到最终目标点的路径
    SSD::SimPoint3DVector routePoints;
	SSD::SimVector<int> indexOfValidPoints;
    bool success = SimOneAPI::GenerateRoute(inputWayPoints, indexOfValidPoints, routePoints);
	for (const auto& point : routePoints) {
        mLeavingTrajectory.push_back(point);
	}
}

void AVPPlanner::planReverseParTrajectory()
{
    // SSD::SimPoint3D p0 { mParkingSpace.boundaryKnots[0].x, mParkingSpace.boundaryKnots[0].y, mParkingSpace.boundaryKnots[0].z };
    // SSD::SimPoint3D p1 { mParkingSpace.boundaryKnots[1].x, mParkingSpace.boundaryKnots[1].y, mParkingSpace.boundaryKnots[1].z };
    // SSD::SimPoint3D p2 { mParkingSpace.boundaryKnots[2].x, mParkingSpace.boundaryKnots[2].y, mParkingSpace.boundaryKnots[2].z };
    // SSD::SimPoint3D p3 { mParkingSpace.boundaryKnots[3].x, mParkingSpace.boundaryKnots[3].y, mParkingSpace.boundaryKnots[3].z };
    
    // double pL = UtilMath::PlanarDistance(p0, p1);
    // double pW = UtilMath::PlanarDistance(p0, p3);
    
    // /* 2. 设置局部坐标系 */
    // // 使用停车位前端中点作为坐标原点
    // mOrigin = {(p0.x + p1.x) / 2, (p0.y + p1.y) / 2}; 
    
    // /* 3. 根据需求定义精确控制点 */
    // // 控制点C0：第一个点需要超过停车位
    // // -------------------------------
    // // 车辆位置在停车位前方，与道路平行
    // double x0_ori = pL + 0.5; // 超出停车位前端0.5米
    // double y0_ori = pW * 1.5; // 在车道上，距离停车位外侧1.5倍停车位宽度
    // SSD::SimPoint2D C0_local = {x0_ori, y0_ori};
    // SSD::SimPoint2D C0 = UtilMath::LocalToGlobal(mOrigin, mOrientation, C0_local);
    // mReversePoint = {C0.x, C0.y}; // 设置倒车起点
    
    // // 控制点C1：前轮过停车线，提供向右倒车空间
    // // -------------------------------------
    // // 前轮位置约等于车辆前端位置减去前悬长度
    // double x1_ori = pL * 0.8; // 前轮刚过停车线
    // double y1_ori = y0_ori;   // 保持与起点相同的y坐标（仍在车道上）
    // SSD::SimPoint2D C1_local = {x1_ori, y1_ori};
    
    // // 控制点C2：左后轮压在停车位左边线上，开始向左转向
    // // -------------------------------------------
    // // 后轮位置约等于车辆后端位置加上后悬长度
    // double x2_ori = pL * 0.3; // 车身已部分进入停车位
    // double y2_ori = pW * 0.8; // 左后轮刚好压在左边线上
    // SSD::SimPoint2D C2_local = {x2_ori, y2_ori};
    
    // // 控制点C3：终点，车辆完全进入停车位
    // // -----------------------------
    // double x3_ori = pL * 0.5; // 车位中央
    // double y3_ori = pW * 0.4; // 车辆完全进入停车位，与内侧保持适当距离
    // SSD::SimPoint2D C3_local = {x3_ori, y3_ori};
    // SSD::SimPoint2D C3 = UtilMath::LocalToGlobal(mOrigin, mOrientation, C3_local);
    // mParkingEndPoint = {C3.x, C3.y}; // 设置停车终点
    
    // /* 4. 使用贝塞尔曲线拟合轨迹 */
    // std::vector<SSD::SimPoint2D> curveLocal;
    // int numPoints = 60; // 轨迹点数量
    
    // for (int i = 0; i <= numPoints; i++) {
    //     double t = static_cast<double>(i) / numPoints;
        
    //     // 三阶贝塞尔曲线公式: B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
    //     double x = pow(1-t, 3) * C0_local.x + 
    //               3 * pow(1-t, 2) * t * C1_local.x + 
    //               3 * (1-t) * t * t * C2_local.x + 
    //               pow(t, 3) * C3_local.x;
        
    //     double y = pow(1-t, 3) * C0_local.y + 
    //               3 * pow(1-t, 2) * t * C1_local.y + 
    //               3 * (1-t) * t * t * C2_local.y + 
    //               pow(t, 3) * C3_local.y;
        
    //     curveLocal.push_back({x, y});
    // }
    
    // /* 5. 转换到全局坐标系 */
    // mReverseTrajectory.clear();
    
    // for (const auto& point : curveLocal) {
    //     SSD::SimPoint2D globalPoint = UtilMath::LocalToGlobal(mOrigin, mOrientation, point);
    //     mReverseTrajectory.push_back({globalPoint.x, globalPoint.y, 0});
    // }
    
    // std::cout << "size: " << mReverseTrajectory.size() << std::endl;
    
    // /* 可选：使用多项式拟合代替贝塞尔曲线 */
    // // 如果您希望使用类似垂直泊车的多项式拟合方式，可以取消下面的注释
    // /*
    // // 准备使用Eigen库进行四次多项式拟合
    // double x0 = C0_local.x;
    // double y0 = C0_local.y;
    // double x1 = C1_local.x;
    // double y1 = C1_local.y;
    // double x2 = C2_local.x;
    // double y2 = C2_local.y;
    // double x3 = C3_local.x;
    // double y3 = C3_local.y;
    
    // // 设置斜率约束
    // double kc0 = 0;       // 起点处水平方向（与道路平行）
    // double kc3 = 0;       // 终点处水平方向（与停车位平行）
    
    // Eigen::MatrixXd A(5, 5);
    // A << pow(x0, 4), pow(x0, 3), pow(x0, 2), pow(x0, 1), 1,
    //     pow(x1, 4), pow(x1, 3), pow(x1, 2), pow(x1, 1), 1,
    //     pow(x2, 4), pow(x2, 3), pow(x2, 2), pow(x2, 1), 1,
    //     pow(x3, 4), pow(x3, 3), pow(x3, 2), pow(x3, 1), 1,
    //     4 * pow(x3, 3), 3 * pow(x3, 2), 2 * pow(x3, 1), 1, 0;
    
    // Eigen::VectorXd b(5);
    // b << y0, y1, y2, y3, kc3;
    // auto xCoeff = A.householderQr().solve(b);
    
    // std::vector<double> coeffs{ xCoeff[4], xCoeff[3], xCoeff[2], xCoeff[1], xCoeff[0] };
    
    // // 生成轨迹点
    // mReverseTrajectory.clear();
    // double step = (x3 - x0) / 60.0;
    
    // for (double x = x0; x <= x3; x += step) {
    //     double y = coeffs[4] * pow(x, 4) + coeffs[3] * pow(x, 3) + 
    //               coeffs[2] * pow(x, 2) + coeffs[1] * x + coeffs[0];
        
    //     SSD::SimPoint2D localPoint = {x, y};
    //     SSD::SimPoint2D globalPoint = UtilMath::LocalToGlobal(mOrigin, mOrientation, localPoint);
    //     mReverseTrajectory.push_back({globalPoint.x, globalPoint.y, 0});
    // }
    // */

	SSD::SimPoint3D p0 = {25.6, -12.8, 0.0};
    SSD::SimPoint3D p1 = {21.6, -12.8, 0.0};  
    SSD::SimPoint3D p2 = {19.7, -14.6, 0.0}; 
    SSD::SimPoint3D p3 = {17.5, -15.8, 0.0};

	// SSD::SimPoint3DVector inputPoints;
	// SSD::SimPoint3DVector targetPath;
	// inputPoints.push_back(p0);
	// inputPoints.push_back(p1);
	// inputPoints.push_back(p2);
	// inputPoints.push_back(p3);
	// SSD::SimVector<int> indexOfValidPoints;
	// if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {
	// 	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
	// 	return;
	// }  
	// mReverseTrajectory.resize(targetPath.size());
	// for (size_t i = 0; i < targetPath.size(); ++i)
	// {
	// 	mReverseTrajectory[i] = {targetPath[i].x, targetPath[i].y, 0.};
	// }
	const int numPoints = 7;
	double traj_data[numPoints][3] = {
		//x          y            z
		{25.6,     -12.8,       0.   },
		{24.4,    -13.5,       0.   },
		{21.7,    -14.0 ,    0.},
		{20.0,     -15.0,       0.   }, 
		{19.5,     -15.9,       0.   }, 
		{18.5,     -15.9,       0.   }, 
		{18.0,     -15.9,       0.   },
	};

	for (size_t i = 0; i < numPoints; i++) {
		mReverseTrajectory.push_back({ traj_data[i][0], traj_data[i][1], traj_data[i][2] });
	}

	mReversePoint.x = p0.x;
	mReversePoint.y = p0.y;
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "Reverse point: [%f, %f]", mReversePoint.x, mReversePoint.y);
	mParkingEndPoint = {p3.x, p3.y};
	mOrigin = {p0.x, p0.y};
	mAxis = {p3.x - p0.x, p3.y - p0.y};
	mOrientation = std::atan2(mAxis.y, mAxis.x);
}



void AVPPlanner::planForwardParTrajectory()
{
	// // 平行停车
	// SSD::SimPoint3DVector inputPoints;
	// SSD::SimPoint3DVector targetPath;
	// SSD::SimPoint3D p0 = {mReversePoint.x, mReversePoint.y, 0.};
	// std::cout << "mInitPoint: [" << mInitPoint.x << ", " << mInitPoint.y << "]" << std::endl;
	// std::cout << "p0: [" << p0.x << ", " << p0.y << "]" << std::endl;
	// inputPoints.push_back(mInitPoint);
	// inputPoints.push_back(p0);
	// SSD::SimVector<int> indexOfValidPoints;
	// if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {
	// 	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
	// 	return;
	// }

	// mForwardTrajectory.resize(targetPath.size());
	// for (size_t i = 0; i < targetPath.size(); ++i)
	// {
	// 	mForwardTrajectory[i] = {targetPath[i].x, targetPath[i].y, 0.};
	// 	std::cout << "Forward trajectory point: [" << mForwardTrajectory[i].x << ", " << mForwardTrajectory[i].y << "]" << std::endl;
	// }
	double traj_data[3][3] = {
		//x          y            z
		{5.5,     -13.6,       0.   }, // 0
		{16.6 ,   -13.6,       0.   }, // 1
		{25.6,    -12.8,       0.   }, // 2
	};

	for (size_t i = 0; i <= 2; i++)
	{
		mForwardTrajectory.push_back({ traj_data[i][0], traj_data[i][1], traj_data[i][2] });
	}
}



void AVPPlanner::planLeavingParTrajectory()
{
	// // 平行停车
	// double xDiff = mReversePoint.x - mInitPoint.x;
	// double yDiff = mReversePoint.y - mInitPoint.y;
	// if (std::abs(xDiff) > std::abs(yDiff)) {
	// 	double leavingStepSize = mLeavingTrajectoryStepSize * UtilMath::Sign(xDiff);
	// 	int leavingStepCount = (int)(xDiff / leavingStepSize);
	// 	for (size_t i = 0; i <= leavingStepCount; ++i)
	// 	{
	// 		double x = mReversePoint.x + i * leavingStepSize;
	// 		double y = mReversePoint.y + i * leavingStepSize / xDiff * yDiff;
	// 		mLeavingTrajectory.push_back({ x, y, 0. });
	// 	}
	// }
	// else {
	// 	double leavingStepSize = mLeavingTrajectoryStepSize * UtilMath::Sign(yDiff);
	// 	int leavingStepCount = (int)(yDiff / leavingStepSize);
	// 	for (size_t i = 0; i <= leavingStepCount; ++i)
	// 	{
	// 		double y = mReversePoint.y + i * leavingStepSize;
	// 		double x = mReversePoint.x + i * leavingStepSize / yDiff * xDiff;
	// 		mLeavingTrajectory.push_back({ x, y, 0. });
	// 	}
	// }

	SSD::SimPoint3D StartPoint = { mReverseTrajectory.back().x, mReverseTrajectory.back().y, mReverseTrajectory.back().z };
	SSD::SimPoint3D EndPoint = { mTerminalPoint.x, mTerminalPoint.y, mTerminalPoint.z };
	SSD::SimPoint3DVector inputWayPoints;
	SSD::SimPoint3D middlePoint = { 20.0, -12.0, 0.0 };
	inputWayPoints.push_back(StartPoint);
	inputWayPoints.push_back(middlePoint);
	inputWayPoints.push_back(EndPoint);
    
    // 使用 generateRoute 规划从停车位出口到最终目标点的路径
    SSD::SimPoint3DVector routePoints;
	SSD::SimVector<int> indexOfValidPoints;
    bool success = SimOneAPI::GenerateRoute(inputWayPoints, indexOfValidPoints, routePoints);
	for (const auto& point : routePoints) {
        mLeavingTrajectory.push_back(point);
	}
}

void AVPPlanner::plan(long Case)
{
	SSD::SimPoint3D p0 = mParkingSpace.boundaryKnots[0];
    SSD::SimPoint3D p1 = mParkingSpace.boundaryKnots[1];
    SSD::SimPoint3D p2 = mParkingSpace.boundaryKnots[2];
    SSD::SimPoint3D p3 = mParkingSpace.boundaryKnots[3];
	double length = UtilMath::PlanarDistance(p0, p1);
	double width = UtilMath::PlanarDistance(p0, p3);
	double ratio = length / width;
	if (Case == 7)
	{
		planReverseVerTrajectory();
		planForwardVerTrajectory();
		if (mLeaveAfterParked)
		{
			planLeavingVerTrajectory();
		}
	}
	else if (Case == 8)
	{
		planReverseParTrajectory();
		planForwardParTrajectory();
		if (mLeaveAfterParked)
		{
			planLeavingParTrajectory();
		}
	}
}
