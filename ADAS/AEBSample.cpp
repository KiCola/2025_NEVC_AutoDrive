#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneEvaluationAPI.h"
#include "SSD/SimPoint3D.h"

#include <stdio.h>
#include "UtilMath.h"
#include "hdmap/SampleGetNearMostLane.h"
#include "hdmap/SampleGetLaneST.h"
#include <memory>
#include <limits>
#include <iostream>

int case_num = 0;

//Main function
//
int main()
{
	freopen("output.txt", "w", stdout);
	bool inAEBState = false;//AEB状态标志
	bool isSimOneInitialized = false;//SimOneAPI是否已初始化的标志
	const char* MainVehicleId = "0";// 主车辆ID
	bool isJoinTimeLoop = true;// 是否加入时间循环的标志
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop); // 调用SimOneAPI的初始化函数
	SimOneAPI::SetDriverName(MainVehicleId, "AEB");// 设置驾驶员名称为"AEB"
	SimOneAPI::SetDriveMode(MainVehicleId, ESimOne_Drive_Mode_API);// 设置驾驶模式为API模式
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);// 使用本地数据初始化评估服务

	int timeout = 20; //设置超时时间为20秒
	/*加载地图*/
	while (true) {
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	while (true) {
		int frame = SimOneAPI::Wait();// 等待下一帧

		/*检查案例运行状态*/
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			fclose(stdout);
			break;
		}

		/*获取主车辆的GPS数据*/
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}

		//std::unique_ptr<SimOne_Data_Traffic_Info> pTrafficInfo = std::make_unique<SimOne_Data_Traffic_Info>();
		//if (!SimOneAPI::GetTrafficSignList(MainVehicleId, pTrafficInfo.get()))
		//{
		//	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch traffic sign failed");
		//}
		//else
		//{
		//	std::cout << "Traffic Sign List:" << std::endl;
		////	for (size_t i = 0; i < pTrafficInfo->trafficSignListSize; ++i) {
		////		std::cout << "Traffic Sign ID: " << pTrafficInfo->id << std::endl;
		////		//std::cout << "Traffic Sign Type: " << pTrafficInfo->type << std::endl;
		//}

		//SimOneAPI::GetTrafficLight
		//获取限速标志牌信息
		SSD::SimVector<HDMapStandalone::MSignal> pTrafficSignal;
		SimOneAPI::GetTrafficSignList(pTrafficSignal);
		double LimitedSpeed = -1;
		if (pTrafficSignal.size() == 0)
		{

			//std::cout << "This scene has no Traffic Sign." << std::endl;
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "This scene has no Traffic Sign.");

		}
		else 
		{
			for (size_t i = 0; i < pTrafficSignal.size(); ++i)
			{
				std::cout << "ID:" << pTrafficSignal[i].id << std::endl;
				std::cout << "Value:" << pTrafficSignal[i].value.GetString() << std::endl;

				LimitedSpeed = std::stod(pTrafficSignal[i].value.GetString());
				std::cout << "LimitedSpeed:" << LimitedSpeed << std::endl;
				//if(pTrafficSignal[i].type == ESimOne_Obstacle_Type::ESimOne_Obstacle_Type_TrafficSign);
			}
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "Have Traffic Sign!!!");
		}

		//std::unique_ptr<SimOne_Data_TrafficLight>

		/*获取主车辆的障碍物信息*/
		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}

		
		/*检查SimOne案例的运行状态*/
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}

			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);//表示主车辆的位置
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);//计算主车辆的速度

			double minDistance = std::numeric_limits<double>::max();//初始化最小距离为最大可能
			int potentialObstacleIndex = pObstacle->obstacleSize;//初始化潜在障碍物的索引为障碍物数组的大小（即无效索引）
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos); //获取主车辆所在车道的ID
			SSD::SimString potentialObstacleLaneId = ""; //初始化潜在障碍物所在车道的ID为空字符串
			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);//获取障碍物的三维坐标
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);//获取障碍物所在的车道ID
				if (mainVehicleLaneId == obstacleLaneId) {//判断障碍物是否在主车辆所在车道
					double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);// 计算主车辆与障碍物之间的平面距离

					if (obstacleDistance < minDistance) {//判断当前障碍物的距离是否小于已知的最小距离
						minDistance = obstacleDistance;
						potentialObstacleIndex = (int)i;
						potentialObstacleLaneId = obstacleLaneId;
					}
				}
			}

			auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];// 获取潜在障碍物的信息
			double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);// 计算潜在障碍物的速度


			SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);//表示潜在障碍物位置

			// 初始化潜在障碍物的s和t值
			double sObstalce = 0;
			double tObstacle = 0;

			// 初始化主车辆的s和t值
			double sMainVehicle = 0;
			double tMainVehicle = 0;

			bool isObstalceBehind = false;//障碍物是否在主车辆后方
			if (!potentialObstacleLaneId.Empty()) {//检查潜在障碍物的车道ID是否为空

				SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);//获取潜在障碍物在车道上的s和t值
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);//获取主车辆在车道上的s和t值

				isObstalceBehind = !(sMainVehicle >= sObstalce);//判断潜在障碍物是否在主车辆后方（即潜在障碍物的s值小于主车辆的s值）
			}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();//创建一个SimOne_Data_Control类型的智能指针

			//主车控制
			pControl->throttle = 0.2f;                             //油门控制，范围为0到1，0表示完全松开，1表示全油门
			pControl->brake = 0;                                   //刹车控制，范围为0到1，0表示完全松开，1表示最大刹车
			pControl->steering = 0;                                //方向盘控制，范围为-1到1，-1表示向左打满，1表示向右打满
			pControl->handbrake = 0;                               //手刹控制，0表示未踩下，1表示踩下手刹
			pControl->isManualGear = 0;                            //是否手动换挡，0表示自动换挡，1表示手动换挡
			pControl->gear = static_cast<ESimOne_Gear_Mode>(1);    //档位控制，根据实际需求设置相应的档位*/ 

			if (mainVehicleSpeed*3.6 >= 34)// 判断主车速度是否大于等于34km/h
			{
				/*如果速度大于等于34km/h，设置油门为0，刹车为0.2*/
				pControl->throttle = 0.0f;
				pControl->brake = 0.2f;
			}
			else
			{
				/* 如果速度小于34km / h，设置油门为0.3，刹车为0*/
				pControl->throttle = 0.3f;
				pControl->brake = 0.0f;
			}


			if (isObstalceBehind) {
				if (minDistance <= 11.5) {//如果障碍物距离小于等于11.5米
					if (obstacleSpeed >= 2)//如果障碍物速度大于等于2米/秒
					{
						case_num = 2;
					}
					else if (case_num == 0 && obstacleSpeed > 0 && obstacleSpeed < 2)//如果case_num为0，障碍物速度大于0且小于2米/秒
					{
						case_num = 3;
					}
					else if (case_num == 3 && obstacleSpeed == 0)//如果case_num为3，障碍物速度为0
					{
						goto FINNAL;
					}

					pControl->throttle = 0.0f; //设置油门为0
					pControl->brake = 1.0f;//设置刹车为1
				}
			}


			std::cout << "speed:" << mainVehicleSpeed << std::endl;//输出主车速度
		FINNAL:
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());//设置车辆控制信息
		}
		else {//如果未初始化完成，输出日志信息
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);//进入下一帧
	}
	return 0;
}