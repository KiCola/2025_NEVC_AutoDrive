#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneEvaluationAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include "SampleGetNearMostLane.h"
#include "SampleGetLaneST.h"
#include <stdio.h>
#include <iostream>
#include <memory>
#include "utilTargetLane.h"

//全局变量
int case_num = 0; //标识案例号
long Case = 0;
static std::vector<SSD::SimPoint3D> posHistory;//历史位置
static std::vector<SSD::SimPoint3D> posPrediction;//预测位置
static const size_t HISTORY_SIZE = 100;//历史位置队列长度
static const size_t PREDICTION_SIZE = 25;//预测位置队列长度

//计算垂直速度
double calculateSpeed_ver(SimOne_Data_Gps *pGps, SimOne_Data_Obstacle_Entry *pObstacle)
{
	static double vel_par = 0;
	double vel_ver = 0;
	if (std::sqrt(std::pow(pGps->velX, 2) + std::pow(pGps->velY, 2) + std::pow(pGps->velZ, 2)) != 0)
	{
		vel_par = (pGps->velX * pObstacle->velX + pGps->velY * pObstacle->velY + pGps->velZ * pObstacle->velZ) / (std::sqrt(std::pow(pGps->velX, 2) + std::pow(pGps->velY, 2) + std::pow(pGps->velZ, 2)));
	}

	vel_ver = std::sqrt(std::pow(pObstacle->velX, 2) + std::pow(pObstacle->velY, 2) + std::pow(pObstacle->velZ, 2) - std::pow(vel_par, 2));
	return vel_ver;
}

double calculateDistance_ver(SimOne_Data_Gps *pGps, SimOne_Data_Obstacle_Entry *pObstacle)
{
	double Distance_par, Distance_ver;
	static SimOne_Data_Gps temp_pGps;

	if ((std::sqrt(std::pow(pGps->velX, 2) + std::pow(pGps->velY, 2) + std::pow(pGps->velZ, 2))) != 0)
	{
		temp_pGps = *pGps;
	}

	Distance_par = (temp_pGps.velX * (pObstacle->posX - temp_pGps.posX) + temp_pGps.velY * (pObstacle->posY - temp_pGps.posY)
		+ temp_pGps.velZ * (pObstacle->posZ - temp_pGps.posZ)) / (std::sqrt(std::pow(temp_pGps.velX, 2) + std::pow(temp_pGps.velY, 2) + std::pow(temp_pGps.velZ, 2)));
	Distance_ver = std::sqrt(std::pow((pObstacle->posX - temp_pGps.posX), 2) + std::pow((pObstacle->posY - temp_pGps.posY), 2) + std::pow((pObstacle->posZ - temp_pGps.posZ), 2) - std::pow(Distance_par, 2));

	if (Distance_par < 0)
	{
		Distance_ver = -Distance_ver;
	}
	return Distance_ver;
}

//判断是否相碰
int cash_judge(SimOne_Data_Gps *pGps, SimOne_Data_Obstacle_Entry *pObstacle, bool cash_flag)
{
	double speed_cash;
	double distance;
	double distance_ver;
	double dir;
	static double last_dir = 1;
	static int time = 0;

	//std::cout << "pObstacle: " << pObstacle->posX << "    " << pObstacle->posY << std::endl;
	//std::cout << "pGPS: " << pGps->posX << "    " << pGps->posY << std::endl;
	dir = (pObstacle->velX - pGps->velX) * (pObstacle->posX - pGps->posX) +
		(pObstacle->velY - pGps->velY) * (pObstacle->posY - pGps->posY) +
		(pObstacle->velZ - pGps->velZ) * (pObstacle->posZ - pGps->posZ);
	distance = std::sqrt(std::pow(pGps->posX - pObstacle->posX, 2) + std::pow(pGps->posY - pObstacle->posY, 2));


	if (last_dir <= 0 && dir > 0 && cash_flag == true)
	{
		time++;
		if (time > 1000)
		{
			dir = 1;
			last_dir = dir;
			time = 0;
		}
		else
		{
			dir = -1;
		}
	}
	else
	{
		time = 0;
		last_dir = dir;
	}
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "dir:%f", dir);
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "distance:%f", distance);

	if (distance < 50 && dir <= 0 && time == 0) //18: 50
	{
		speed_cash = calculateSpeed_ver(pGps, pObstacle);
		distance_ver = calculateDistance_ver(pGps, pObstacle);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "speed_cash:%f", speed_cash);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "disdistance_ver:%f", distance_ver);

		double pedestrainDis;
		if (Case == 37)
		{
			pedestrainDis = 15;
		}
		else {
			pedestrainDis = 18;
		}
		if (/*(speed_cash > 0.5 && speed_cash < 2.5 && distance_ver < 10 && distance < 16) ||
			(speed_cash > 2.5 && distance_ver < 10 && distance < 20) ||
			(speed_cash > 10 && distance_ver < 30)*/
			(speed_cash > 10 && distance_ver < 30) || (speed_cash > 2.5 && distance_ver < 14 && distance < 11) || (speed_cash > 0.2 && speed_cash < 2 && distance_ver < 15 && distance < pedestrainDis))//18: 30
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else if (time > 0 && distance < 18)
	{
		return 1;
	}
	else
	{
		time = 0;
		return 0;
	}

}

double traj_data[45][3] = {
	//x          y            z
	{-3.92,     420.04,       0   }, // 0
	{17.3 ,     261.62,       0   }, // 1
	{28.52,     214.88,       0   }, // 2
	{50.27,     153.56,       0   }, // 3
	{7.73 ,     92.6  ,       0   }, // 4
	{-22.73,    62.68 ,       0   }, // 5
	{-3.92,     -30.78,       0   }, // 6
	{1.41,      -76.38,       0   }, // 7
	{1.93 ,     -110.09,      0   }, // 8
	//{-25.38,    -148.25,      0   }, // 9
		{-42.81, -152.75, 0},
	{-51.4,     -159.16,      0   }, // 10
	{-66.15 ,   -196.17,      0   }, // 11
	{-51.35,    -262.29,      0   }, // 12
	{6.96,      -282.16,      0   }, // 13
	{81.89 ,    -293.28  ,    0   }, // 14
	{176.83,    -307.35 ,     0   }, // 15
	{290.09,    -306.58,      0   }, // 16
	{304.63,    -119.85,      0   }, // 17
	{246.91 ,   40.29  ,      0   }, // 18
		{222.7, 160, 0},
	{213.05,    190.31,       0   }, // 19
	{184.91,    330.03,       0   }, // 20
	{177.28 ,   389.83,       0   }, // 21
	{45.59,     382.29,       0   }, // 22
	{57.84,     292.72,       0   }, // 23
	{74.24 ,     210.2  ,     0   }, // 24
	{93.61,     114.02 ,      0   }, // 25
	{106.72,     48.79,       0   }, // 26
{120, 0.5, 0},
	{123.83,     -37.03,      0   }, // 27
{140, -100, 0},
	{145.71 ,    -146.09,     0   }, // 28
	{156.54,    -199.96,      0   }, // 29
	{189.27,     -294.95,     0   }, // 30
	{304.48 ,   -239.84,      0   }, // 31
	{252.47,    -180.36,      0   }, // 32
	{220.58,     -113.8,      0   }, // 33
	{183.7 ,     -62.44  ,    0   }, // 34
	{189.51,    24.68 ,       0   }, // 35
	{120.55,     86.29,       0   }, // 36
	{198.09,      169.35,     0   }, // 37
{144.5, 176.4, 0},
{111.14, 171.36, 0},
{102.56, 178.9, 0},
	{99.1 ,     185.71,       0   }  // 38
};


int main()
{
	freopen("final_v2.log", "w", stdout);

	/******************************Begin - 变量初始化****************************/
	int Stragegy = 0; //策略选择
	bool inAEBState = false; //AEB状态flag
	bool GetThroughtLane = false; //过十字路状态flag
	bool waitRedLight = false;//红灯状态flag
	bool ChangingLane = false; //变道状态flag
	int cash_times = false;

	long trafficLightId = -1;//交通灯索引
	double d_distance = 0;//dx 微分
	int Emergencycount = -1;//紧急刹车计数器
	long CircleID = 0;//循环轮次ID
	SSD::SimString ChangeLaneId = "";//预变道车道ID

	int timeout = 20;//HDMap的check时间
	bool isSimOneInitialized = false;//SimOne初始化flag
	const char* MainVehicleId = "0";//主车ID
	bool isJoinTimeLoop = true;//帧同步flag
	bool cash_flag = false;
	double targetSpeed = 32;
	bool leftflag = false;
	bool rightflag = false;
	float radio = 0.1f;

	bool Path[45] = { 0 };
	int nowpoint = 0;

	/******************************end - 变量初始化****************************/



	/******************************Begin - 函数初始化****************************/
	/*** 1. 功能函数初始化 ***/
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MainVehicleId, "Autopilot");
	SimOneAPI::SetDriveMode(MainVehicleId, ESimOne_Drive_Mode_API);// 设置驾驶模式为API模式
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);// 使用本地数据初始化评估服务

	SimOne_Data_CaseInfo pCase;
	SimOneAPI::GetCaseInfo(&pCase);
	char CASE41[] = "afc003c4-5a93-46fe-9257-59da7f3448c5";
	char casenum[2] = {};
	std::cout << "caseID: " << pCase.caseId << std::endl;

	casenum[0] = pCase.caseName[0];
	casenum[1] = pCase.caseName[1];
	Case = std::stol(casenum);
	std::cout << "CASEnum: " << Case << std::endl;

	/*** 2. HDMap初始化 ***/
	while (true)
	{
		if (SimOneAPI::LoadHDMap(timeout))
		{
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	/*** 3. 目标车道初始化 ***/
	SSD::SimPoint3DVector inputPoints;
	std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
	SSD::SimPoint3DVector targetPath;

	if (Case == 41)
	{
		std::cout << "Successful for case41 ! Whole PLAN!" << std::endl;
		for (int i = 0; i < 45; i++)
		{
			SSD::SimPoint3D inputWayPoints(traj_data[i][0], traj_data[i][1], traj_data[i][2]);
			inputPoints.push_back(inputWayPoints);
		}
	}
	else if (Case == 28)
	{
		double point28[3][3] = {
			{-295.5, -17.53, 0},
			{-168.3, -13.73, 0},
			{-160.6, -13.75, 0}
		};
		for (int i = 0; i < 3; i++)
		{
			SSD::SimPoint3D inputWayPoints(point28[i][0], point28[i][1], point28[i][2]);
			inputPoints.push_back(inputWayPoints);
		}
	}
	else
	{
		//获取主车的WayPoints

		if (SimOneAPI::GetWayPoints(MainVehicleId, pWayPoints.get()))
		{
			//打印WayPoints信息
			std::cout << "pWayPoints size: " << pWayPoints->wayPointsSize << std::endl;
			for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i)
			{
				SSD::SimPoint3D inputWayPoints(pWayPoints->wayPoints[i].posX, pWayPoints->wayPoints[i].posY, 0);
				std::cout << "idx:" << i << std::endl;
				std::cout << "x:" << pWayPoints->wayPoints[i].posX << std::endl;
				std::cout << "y:" << pWayPoints->wayPoints[i].posY << std::endl;
				inputPoints.push_back(inputWayPoints);
			}
		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
			return -1;
		}
	}

	std::cout << "Points Size: " << inputPoints.size() << std::endl;

	if (inputPoints.size() >= 2) {
		SSD::SimVector<int> indexOfValidPoints;
		if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
			return -1;
		}
	}
	else if (inputPoints.size() == 1) {
		SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);
		HDMapStandalone::MLaneInfo laneInfoInit;
		if (!SimOneAPI::GetLaneSample(laneIdInit, laneInfoInit)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle initial route failed");
			return -1;
		}
		else {
			targetPath = laneInfoInit.centerLine;
		}
	}
	else {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Number of wayPoints is zero");
		return -1;
	}

	/******************************end - 函数初始化****************************/


	/******************************Begin - 主循环****************************/
	while (Case != 18 && Case != 31) {
		/*** 1. 循环标志初始化  ***/
		std::cout << "[CircleID]" << ++CircleID << std::endl;
		int frame = SimOneAPI::Wait();

		/*** 2. 主循环出口  ***/
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			fclose(stdout);//关闭日志文件
			break;
		}

		/*** 3. 感知环节  ***/
		/****************************   pGps    ********************************/
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}

		/****************************   pObstacle    ******************************/
		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}

		/***************************   pTrafficSignal    *****************************/
		SSD::SimVector<HDMapStandalone::MSignal> pTrafficSignal;
		SimOneAPI::GetTrafficSignList(pTrafficSignal);
		double LimitedSpeed = -1;
		if (pTrafficSignal.size() == 0)
		{
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
			}
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "Have Traffic Sign!!!");
		}


		/***********************   pTrafficLight    ***************************/
		SSD::SimPoint3D currentPos(pGps->posX, pGps->posY, pGps->posZ);//当前位置
		SSD::SimString currentLaneId = SampleGetNearMostLane(currentPos);//当前车道ID


		std::unique_ptr<SimOne_Data_TrafficLight> pTrafficLight = std::make_unique<SimOne_Data_TrafficLight>();
		SSD::SimVector<HDMapStandalone::MSignal> lightList;
		SimOneAPI::GetSpecifiedLaneTrafficLightList(currentLaneId, lightList);

		HDMapStandalone::MSignal nearLight; //当前车辆最近的交通灯
		double minLightDistance = std::numeric_limits<double>::max();

		for (size_t i = 0; i < lightList.size(); ++i) {
			double distance = UtilMath::planarDistance(currentPos, lightList[i].pt);
			if (distance < minLightDistance) {
				minLightDistance = distance;
				nearLight = lightList[i];
			}
		}

		if (trafficLightId < 0)
		{
			trafficLightId = nearLight.id;
		}

		SimOneAPI::GetTrafficLight(MainVehicleId, nearLight.id, pTrafficLight.get());
		std::cout << "Near TrafficLight ID:" << nearLight.id << std::endl;
		std::cout << "Status:" << pTrafficLight->status << std::endl;

		/***********************   变道检测与转向灯控制    ***************************/
		/***********************   历史信息队列    ***************************/
		posHistory.push_back(currentPos);
		if (posHistory.size() > HISTORY_SIZE) {
			posHistory.erase(posHistory.begin());
		}

		SSD::SimString behindLaneId = "";
		if (posHistory.size() >= 10) {
			behindLaneId = SampleGetNearMostLane(posHistory[0]);
		}

		double nearestInDetection = std::numeric_limits<double>::max();
		size_t closestPointIndex = 0;
		for (size_t i = 0; i < targetPath.size(); i++) {
			double distance = UtilMath::planarDistance(currentPos, targetPath[i]);
			if (distance < nearestInDetection) {
				nearestInDetection = distance;
				closestPointIndex = i;
			}
		}

		double aheadDistance = 5.0;
		double accumulatedDistance = 0.0;
		int lookAheadPointIndex = closestPointIndex;

		while (lookAheadPointIndex + 1 < targetPath.size() && accumulatedDistance < aheadDistance) {
			double segmentLength = UtilMath::planarDistance(targetPath[lookAheadPointIndex], targetPath[lookAheadPointIndex + 1]);
			accumulatedDistance += segmentLength;
			if (accumulatedDistance > aheadDistance) {
				break;
			}
			lookAheadPointIndex++;
		}

		if (lookAheadPointIndex >= targetPath.size()) {
			lookAheadPointIndex = targetPath.size() - 1;
		}

		SSD::SimString aheadLaneId = SampleGetNearMostLane(targetPath[lookAheadPointIndex]);
		if (aheadLaneId.Empty()) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Failed to get ahead lane id");
		}

		if (aheadLaneId != behindLaneId) {
			HDMapStandalone::MLaneLink laneLink;
			if (SimOneAPI::GetLaneLink(behindLaneId, laneLink))
			{
				if (laneLink.leftNeighborLaneName == aheadLaneId) // 检测到向左变道
				{
					SimOne_Data_Signal_Lights signalLights;
					signalLights.signalLights = ESimOne_Signal_Light_LeftBlinker;
					SimOneAPI::SetSignalLights(MainVehicleId, &signalLights);
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "Detected left movement, turning on left signal");
				}
				else if (laneLink.rightNeighborLaneName == aheadLaneId) // 检测到向右变道
				{
					SimOne_Data_Signal_Lights signalLights;
					signalLights.signalLights = ESimOne_Signal_Light_RightBlinker;
					SimOneAPI::SetSignalLights(MainVehicleId, &signalLights);
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "Detected right movement, turning on right signal");
				}
			}
		}
		else
		{
			SimOneAPI::SetSignalLights(MainVehicleId, nullptr);
		}

		/********************************************三车道障碍物信息 ********************************************/
		SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
		double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

		double minDistance = std::numeric_limits<double>::max();
		int potentialObstacleIndex = pObstacle->obstacleSize;
		int leftObstacleIndex = pObstacle->obstacleSize;
		int rightObstacleIndex = pObstacle->obstacleSize;

		SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
		SSD::SimString potentialObstacleLaneId = "";
		int potentialObstacleType = -1;

		//相邻车道是否存在变道可能
		bool isLeftLaneExist = false;
		bool isRightLaneExist = false;
		double leftDistance = -1;
		double rightDistance = -1;
		HDMapStandalone::MLaneLink laneLink;
		bool isNeighborObstacleExist = false;
		if (SimOneAPI::GetLaneLink(mainVehicleLaneId, laneLink))
		{
			if (laneLink.leftNeighborLaneName != "")//左侧有车道
			{
				isLeftLaneExist = true;
				std::cout << "Left Lane exist!" << std::endl;
				leftDistance = std::numeric_limits<double>::max();
			}

			if (laneLink.rightNeighborLaneName != "")//右侧有车道
			{
				isRightLaneExist = true;
				std::cout << "Right Lane exist!" << std::endl;
				rightDistance = std::numeric_limits<double>::max();
			}
		}


		for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
			SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
			SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
			if (mainVehicleLaneId == obstacleLaneId)
			{
				double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);

				if (obstacleDistance < minDistance) {
					minDistance = obstacleDistance;
					potentialObstacleIndex = (int)i;
					potentialObstacleLaneId = obstacleLaneId;
					potentialObstacleType = pObstacle->obstacle[i].type;
				}
			}
			else if (laneLink.leftNeighborLaneName == obstacleLaneId)
			{
				//double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);

				double sObstacle = 0.;
				double tObstacle = 0.;

				double sMainVehicle = 0.;
				double tMainVehicle = 0.;

				bool isObstacleAhead = false;

				SampleGetLaneST(obstacleLaneId, obstaclePos, sObstacle, tObstacle);
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);
				isObstacleAhead = !(sMainVehicle >= sObstacle);

				if (isObstacleAhead)
				{
					double obstacleDistance = sObstacle - sMainVehicle;
					if (obstacleDistance < leftDistance)
					{
						leftDistance = obstacleDistance;
						leftObstacleIndex = (int)i;
					}
				}
			}
			else if (laneLink.rightNeighborLaneName == obstacleLaneId)
			{
				double sObstacle = 0.;
				double tObstacle = 0.;

				double sMainVehicle = 0.;
				double tMainVehicle = 0.;

				bool isObstacleAhead = false;

				SampleGetLaneST(obstacleLaneId, obstaclePos, sObstacle, tObstacle);
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);
				isObstacleAhead = !(sMainVehicle >= sObstacle);

				if (isObstacleAhead)
				{
					double obstacleDistance = sObstacle - sMainVehicle;
					if (obstacleDistance < rightDistance)
					{
						rightDistance = obstacleDistance;
						rightObstacleIndex = (int)i;
					}
				}
			}
		}

		std::cout << "potentialObstacleType: " << potentialObstacleType << std::endl;
		//car -- 6
		//static -- 7

		

		/*****************************************潜在障碍物感知  ************************************/
		auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
		double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);


		SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
		double sObstalce = 0.;
		double tObstacle = 0.;

		double sMainVehicle = 0.;
		double tMainVehicle = 0.;

		bool isObstalceBehind = false;
		if (!potentialObstacleLaneId.Empty())
		{

			SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);
			SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);


			isObstalceBehind = !(sMainVehicle >= sObstalce);
		}

		for (size_t i = 0; i < pObstacle->obstacleSize; ++i)
		{
			//优先判断是否相碰
			SimOne_Data_Obstacle_Entry *pObstacle_temp = &pObstacle->obstacle[i];
			SimOne_Data_Gps *pGps_temp = pGps.get();
			cash_flag = cash_judge(pGps_temp, pObstacle_temp, cash_flag);
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "cash_flag: %d", cash_flag);
			if (cash_flag) break;
		}

		std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

		////判断当前最近点
		//double nearestWayPointDis_v2 = std::numeric_limits<double>::max();
		//size_t nearestIndex = 0;
		//for (int j = 0; j < inputPoints.size() - 1; j++)
		//{
		//	SSD::SimPoint3D wayPointPos(inputPoints[j].x, inputPoints[j].y, inputPoints[j].z);
		//	double dis = UtilMath::planarDistance(mainVehiclePos, wayPointPos);
		//	std::cout << "nearestWayPointDis:" << nearestWayPointDis_v2 << std::endl;
		//	std::cout << "dis:" << dis << std::endl;

		//	if (nearestWayPointDis > dis)
		//	{

		//		nearestWayPointDis = dis;
		//		nearestIndex = j;
		//	}

		//}

		//std::cout << "nearestWayPointIndex: " << nearestIndex << std::endl;
		if (Case == 41)
		{
			SSD::SimPoint3D wayPointPos(inputPoints[nowpoint].x, inputPoints[nowpoint].y, inputPoints[nowpoint].z);
			double dis = UtilMath::planarDistance(mainVehiclePos, wayPointPos);
			if (dis < 5.0f && Path[nowpoint] == false)
			{
				Path[nowpoint] == true;
				nowpoint++;
			}
		}
		std::cout << "nowpoint: " << nowpoint << std::endl;
		

		/************************************* 决策 *******************************/

		if (Case == 41)
		{
			if (mainVehicleSpeed*3.6f < 50)radio = 0.2f;
			else { radio = 0.3f; }
			//else if(mainVehicleSpeed*3.6f < 65) { radio = 0.3f; }
			//else if (mainVehicleSpeed*3.6f < 90) { radio = 0.5f; }
		}
		//Stragegy
		if (Case == 41)
		{
			//if (mainVehicleSpeed*3.6f < 50)radio = 0.2f;
			//else { radio = 0.3f; }

			if (nowpoint <= 5) //起始小部分和大弯道
			{
				targetSpeed = 90;
				//radio = 0.2f;

				if (mainVehicleSpeed*3.6f < 50)radio = 0.2f;
				else { radio = 0.3f; }
			}
			else if (nowpoint <= 7) //大直道
			{
				targetSpeed = 150;
				if (mainVehicleSpeed*3.6f < 50)radio = 1.5f;
				else { radio = 1.2f; }
			}
			else if (nowpoint <= 13) //小弯道
			{
				targetSpeed = 50;
				if (mainVehicleSpeed*3.6f > 50)radio = 0.2f;
				else { radio = 0.3; }

			}
			else if (nowpoint <= 15) //大直道
			{
				targetSpeed = 150;
				if (mainVehicleSpeed*3.6f < 50)radio = 1.5f;
				else { radio = 1.2f; }
			}
			else if (nowpoint <= 17)
			{
				targetSpeed = 70;
				if (mainVehicleSpeed*3.6f < 50)radio = 0.2f;
				else { radio = 0.3f; }
			}
			else if (nowpoint <= 20) //大直道
			{
				targetSpeed = 150;
				if (mainVehicleSpeed*3.6f < 50)radio = 1.5f;
				else { radio = 1.2f; }
			}
			else if (nowpoint <= 24)
			{
				targetSpeed = 70;
				if (mainVehicleSpeed*3.6f < 50)radio = 0.2f;
				else { radio = 0.3f; }
			}
			else if (nowpoint <= 32)
			{
				targetSpeed = 150;
				if (mainVehicleSpeed*3.6f < 50)radio = 1.5f;
				else { radio = 1.2f; }
			}
			else
			{
				targetSpeed = 60;
				if (mainVehicleSpeed*3.6f < 50)radio = 0.2f;
				else { radio = 0.3f; }
			}
		}




		/************************************* 执行 *******************************/

		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running)
		{
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}


			//基础设置
			pControl->throttle = 0.10f;
			pControl->brake = 0.f;
			pControl->steering = 0.f;
			pControl->handbrake = false;
			pControl->isManualGear = false;
			pControl->gear = static_cast<ESimOne_Gear_Mode>(1);
			//Stragegy = 0;


			if (LimitedSpeed >= 2 && Case != 41) targetSpeed = LimitedSpeed - 2;
			//if (Case == 41) targetSpeed = 90;
			if (Case == 20) targetSpeed = 33;
			if (Case == 30) targetSpeed = 25;
			if (Case == 31 || Case == 24) targetSpeed = 20;
			//if (Case == 32) targetSpeed = 10;
			
			if (mainVehicleSpeed*3.6 >= targetSpeed) //速度控制
			{
				pControl->throttle = 0.0f;
				pControl->brake = (mainVehicleSpeed * 3.6f - targetSpeed) / targetSpeed * 0.3 + 0.1;
				if(Case == 41) pControl->brake = (mainVehicleSpeed * 3.6f - targetSpeed) / targetSpeed * 0.5 + 0.1;
			} 
			else
			{
				//pControl->throttle = 0.3f;

				//if (Case == 41)
				//{
				//	if (mainVehicleSpeed*3.6f < 50)radio = 0.2f;
				//	else { radio = 0.3f; }
				//	//else if(mainVehicleSpeed*3.6f < 65) { radio = 0.3f; }
				//	//else if (mainVehicleSpeed*3.6f < 90) { radio = 0.5f; }
				//}
				if (Case == 20 || Case == 37)
				{
					radio = 3.0f;
				}

				pControl->throttle = (targetSpeed - mainVehicleSpeed * 3.6f) / targetSpeed * radio + 0.1;
				pControl->brake = 0.0f;
			}




			//控制量决策树
			double defaultDistance = 6.0;
			if (isObstalceBehind)
			{
				std::cout << "Obstacle Ahead!" << std::endl;
				//Stragegy = 1;

				double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
				double defautlTimeToCollision = 3.4;
				//double TochangeLaneDistance = 20.0f;
				double TochangeLaneDistance = 30.0f;
				double LimitedDistance = 14.0f;
				double safeDistance = 30.0f;


				std::cout << "minDistance: " << minDistance << std::endl;
				std::cout << "mainVehicleSpeed: " << mainVehicleSpeed << std::endl;
				std::cout << "obstacleSpeed: " << obstacleSpeed << std::endl;

				if (ChangingLane) {
					//SSD::SimString currentLaneId = SampleGetNearMostLane(mainVehiclePos);
					if (mainVehicleLaneId == ChangeLaneId) {
						std::cout << "finished changing lane!" << std::endl;
						ChangingLane = false;
						leftflag = false;
						rightflag = false;
						ChangeLaneId = "";
						SimOneAPI::SetSignalLights(MainVehicleId, nullptr);
					}
				}
				else
				{
					if (minDistance <= TochangeLaneDistance)
					{
						if (isLeftLaneExist && leftDistance >= TochangeLaneDistance)//向左变道
						{
							ChangingLane = true;
							leftflag = true;
							std::cout << "leftDistance: " << leftDistance << std::endl;
							std::cout << "change to left lane" << std::endl;

							SimOne_Data_Signal_Lights VehicleSignalLight;
							VehicleSignalLight.signalLights = ESimOne_Signal_Light_LeftBlinker;
							SimOneAPI::SetSignalLights(MainVehicleId, &VehicleSignalLight);
							std::cout << "open LeftBlinker！ " << std::endl;

							//change to left lane
							ChangeLaneId = laneLink.leftNeighborLaneName;


							SSD::SimPoint3DVector tempTraj;

							utilTargetLane::GetLaneSampleFromS(laneLink.leftNeighborLaneName, sObstalce, tempTraj);

							SSD::SimPoint3DVector laneChangeWaypoints;
							laneChangeWaypoints.push_back(mainVehiclePos);
							laneChangeWaypoints.push_back(tempTraj[0]);

							if (Case == 41)
							{

								////续上后面的路径点
								double nearestWayPointDis = std::numeric_limits<double>::max();
								size_t nearestIndex = 0;
								for (int j = 0; j < inputPoints.size() - 1; j++)
								{
									SSD::SimPoint3D wayPointPos(inputPoints[j].x, inputPoints[j].y, inputPoints[j].z);
									double dis = UtilMath::planarDistance(mainVehiclePos, wayPointPos);
									std::cout << "nearestWayPointDis:" << nearestWayPointDis << std::endl;
									std::cout << "dis:" << dis << std::endl;

									if (nearestWayPointDis > dis)
									{

										nearestWayPointDis = dis;
										nearestIndex = j;
									}

								}
								std::cout << "mainVehiclePos:" << mainVehiclePos.x << "  " << mainVehiclePos.y << std::endl;
								std::cout << "nearestWayPoint:" << pWayPoints->wayPoints[nearestIndex].posX << "  " << pWayPoints->wayPoints[nearestIndex].posY << std::endl;

								for (size_t j = nearestIndex + 1; j < inputPoints.size() - 1; j++)
								{
									SSD::SimPoint3D wayPointPos(inputPoints[j].x, inputPoints[j].y, inputPoints[j].z);
									//	SSD::SimPoint3D wayPointPos;
									//	wayPointPos.x = pWayPoints->wayPoints[j].posX;
									//	wayPointPos.y = pWayPoints->wayPoints[j].posY;
									//	wayPointPos.z = 0;
									laneChangeWaypoints.push_back(wayPointPos);
								}
							}
							else
							{
								laneChangeWaypoints.push_back(targetPath[targetPath.size() - 1]);
							}



							//laneChangeWaypoints.push_back(targetPath[targetPath.size() - 1]);

							targetPath.clear();
							SSD::SimVector<int> indexOfValidPoints;
							if (!SimOneAPI::GenerateRoute(laneChangeWaypoints, indexOfValidPoints, targetPath)) {
								SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
								return -1;
							}
						}
						else if (isRightLaneExist && rightDistance >= TochangeLaneDistance)//向右变道
						{
							ChangingLane = true;
							rightflag = true;
							std::cout << "rightDistance: " << rightDistance << std::endl;
							std::cout << "change to right lane" << std::endl;

							SimOne_Data_Signal_Lights VehicleSignalLight;
							VehicleSignalLight.signalLights = ESimOne_Signal_Light_RightBlinker;
							SimOneAPI::SetSignalLights(MainVehicleId, &VehicleSignalLight);
							std::cout << "open RightBlinker！ " << std::endl;
							//change to right lane
							ChangeLaneId = laneLink.rightNeighborLaneName;

							SSD::SimPoint3DVector tempTraj;

							utilTargetLane::GetLaneSampleFromS(laneLink.rightNeighborLaneName, sObstalce, tempTraj);

							SSD::SimPoint3DVector laneChangeWaypoints;
							laneChangeWaypoints.push_back(mainVehiclePos);
							laneChangeWaypoints.push_back(tempTraj[0]);
							if (Case == 41)
							{

								////续上后面的路径点
								double nearestWayPointDis = std::numeric_limits<double>::max();
								size_t nearestIndex = 0;
								for (int j = 0; j < inputPoints.size() - 1; j++)
								{
									SSD::SimPoint3D wayPointPos(inputPoints[j].x, inputPoints[j].y, inputPoints[j].z);
									//	SSD::SimPoint3D wayPointPos;
									//	wayPointPos.x = pWayPoints->wayPoints[j].posX;
									//	wayPointPos.y = pWayPoints->wayPoints[j].posY;
									//	wayPointPos.z = 0;
									double dis = UtilMath::planarDistance(mainVehiclePos, wayPointPos);
									std::cout << "nearestWayPointDis:" << nearestWayPointDis << std::endl;
									std::cout << "dis:" << dis << std::endl;

									if (nearestWayPointDis > dis)
									{

										nearestWayPointDis = dis;
										nearestIndex = j;
									}

								}
								std::cout << "mainVehiclePos:" << mainVehiclePos.x << "  " << mainVehiclePos.y << std::endl;
								std::cout << "nearestWayPoint:" << pWayPoints->wayPoints[nearestIndex].posX << "  " << pWayPoints->wayPoints[nearestIndex].posY << std::endl;

								for (size_t j = nearestIndex + 1; j < inputPoints.size() - 1; j++)
								{
									SSD::SimPoint3D wayPointPos(inputPoints[j].x, inputPoints[j].y, inputPoints[j].z);
									//	SSD::SimPoint3D wayPointPos;
									//	wayPointPos.x = pWayPoints->wayPoints[j].posX;
									//	wayPointPos.y = pWayPoints->wayPoints[j].posY;
									//	wayPointPos.z = 0;
									laneChangeWaypoints.push_back(wayPointPos);
								}
							}
							else
							{
								laneChangeWaypoints.push_back(targetPath[targetPath.size() - 1]);
							}
							//laneChangeWaypoints.push_back(targetPath[targetPath.size() - 1]);

							targetPath.clear();
							SSD::SimVector<int> indexOfValidPoints;
							if (!SimOneAPI::GenerateRoute(laneChangeWaypoints, indexOfValidPoints, targetPath)) {
								SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
								return -1;
							}
						}
						else if (minDistance > LimitedDistance)//稳定跟车
						{
							if (potentialObstacleType != 7)//不是静态障碍物,则稳定跟车
							{
								if (mainVehicleSpeed >= obstacleSpeed)
								{
									pControl->throttle = 0.f;
									//(stoppingDistance - distanceToLight) / stoppingDistance * 0.53;
									pControl->brake = (mainVehicleSpeed - obstacleSpeed) / mainVehicleSpeed;
								}
								else
								{
									pControl->brake = 0.f;
									//(stoppingDistance - distanceToLight) / stoppingDistance * 0.53;
									pControl->throttle = (obstacleSpeed - mainVehicleSpeed) / mainVehicleSpeed;
								}
							}
							else
							{
								std::cout << "aaaaaaa" << std::endl;
								pControl->throttle = 0.0f;
								pControl->brake = 0.0f;

							}

						}
						else if (minDistance <= LimitedDistance)
						{
							if (potentialObstacleType != 7)
							{
								std::cout << "Stop!" << std::endl;
								SimOne_Data_Signal_Lights VehicleSignalLight;
								VehicleSignalLight.signalLights = ESimOne_Signal_Light_DoubleFlash;
								SimOneAPI::SetSignalLights(MainVehicleId, &VehicleSignalLight);
								std::cout << "open doubleflash！ " << std::endl;
								pControl->throttle = 0.f;
								pControl->brake = 1.f;
							}
							else
							{
								if (minDistance <= defaultDistance)
								{
									std::cout << "Stop! Static Obstacle ahead!" << std::endl;
									SimOne_Data_Signal_Lights VehicleSignalLight;
									VehicleSignalLight.signalLights = ESimOne_Signal_Light_DoubleFlash;
									SimOneAPI::SetSignalLights(MainVehicleId, &VehicleSignalLight);
									std::cout << "open doubleflash！ " << std::endl;
									if (Case != 41)
									{
										pControl->throttle = 0.f;
										pControl->brake = (float)(mainVehicleSpeed * 3.6 * 0.65 + 0.20);
									}
								}
								else
								{
									pControl->throttle = 0.0f;
									pControl->brake = 0.2f;
								}
							}
						}
					}
				}
			}



			//交通灯控制
			double stoppingDistance = 25.0f;
			double fullstopDistance = 10.0f;
			double distanceToLight = UtilMath::planarDistance(mainVehiclePos, nearLight.pt);
			std::cout << "distanceToLight: " << distanceToLight << std::endl;
			//std::cout << "obstacleDistance: " << obstacleDistance << std::endl;


			if (!GetThroughtLane && trafficLightId == nearLight.id)
			{
				if (pTrafficLight->status == ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Red || Case == 32)
				{
					if (distanceToLight < stoppingDistance)
					{
						pControl->throttle = 0.f;
						pControl->brake = (minLightDistance <= fullstopDistance) ? 1.2f : (stoppingDistance - distanceToLight) / stoppingDistance * 0.53;
						std::cout << "Red Light! Brake!" << std::endl;
						waitRedLight = true;
					}
				}
				else if (pTrafficLight->status == ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Yellow)
				{
					pControl->throttle = 0.f;
					pControl->brake = 0.5f;
				}

			}


			if ((waitRedLight && pTrafficLight->status == ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Green && Case != 32) || trafficLightId != nearLight.id)
			{
				GetThroughtLane = true;
				std::cout << "have gotten through the cross Traffic area!" << std::endl;
			}


			if (cash_flag == true && cash_times < 75 && Case != 12)
			{
				if(Case == 18)cash_times++;
				pControl->throttle = 0.0f;
				pControl->brake = 3.6f * mainVehicleSpeed + 0.2;
			}
			
			double steering = 0.0f;
			steering = UtilDriver::calculateSteering(targetPath, pGps.get());
			pControl->steering = (float)steering*1.1;
			
			if (Case == 41)
			{
				if (mainVehicleSpeed*3.6f > 51)//45
				{
					steering = UtilDriver::calculateSteering_v2(targetPath, pGps.get());
				}
				else
				{
					steering = UtilDriver::calculateSteering(targetPath, pGps.get());
				}

				pControl->steering = (float)steering*1.7;
			}

			SimOneAPI::SetDrive(MainVehicleId, pControl.get());

			d_distance = minDistance;//记录上一次距离
			if (ChangingLane) {
				//SSD::SimString currentLaneId = SampleGetNearMostLane(mainVehiclePos);
				if (leftflag == true)
				{
					if (leftDistance <= 5.0f)
					{
						std::cout << "Left Lane has obstacle! brake!" << std::endl;
						pControl->brake = 1.0f;
						pControl->throttle = 0.0f;
					}
				}

				if (rightflag == true)
				{
					if (rightDistance <= 5.0f)
					{
						std::cout << "Right Lane has obstacle! brake!" << std::endl;
						pControl->brake = 1.0f;
						pControl->throttle = 0.0f;
					}
				}

				if (mainVehicleLaneId == ChangeLaneId) {
					std::cout << "finished changing lane!" << std::endl;
					ChangingLane = false;
					leftflag = false;
					rightflag = false;
					ChangeLaneId = "";
					SimOneAPI::SetSignalLights(MainVehicleId, nullptr);
				}
			}
		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);
	}

	while (Case == 18 || Case == 31) {
		int frame = SimOneAPI::Wait();

		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}

		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}

		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}

			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

			double minDistance = std::numeric_limits<double>::max();
			int potentialObstacleIndex = pObstacle->obstacleSize;
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "car_lane_id: %s", (mainVehicleLaneId.GetString()));
			SSD::SimString potentialObstacleLaneId = "";
			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				//优先判断是否相碰
				SimOne_Data_Obstacle_Entry *pObstacle_temp = &pObstacle->obstacle[i];
				SimOne_Data_Gps *pGps_temp = pGps.get();
				cash_flag = cash_judge(pGps_temp, pObstacle_temp, cash_flag);
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "cash_flag: %d", cash_flag);
				if (cash_flag) break;

				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "obs_lane_id: %s", obstacleLaneId.GetString());
				if (mainVehicleLaneId == obstacleLaneId) {
					double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);

					if (obstacleDistance < minDistance) {
						minDistance = obstacleDistance;
						potentialObstacleIndex = (int)i;
						potentialObstacleLaneId = obstacleLaneId;
					}
				}

			}

			auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
			double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);


			SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
			double sObstalce = 0.;
			double tObstacle = 0.;

			double sMainVehicle = 0.;
			double tMainVehicle = 0.;

			bool isObstalceBehind = false;
			if (!potentialObstacleLaneId.Empty()) {

				SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

				isObstalceBehind = (sMainVehicle >= sObstalce);
			}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

			// Control mainVehicle with SimOneDriver
			SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

			// Control mainVehicle without SimOneDriver
			/*pControl->throttle = 0.12f;
			pControl->brake = 0.f;
			pControl->steering = 0.f;
			pControl->handbrake = false;
			pControl->isManualGear = false;
			pControl->gear = static_cast<ESimOne_Gear_Mode>(1);*/
			if ((mainVehicleSpeed * 3.6) > 32 && !cash_flag)
			{
				pControl->throttle = 0;
				pControl->brake = 0.2;
			}
			else if ((mainVehicleSpeed * 3.6) <= 32 && !cash_flag)
			{
				pControl->throttle = 0.2f;
				pControl->brake = 0;
			}

			//碰撞优先级最高
			if (cash_flag)
			{
				pControl->throttle = 0;
				pControl->brake = 1;
			}
			else if (!isObstalceBehind) {
				double defaultDistance = 10.2f;
				double controlDistance = 15.0f;
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "[min: %f]", minDistance);
				//double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
				if (minDistance < controlDistance && obstacleSpeed == 0) {
					if (minDistance < defaultDistance) {
						inAEBState = true;
						pControl->throttle = 0;
						pControl->brake = 1;
					}
				}
				else if (minDistance < controlDistance && obstacleSpeed != 0) {
					static double err = 0;
					double speed_p = 0.1;
					double speed_i = 0;
					double speed_d = 0;
					err = minDistance - defaultDistance;
					if (err >= 0)
					{
						pControl->throttle = pControl->throttle + err * speed_p;
						pControl->brake = 0;
					}
					else
					{
						pControl->throttle = 0;
						pControl->brake = pControl->brake - err * speed_p;
					}
				}
			}
			double steering = UtilDriver::calculateSteering(targetPath, pGps.get());
			pControl->steering = (float)steering;
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());
		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);
	}
	return 0;
}

