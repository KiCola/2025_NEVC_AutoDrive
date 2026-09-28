# ADAS 代码导览

本目录保存参赛时的 ADAS 源码。`LKASample.cpp` 是自动驾驶测试场景的主要实现，AEB 和 AVP 代码也参与了 ADAS 任务。这里按原始提交保留源码，不将其描述成可独立编译的完整工程。

| 文件 | 主要内容 |
| --- | --- |
| [LKASample.cpp](LKASample.cpp) | 读取 GPS、障碍物、车道和交通灯信息；处理测试场景中的行驶、换道及避障决策，并向 SimOne 输出控制指令。 |
| [AEBSample.cpp](AEBSample.cpp) | 使用障碍物和道路信息执行自动紧急制动控制。 |
| [AVPPlanner.cpp](AVPPlanner.cpp) | 根据停车位几何和车辆参数生成倒车、前进及驶离轨迹。 |

## 阅读入口

建议先看 `LKASample.cpp` 的 `main()`：从 SimOne 初始化、读取场景数据，到生成控制指令，可以看到完整场景流程。AEB 是相对独立的控制示例；AVP 文件聚焦轨迹规划实现。

## 工程依赖与限制

- 源码使用 SimOne SDK 的服务、传感器、高精地图及评测 API；仓库没有包含 SDK。
- `LKASample.cpp` 还引用 `UtilDriver.h`、`UtilMath.h`、`SampleGetNearMostLane.h`、`SampleGetLaneST.h` 和 `utilTargetLane.h` 等原工程头文件。
- `AVPPlanner.cpp` 依赖 `AVPPlanner.hpp`、`AVPLog.hpp`、`UtilMath.hpp` 及 Eigen；这些头文件未在仓库中提供。
- 因缺少上述依赖，本目录没有可验证的独立构建命令。要在 SimOne 环境中使用，需要把源码放回具备匹配 SDK 和原工程头文件的项目中。

这些文件属于仿真竞赛场景代码，不能直接用于实车控制。
