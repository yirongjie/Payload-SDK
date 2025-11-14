/**
 ********************************************************************
 * @file    waypoint_mission_handler.hpp
 * @brief   PSDK Waypoint V3 航点任务处理器 (已从 V2 迁移)
 *********************************************************************
 */

#ifndef WAYPOINT_MISSION_HANDLER_HPP
#define WAYPOINT_MISSION_HANDLER_HPP

#include "dji_typedef.h"
#include "dji_waypoint_v3.h" // [修改] 关键头文件
#include "dji_platform.h"
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>

class WaypointMissionHandler
{
public:
    WaypointMissionHandler();
    ~WaypointMissionHandler();

    /**
     * @brief 初始化航点 V3 服务
     * @return true 成功, false 失败
     */
    bool init();

    /**
     * @brief [修改] 从文件路径加载 KMZ 航线文件并执行 (阻塞直到任务完成)
     * @param kmzFilePath   在妙算 3 本地文件系统中的 KMZ 文件完整路径
     * @return T_DjiReturnCode 操作结果
     */
    T_DjiReturnCode FlyKmzFile(const std::string &kmzFilePath);

private:
    /**
     * @brief 航点 V3 任务状态回调 (来自 test_waypoint_v3.c)
     */
    static T_DjiReturnCode MissionStateCallback(T_DjiWaypointV3MissionState missionState);

    /**
     * @brief 航点 V3 动作状态回调 (来自 test_waypoint_v3.c)
     */
    static T_DjiReturnCode ActionStateCallback(T_DjiWaypointV3ActionState actionState);

    bool m_isInitialized;
    T_DjiOsalHandler *m_osalHandler;
    
    // V3 API 不需要 mission ID 计数器

    // [新增] 用于任务完成的同步
    std::mutex m_mission_mutex;
    std::condition_variable m_mission_cv;
    std::atomic<bool> m_isMissionRunning;
    
    // [新增] 静态实例指针，用于回调
    static WaypointMissionHandler *s_instance;

    T_DjiReturnCode stopMission();
};

#endif // WAYPOINT_MISSION_HANDLER_HPP