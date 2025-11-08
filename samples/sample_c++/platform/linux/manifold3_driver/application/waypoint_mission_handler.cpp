/**
 ********************************************************************
 * @file    waypoint_mission_handler.cpp
 * @brief   WaypointMissionHandler 类的实现 (已从 V2 迁移到 V3)
 *********************************************************************
 */

#include "waypoint_mission_handler.hpp"
#include "dji_platform.h"
#include "dji_logger.h"
#include <string.h> // for memcpy
#include <stdio.h>  // for FILE, fopen, etc.

// [修复 1] 移除这一行，因为它导致了错误
// #include "utils/util_file.h" 

// [新增] 初始化静态实例
WaypointMissionHandler *WaypointMissionHandler::s_instance = nullptr;

WaypointMissionHandler::WaypointMissionHandler()
    : m_isInitialized(false), m_osalHandler(nullptr), m_isMissionRunning(false)
{
    // [新增]
    s_instance = this;
}

WaypointMissionHandler::~WaypointMissionHandler()
{
    if (m_isInitialized)
    {
        // [修改] V3 反初始化
        DjiWaypointV3_DeInit();
        USER_LOG_INFO("WaypointHandler(V3): Deinitialized.");
    }
    // [新增]
    s_instance = nullptr;
}


// ... (构造函数, 析构函数, init() 函数保持不变) ...
// ...
bool WaypointMissionHandler::init()
{
    T_DjiReturnCode ret;
    m_osalHandler = DjiPlatform_GetOsalHandler();
    if (!m_osalHandler)
    {
        USER_LOG_ERROR("WaypointHandler(V3): Failed to get OsalHandler.");
        return false;
    }

    ret = DjiWaypointV3_Init();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("WaypointHandler(V3): Failed to init DjiWaypointV3, ret=0x%08llX", ret);
        return false;
    }

    ret = DjiWaypointV3_RegMissionStateCallback(WaypointMissionHandler::MissionStateCallback);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("WaypointHandler(V3): Failed to register state callback, ret=0x%08llX", ret);
        return false;
    }

    ret = DjiWaypointV3_RegActionStateCallback(WaypointMissionHandler::ActionStateCallback);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("WaypointHandler(V3): Failed to register action state callback, ret=0x%08llX", ret);
        return false;
    }

    m_isInitialized = true;
    USER_LOG_INFO("WaypointHandler(V3): Initialized successfully.");
    return true;
}


/**
 * @brief [修改] 核心实现: "从 KMZ 文件飞行" (变为阻塞)
 * @note 此函数逻辑完全基于 test_waypoint_v3.c
 */
T_DjiReturnCode WaypointMissionHandler::FlyKmzFile(const std::string &kmzFilePath)
{
    if (!m_isInitialized)
    {
        USER_LOG_ERROR("WaypointHandler(V3): Cannot FlyKmzFile, not initialized.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }

    T_DjiReturnCode returnCode;
    FILE *kmzFile = NULL;
    uint32_t kmzFileSize = 0;
    uint8_t *kmzFileBuf = NULL;
    size_t readLen;

    // [新增] 任务开始前，设置运行标志
    {
        std::lock_guard<std::mutex> lock(m_mission_mutex);
        m_isMissionRunning = true; 
    }

    // 1. 打开 KMZ 文件
    kmzFile = fopen(kmzFilePath.c_str(), "r");
    if (kmzFile == NULL)
    {
        USER_LOG_ERROR("WaypointHandler(V3): Open kmz file failed: %s", kmzFilePath.c_str());
        m_isMissionRunning = false; // [新增] 错误退出
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    // 2. [修复 2] 使用标准 C 函数 fseek/ftell 获取文件大小
    fseek(kmzFile, 0, SEEK_END);
    long fileSize_long = ftell(kmzFile);
    if (fileSize_long <= 0) {
         USER_LOG_ERROR("WaypointHandler(V3): ftell failed or file is empty.");
         fclose(kmzFile);
         m_isMissionRunning = false; // [新增] 错误退出
         return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    kmzFileSize = (uint32_t)fileSize_long;
    fseek(kmzFile, 0, SEEK_SET); // 必须重置回文件开头
    
    // [移除] 导致错误的代码
    // returnCode = UtilFile_GetFileSize(kmzFile, &kmzFileSize);
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || kmzFileSize == 0)
    // {
    //    ...
    // }

    // 3. 分配内存 (来自 test_waypoint_v3.c)
    kmzFileBuf = (uint8_t *)m_osalHandler->Malloc(kmzFileSize);
    if (kmzFileBuf == NULL)
    {
        USER_LOG_ERROR("WaypointHandler(V3): Malloc kmz file buf error.");
        fclose(kmzFile);
        m_isMissionRunning = false; // [新增] 错误退出
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    // 4. 将文件完整读入内存 (来自 test_waypoint_v3.c)
    readLen = fread(kmzFileBuf, 1, kmzFileSize, kmzFile);
    if (readLen != kmzFileSize)
    {
        USER_LOG_ERROR("WaypointHandler(V3): Read kmz file data failed.");
        fclose(kmzFile);
        m_osalHandler->Free(kmzFileBuf);
        m_isMissionRunning = false; // [新增] 错误退出
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    // 5. 文件已读完，关闭
    fclose(kmzFile);
    kmzFile = NULL;

    // 6. (关键步骤) 上传 KMZ 文件 (来自 test_waypoint_v3.c)
    USER_LOG_INFO("WaypointHandler(V3): Uploading kmz file (Size: %d bytes)...", kmzFileSize);
    returnCode = DjiWaypointV3_UploadKmzFile(kmzFileBuf, kmzFileSize);
    
    // 7. 释放文件缓冲区
    m_osalHandler->Free(kmzFileBuf);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("WaypointHandler(V3): Upload kmz file failed, ret=0x%08llX", returnCode);
        m_isMissionRunning = false; // [新增] 错误退出
        return returnCode;
    }

    USER_LOG_INFO("WaypointHandler(V3): Upload success. Executing START action...");

    // 8. (关键步骤) 启动任务 (来自 test_waypoint_v3.c)
    returnCode = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("WaypointHandler(V3): Execute start action failed, ret=0x%08llX", returnCode);
        m_isMissionRunning = false; // [新增] 错误退出
        return returnCode;
    }

    USER_LOG_INFO("WaypointHandler(V3): Mission started successfully. Waiting for completion...");

    // 9. [新增] 等待任务完成
    std::unique_lock<std::mutex> lock(m_mission_mutex);
    m_mission_cv.wait(lock, [this]{ return !m_isMissionRunning.load(); });

    // --------------------------------------------------------------------------
    // 任务结束后，显式停止任务，确保飞控退出航点模式
    // --------------------------------------------------------------------------
    T_DjiReturnCode stopRet = stopMission(); 
    if (stopRet != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        // 记录警告，但仍返回成功，因为任务本身已完成
        USER_LOG_WARN("WaypointHandler(V3): Mission completed, but explicit STOP failed, ret=0x%08llX", stopRet);
    }
    // --------------------------------------------------------------------------

    USER_LOG_INFO("WaypointHandler(V3): Mission completion signal received. Returning.");
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


// [修改] V3 状态回调
T_DjiReturnCode WaypointMissionHandler::MissionStateCallback(T_DjiWaypointV3MissionState missionState)
{
    // (逻辑来自 test_waypoint_v3.c)
    // [修复 3] 使用 C++ 风格的零初始化 {}
    static T_DjiWaypointV3MissionState s_lastState = {}; 
    
    if (s_lastState.state == missionState.state
        && s_lastState.currentWaypointIndex == missionState.currentWaypointIndex
        && s_lastState.wayLineId == missionState.wayLineId) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    USER_LOG_INFO("Waypoint(V3) Mission State: %d, current waypoint index: %d, wayLine id: %d", 
                  missionState.state,
                  missionState.currentWaypointIndex, missionState.wayLineId);

    // [新增] 检查任务是否完成 (State 0)
    // 在您的日志中： 11-04 00:59:27.968 ... Mission State: 0
    if (missionState.state == 0 && s_lastState.state != 0) 
    {
        USER_LOG_INFO("Waypoint(V3) Mission State: FINISHED (State 0).");
        if (s_instance) 
        {
            // 使用 s_instance 访问非静态成员
            std::lock_guard<std::mutex> lock(s_instance->m_mission_mutex);
            s_instance->m_isMissionRunning = false;
            // 唤醒正在 FlyKmzFile 中等待的线程
            s_instance->m_mission_cv.notify_one(); 
        }
    }

    memcpy(&s_lastState, &missionState, sizeof(T_DjiWaypointV3MissionState));

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// [修改] V3 动作回调
T_DjiReturnCode WaypointMissionHandler::ActionStateCallback(T_DjiWaypointV3ActionState actionState)
{
    // (逻辑来自 test_waypoint_v3.c)
    USER_LOG_INFO(
        "Waypoint(V3) Action State: %d, current waypoint index: %d, wayLine id: %d, action group: %d, action id: %d",
        actionState.state,
        actionState.currentWaypointIndex, actionState.wayLineId,
        actionState.actionGroupId, actionState.actionId);

    // [!!!! 在此添加新代码 !!!!]
    // 这是一个修复 (Workaround):
    // 我们假设 "hover" 动作 (actionId=0) 是我们的最后一个动作。
    // 当这个动作 *开始执行* (actionState.state == 1) 时,
    // 我们就手动将任务标记为 "已完成", 以便释放 FlyKmzFile 中的锁。
    // 这将解决因 PSDK 在 hover 动作后不发送 missionState==0 而导致的死锁问题。
    
    // 检查: 1. 动作ID是否为0 (我们在 dji_kmz_mission_generator.py 中分配给 hover 的ID)
    //       2. 动作状态是否为 1 (DJI_WAYPOINT_V3_ACTION_STATE_RUNNING)
    if (actionState.actionId == 0 && actionState.state == 1) 
    {
        USER_LOG_INFO("Waypoint(V3) Action: 最后一个悬停动作已开始。手动触发任务完成信号...");
        
        // 检查 s_instance 是否存在 并且 任务是否在运行
        if (s_instance && s_instance->m_isMissionRunning.load()) 
        {
            // 使用 s_instance 访问非静态成员
            std::lock_guard<std::mutex> lock(s_instance->m_mission_mutex);
            s_instance->m_isMissionRunning = false;
            // 唤醒正在 FlyKmzFile 中等待的线程
            s_instance->m_mission_cv.notify_one(); 
        }
    }
    // [!!!! 新代码结束 !!!!]

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief 停止当前的航点任务
 */
T_DjiReturnCode WaypointMissionHandler::stopMission()
{
    // 实际 PSDK API: DjiWaypointV3_Action 用于控制任务（启动/停止/暂停）
    T_DjiReturnCode ret = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_STOP);

    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("WaypointHandler(V3): Failed to stop mission (DjiWaypointV3_Action(STOP)), ret=0x%08llX", ret);
    } else {
        USER_LOG_INFO("WaypointHandler(V3): Mission explicitly STOPPED via DjiWaypointV3_Action(STOP).");
    }
    
    return ret;
}