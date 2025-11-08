/**
 ********************************************************************
 * @file    flight_controller_handler.cpp
 * @brief   PSDK 飞行控制处理器的实现
 *********************************************************************
 */

#include "flight_controller_handler.hpp"
#include "dji_flight_controller.h"
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include <cmath>

/**
 * @brief 摇杆控制权切换事件的回调函数
 * @note  这个函数是静态的，因为它是一个 C 风格的回调。
 */
static T_DjiReturnCode
FlightCtrlJoystickCtrlAuthSwitchEventCb(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData)
{
    // 这里我们只打印日志，您可以根据需要实现更复杂的逻辑
    if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK)
    {
        USER_LOG_INFO("FlightHandler: OSDK (this application) obtained joystick control authority.");
    }
    else
    {
        USER_LOG_INFO("FlightHandler: OSDK (this application) lost joystick control authority.");
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

FlightControllerHandler::FlightControllerHandler() : m_isInitialized(false)
{
}

FlightControllerHandler::~FlightControllerHandler()
{
    if (m_isInitialized)
    {
        USER_LOG_INFO("FlightHandler: Releasing joystick control authority...");
        T_DjiReturnCode ret = DjiFlightController_ReleaseJoystickCtrlAuthority();
        if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("FlightHandler: Failed to release joystick control authority.");
        }
    }
    USER_LOG_INFO("FlightHandler: Destroyed.");
}

bool FlightControllerHandler::init()
{
    T_DjiReturnCode ret;

    // 1. 初始化飞行控制器
    // 我们使用一个默认的 RID 信息，您应该替换为您自己的
    T_DjiFlightControllerRidInfo ridInfo = {0};
    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 0;

    ret = DjiFlightController_Init(ridInfo);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to init flight controller, ret=0x%08llX", ret);
        return false;
    }

    // 2. 初始化数据订阅
    ret = DjiFcSubscription_Init();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to init data subscription, ret=0x%08llX", ret);
        return false;
    }

    // 2b. 订阅 GPS 位置数据
    // 我们不需要回调 (NULL)，因为我们将使用 GetLatestValueOfTopic 手动拉取
    ret = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                           DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, // 5Hz 足够用于状态查询
                                           NULL);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to subscribe to GPS_POSITION, ret=0x%08llX", ret);
        return false;
    }
    else
    {
        USER_LOG_INFO("FlightHandler: Subscribed to GPS_POSITION topic.");
    }

    // +++++++++ [订阅融合高度数据] +++++++++
    ret = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                           DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, // 10Hz 足够
                                           NULL);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to subscribe to HEIGHT_FUSION, ret=0x%08llX", ret);
        return false;
    }
    else
    {
        USER_LOG_INFO("FlightHandler: Subscribed to HEIGHT_FUSION topic.");
    }

    // +++++++++ [订阅四元数 (用于计算偏航角)] +++++++++
    ret = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                           DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, // [修改] 提高频率
                                           NULL);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to subscribe to QUATERNION, ret=0x%08llX", ret);
        return false;
    }
    else
    {
        USER_LOG_INFO("FlightHandler: Subscribed to QUATERNION topic.");
    }
    // +++++++++++++++++++++++++++++++++++++++++

    // 3. 注册控制权切换的回调
    ret = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(FlightCtrlJoystickCtrlAuthSwitchEventCb);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to register authority callback, ret=0x%08llX", ret);
        return false;
    }

    // 4. 获取控制权
    USER_LOG_INFO("FlightHandler: Requesting joystick control authority...");
    ret = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to obtain joystick control authority, ret=0x%08llX", ret);
        return false;
    }

    // 等待一小会，让回调有时间触发 (可选，但有助于调试)
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("FlightHandler: Initialized successfully.");
    m_isInitialized = true;
    return true;
}

T_DjiReturnCode FlightControllerHandler::turnOnMotors()
{
    T_DjiReturnCode ret = DjiFlightController_TurnOnMotors();

    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiFlightController_TurnOnMotors failed, ret=0x%08llX", ret);
    }

    return ret;
}

T_DjiReturnCode FlightControllerHandler::takeoff()
{
    if (!m_isInitialized)
    {
        USER_LOG_ERROR("FlightHandler: Cannot takeoff, not initialized.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }
    USER_LOG_INFO("FlightHandler: Executing takeoff...");
    return DjiFlightController_StartTakeoff();
}

T_DjiReturnCode FlightControllerHandler::land()
{
    if (!m_isInitialized)
    {
        USER_LOG_ERROR("FlightHandler: Cannot land, not initialized.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }
    USER_LOG_INFO("FlightHandler: Executing landing...");
    return DjiFlightController_StartLanding();
}

/**
 * @brief 实现 getGpsPosition
 */
T_DjiReturnCode FlightControllerHandler::getGpsPosition(double &out_latitude, double &out_longitude)
{
    if (!m_isInitialized)
    {
        USER_LOG_ERROR("FlightHandler: Cannot get GPS, not initialized.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }

    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiReturnCode ret;

    // 1. 从订阅模块中拉取最新数据
    ret = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                  (uint8_t *)&gpsPosition,
                                                  sizeof(T_DjiFcSubscriptionGpsPosition),
                                                  &timestamp);

    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to get GPS_POSITION data, ret=0x%08llX", ret);
        return ret;
    }

    // 检查时间戳是否有效 (可选但推荐)
    if (timestamp.millisecond == 0)
    {
        USER_LOG_WARN("FlightHandler: GPS_POSITION data is stale (timestamp is 0).");
        // 注意：我们仍然返回成功，但数据可能是旧的
    }

    // 2. 转换: int32_t (deg * 10^7) -> double (deg)
    //    根据 DJI 示例标准: Y 对应纬度 (Latitude), X 对应经度 (Longitude)
    out_latitude = (double)gpsPosition.y / 10000000.0;
    out_longitude = (double)gpsPosition.x / 10000000.0;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
// ... (在 getGpsPosition 函数结束的 '}' 之后) ...

/**
 * @brief [新增] 实现 getRelativeAltitude
 * [参考: test_flight_controller_command_flying.cpp, DjiUser_FlightControlGetValueOfRelativeHeight]
 */
T_DjiReturnCode FlightControllerHandler::getRelativeAltitude(float &out_altitude)
{
    if (!m_isInitialized)
    {
        USER_LOG_ERROR("FlightHandler: Cannot get Altitude, not initialized.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }

    T_DjiFcSubscriptionAltitudeOfHomePoint altitudeData = 0; // T_DjiFcSubscriptionAltitudeOfHomePoint 就是 float
    T_DjiDataTimestamp timestamp = {0};
    T_DjiReturnCode ret;

    // 1. 从订阅模块中拉取最新数据
    ret = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                  (uint8_t *)&altitudeData,
                                                  sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                  &timestamp);

    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to get HEIGHT_FUSION data, ret=0x%08llX", ret);
        return ret;
    }

    if (timestamp.millisecond == 0)
    {
        USER_LOG_WARN("FlightHandler: HEIGHT_FUSION data is stale (timestamp is 0).");
    }

    // 2. 赋值
    out_altitude = altitudeData;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
/**
 * @brief [新增] 实现 getYawAngle
 * [参考: test_flight_controller_command_flying.cpp, DjiUser_FlightControlGetValueOfQuaternion]
 */
T_DjiReturnCode FlightControllerHandler::getYawAngle(float &out_yaw_deg)
{
    if (!m_isInitialized)
    {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }

    T_DjiFcSubscriptionQuaternion quaternion = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiReturnCode ret;

    ret = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                  (uint8_t *)&quaternion,
                                                  sizeof(T_DjiFcSubscriptionQuaternion),
                                                  &timestamp);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        return ret;
    }

    // [参考: test_flight_controller_command_flying.cpp]
    double yaw_rad = atan2(2.0 * (quaternion.q1 * quaternion.q2 + quaternion.q0 * quaternion.q3),
                           -2.0 * (quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3) + 1.0);

    out_yaw_deg = (float)(yaw_rad * 180.0 / M_PI); // 转换为度
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FlightControllerHandler::executeJoystickControl(float x, float y, float z, float yaw,
                                                                HorizontalLogic h_logic,
                                                                HorizontalCoordinate h_coord)
{
    if (!m_isInitialized)
    {
        USER_LOG_ERROR("FlightHandler: Cannot execute control, not initialized.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }

    // 1. 定义控制模式
    T_DjiFlightControllerJoystickMode joystickMode;

    // 垂直和偏航逻辑保持不变 (同 PSDK 示例)
    joystickMode.verticalControlMode = DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE;
    joystickMode.yawControlMode = DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE;
    joystickMode.stableControlMode = DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE;

    // 2. 根据参数设置水平逻辑
    if (h_logic == HorizontalLogic::VELOCITY)
    {
        joystickMode.horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE;
    }
    else
    {
        joystickMode.horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE;
    }

    // 3. 根据参数设置水平坐标系
    if (h_coord == HorizontalCoordinate::BODY)
    {
        joystickMode.horizontalCoordinate = DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE;
    }
    else
    {
        joystickMode.horizontalCoordinate = DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE;
    }

    // 4. 定义控制指令
    T_DjiFlightControllerJoystickCommand command = {0};
    command.x = x;
    command.y = y;
    command.z = z;
    command.yaw = yaw;

    // 5. 设置模式并执行

    DjiFlightController_SetJoystickMode(joystickMode);
    // USER_LOG_INFO("PSDKServer:fly: DjiFlightController_SetJoystickMode");

    T_DjiReturnCode ret = DjiFlightController_ExecuteJoystickAction(command);
    // USER_LOG_INFO("PSDKServer:fly: DjiFlightController_ExecuteJoystickAction");
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to execute joystick action, ret=0x%08llX", ret);
    }

    return ret;
}


/**
 * @brief [新增] 实现 reObtainJoystickCtrlAuthority
 */
T_DjiReturnCode FlightControllerHandler::reObtainJoystickCtrlAuthority()
{
    if (!m_isInitialized)
    {
        USER_LOG_ERROR("FlightHandler: Cannot re-obtain authority, not initialized.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }

    USER_LOG_INFO("FlightHandler: Re-Requesting joystick control authority (post-mission)...");
    T_DjiReturnCode ret = DjiFlightController_ObtainJoystickCtrlAuthority(); //
    
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("FlightHandler: Failed to re-obtain joystick control authority, ret=0x%08llX", ret);
    }
    else
    {
        USER_LOG_INFO("FlightHandler: Re-obtained joystick control successfully.");
    }
     T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    // (可选但推荐) 等待飞控处理权限切换
    if (osalHandler) {
        osalHandler->TaskSleepMs(500); // 等待 500ms
    }
    
    return ret;
}
// ++++++++++++++++++++++++++++