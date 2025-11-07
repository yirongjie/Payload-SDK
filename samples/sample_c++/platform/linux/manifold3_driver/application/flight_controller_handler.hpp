/**
 ********************************************************************
 * @file    flight_controller_handler.hpp
 * @brief   PSDK 飞行控制处理器
 *********************************************************************
 */

#ifndef FLIGHT_CONTROLLER_HANDLER_HPP
#define FLIGHT_CONTROLLER_HANDLER_HPP

#include "dji_typedef.h"

class FlightControllerHandler
{
public:
    // ++++++++++ [新增] ++++++++++
    // Enums to define control logic
    /**
     * @brief 定义水平控制逻辑
     */
    enum class HorizontalLogic
    {
        VELOCITY, // 控制速度 (单位: m/s)
        POSITION  // 控制位置偏移 (单位: m)
    };
    /**
     * @brief 定义水平坐标系
     */
    enum class HorizontalCoordinate
    {
        BODY,  // 机体坐标系 (前/后/左/右)
        GROUND // 地面坐标系 (北/南/东/西)
    };
    // ++++++++++++++++++++++++++++

    FlightControllerHandler();
    ~FlightControllerHandler();

    /**
     * @brief 初始化飞行控制器并获取控制权
     * @return true 成功, false 失败
     */
    bool init();

    /**
     * @brief 执行起飞
     */
    T_DjiReturnCode takeoff();

    /**
     * @brief 执行降落
     */
    T_DjiReturnCode land();

    /**
     * @brief 获取最新的 GPS 位置
     * @param out_latitude  [out] 纬度 (十进制度)
     * @param out_longitude [out] 经度 (十进制度)
     * @return T_DjiReturnCode 操作结果
     */
    T_DjiReturnCode getGpsPosition(double &out_latitude, double &out_longitude);

    /**
     * @brief 获取最新的相对高度 (相对于起飞点)
     * @param out_altitude  [out] 相对高度 (米)
     * @return T_DjiReturnCode 操作结果
     */
    T_DjiReturnCode getRelativeAltitude(float &out_altitude);

    /**
     * @brief 获取最新的偏航角 (相对于磁北)
     * @param out_yaw_deg  [out] 偏航角 (度)
     * @return T_DjiReturnCode 操作结果
     */
    T_DjiReturnCode getYawAngle(float &out_yaw_deg);

    /**
     * @brief 执行统一的摇杆控制
     * @param x         水平 x 轴输入 (单位: m/s 或 m, 取决于 h_logic)
     * @param y         水平 y 轴输入 (单位: m/s 或 m, 取决于 h_logic)
     * @param z         垂直 z 轴输入 (单位: m/s, 垂直逻辑固定为速度)
     * @param yaw       偏航轴输入 (单位: deg/s, 偏航逻辑固定为角速度)
     * @param h_logic   水平控制逻辑 (VELOCITY 或 POSITION)
     * @param h_coord   水平坐标系 (BODY 或 GROUND)
     */
    T_DjiReturnCode executeJoystickControl(float x, float y, float z, float yaw,
                                           HorizontalLogic h_logic,
                                           HorizontalCoordinate h_coord);
    // ++++++++++++++++++++++++++++

    /**
     * @brief [新增] 重新获取摇杆控制权
     * @note 在 Waypoint V3 任务结束后，必须调用此函数才能恢复摇杆控制 (例如 land)
     */
    T_DjiReturnCode reObtainJoystickCtrlAuthority();

private:
    bool m_isInitialized;
};

#endif // FLIGHT_CONTROLLER_HANDLER_HPP