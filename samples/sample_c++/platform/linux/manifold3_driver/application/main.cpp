/**
 ********************************************************************
 * @file    main.cpp
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <liveview/test_liveview_entry.hpp>
#include <perception/test_perception_entry.hpp>
#include <perception/test_lidar_entry.hpp>
#include <perception/test_radar_entry.hpp>
#include <flight_control/test_flight_control.h>
#include <gimbal/test_gimbal_entry.hpp>
#include "application.hpp"
#include "fc_subscription/test_fc_subscription.h"
#include <gimbal_emu/test_payload_gimbal_emu.h>
#include <camera_emu/test_payload_cam_emu_media.h>
#include <camera_emu/test_payload_cam_emu_base.h>
#include <dji_logger.h>
#include "widget/test_widget.h"
#include "widget/test_widget_speaker.h"
#include <widget_manager/test_widget_manager.hpp>
#include <power_management/test_power_management.h>
#include "data_transmission/test_data_transmission.h"
#include <flight_controller/test_flight_controller_entry.h>
#include <positioning/test_positioning.h>
#include <hms_manager/hms_manager_entry.h>
#include "camera_manager/test_camera_manager_entry.h"
#include <hms_manager/hms_manager_entry.h>
#include <liveview/dji_liveview_object_detection.hpp>
#include "liveview/test_liveview.h"
#include <signal.h>
/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/

int main_sample(int argc, char **argv)
{
    signal(SIGINT, [](int signalNum) -> void
           { exit(0); });
    Application application(argc, argv);
    char inputChar;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    E_DjiMountPosition mountPosition = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;

start:
    std::cout
        << "\n"
        << "| Available commands:                                                                              |\n"
        << "| [0] Fc subscribe sample - subscribe quaternion and gps data                                      |\n"
        << "| [1] Flight controller sample - you can control flying by PSDK                                    |\n"
        << "| [2] Hms info manager sample - get health manger system info by language                          |\n"
        << "| [a] Gimbal manager sample - you can control gimbal by PSDK                                       |\n"
        << "| [d] Stereo vision view sample - display the stereo image                                         |\n"
        << "| [e] Run camera manager sample - you can test camera's functions interactively                    |\n"
        << "| [f] Start rtk positioning sample - you can receive rtk rtcm data when rtk signal is ok           |\n"
        << "| [g] Request Lidar data sample - Request Lidar data and store the point cloud data as pcd files   |\n"
        << "| [h] Request Radar data sample - Request radar data                                               |\n"
        << "| [i] Run manifold3 AI sample - request h.264 bitstream data, codec it and display it on pilot     |\n"
        << "| [j] Run Hms Enhance sample - shake motor and play sound on pilot                                 |\n"
        << "| [l] Run widget states manager sample, control widget states on other payload                     |\n"
        << "| [m] Run Open Ar sample - draw ar gragh                                                           |\n"
        << "| [n] Run H.264 liveview sample - save H.264 files in the current directory                        |\n"
        << std::endl;

    std::cin >> inputChar;
    switch (inputChar)
    {
    case '0':
        DjiTest_FcSubscriptionRunSample();
        break;
    case '1':
        DjiUser_RunFlightControllerSample();
        break;
    case '2':
        DjiUser_RunHmsManagerSample();
        break;
    case 'a':
        DjiUser_RunGimbalManagerSample();
        break;
    case 'd':
        DjiUser_RunStereoVisionViewSample();
        break;
    case 'e':
        DjiUser_RunCameraManagerSample();
        break;
    case 'f':
        returnCode = DjiTest_PositioningStartService();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("rtk positioning sample init error");
            break;
        }
        USER_LOG_INFO("Start rtk positioning sample successfully");
        break;
    case 'g':
        DjiUser_RunLidarDataSubscriptionSample();
        break;
    case 'h':
        DjiUser_RunRadarDataSubscriptionSample();
        break;
    case 'i':
        DjiUser_RunCameraAiDetectionSample();
        break;
    case 'j':
        DjiUser_RunHmsEnhanceSample();
        break;
    case 'l':
        DjiTest_WidgetMannagerStart();
        break;
    case 'm':
        DjiUser_RunOpenArSample();
        break;
    case 'n':
        DjiTest_LiveviewRunSample(mountPosition);
        break;
    case 'q':
        break;
    default:
        break;
    }

    osalHandler->TaskSleepMs(2000);

    goto start;
}

/*
 * ======================================================================
 * PSDKServer 指令表 (TCP 协议, 端口 8899)
 *
 * 1. 拍照 (Take Photo)
 * 命令: "tp [height]"
 * 参数: [height] - 240, 360, 480, 720, 1080
 * 示例: "tp 480"
 * 响应: [4字节大小N][N字节JPEG数据] (成功) | [4字节大小=0] (失败)
 *
 * 2. 飞行控制: 起飞 (Flight Control: Take Off)
 * 命令: "fc_takeoff"
 * 示例: "fc_takeoff"
 * 响应: [4字节大小=1] (成功) | [4字节大小=0] (失败)
 *
 * 3. 飞行控制: 降落 (Flight Control: Land)
 * 命令: "fc_land"
 * 示例: "fc_land"
 * 响应: [4字节大小=1] (成功) | [4字节大小=0] (失败)
 *
 * 4. 飞行控制: 速度控制 (机体坐标系)
 * 命令: "fc_vel [x] [y] [z] [yaw]"
 * 用途: "遥控杆"模式，基于机头朝向。
 * 参数: [x]  - 前/后 速度 (m/s) (前为正, 后为负)
 *       [y]  - 左/右 速度 (m/s) (右为正, 左为负)
 *       [z]  - 上/下 速度 (m/s) (上为正, 下为负)
 *       [yaw]- 偏航角速度 (deg/s) (顺时针为正, 逆时针为负)
 * 示例: "fc_vel 0.5 0 0 0" (向前飞)
 * 响应: [4字节大小=1] (成功) | [4字节大小=0] (失败)
 *
 * 5. 飞行控制: 速度控制 (地面坐标系)
 * 命令: "fc_vel_gnd [x] [y] [z] [yaw]"
 * 用途: "GPS"模式，基于磁北极。依赖GPS/指南针。
 * 参数: [x]  - 北/南 速度 (m/s) (北为正, 南为负)
 *       [y]  - 东/西 速度 (m/s) (东为正, 西为负)
 *       [z]  - 上/下 速度 (m/s) (上为正, 下为负)
 *       [yaw]- 偏航角速度 (deg/s) (顺时针为正, 逆时针为负)
 * 示例: "fc_vel_gnd 0 0.5 0 0" (向东飞)
 * 响应: [4字节大小=1] (成功) | [4字节大小=0] (失败)
 *
 * 6. 飞行控制: 位置控制 (机体坐标系)
 * 命令: "fc_pos [x] [y] [z] [yaw]"
 * 用途: "相对位移"模式。飞到(x,y,z)米 和 旋转(yaw)度 后悬停 (内部使用速度+时间实现)。
 * 参数: [x]  - 前/后 目标位置偏移 (m) (前为正, 后为负)
 *       [y]  - 左/右 目标位置偏移 (m) (右为正, 左为负)
 *       [z]  - 上/下 目标位置偏移 (m) (上为正, 下为负)
 *       [yaw]- 目标偏航角偏移 (deg) (顺时针为正, 逆时针为负)
 * 示例: "fc_pos 2 0 0 0" (移动到前方2米处并悬停)
 * 示例: "fc_pos 0 0 1 90" (垂直上升1米，同时顺时针旋转90度)
 * 响应: [4字节大小=1] (成功=任务已启动) | [4字节大小=0] (失败)
 *
 * 7. 飞行控制: 位置控制 (地面坐标系)
 * 命令: "fc_pos_gnd [n] [e] [z] [yaw]"
 * 用途: "相对位移" + "GPS"模式。飞到(n,e,z)米 和 旋转(yaw)度 后悬停 (内部使用速度+时间实现)。
 * 参数: [n]  - 向北 目标位置偏移 (m) (北为正, 南为负)
 *       [e]  - 向东 目标位置偏移 (m) (东为正, 西为负)
 *       [z]  - 上/下 目标位置偏移 (m) (上为正, 下为负)
 *       [yaw]- 目标偏航角偏移 (deg) (顺时针为正, 逆时针为负)
 * 示例: "fc_pos_gnd 0 5 0 0" (移动到正东5米处并悬停)
 * 响应: [4字节大小=1] (成功=任务已启动) | [4字节大小=0] (失败)
 *
 * 8. 飞行控制: 获取GPS
 * 命令: "get_gps"
 * 用途: 获取无人机当前 GPS 坐标。
 * 示例: "get_gps"
 * 响应: [4字节大小N][N字节JSON数据] (成功) | [4字节大小=0] (失败)
 * JSON 示例: {"lat": 22.12345678, "lon": 113.12345678}
 *
 * 9. 飞行控制: 获取完整姿态 (Get Pose)
 * 命令: "get_pose"
 * 用途: 获取无人机当前 3D 坐标和偏航角。
 * 示例: "get_pose"
 * 响应: [4字节大小N][N字节JSON数据] (成功) | [4字节大小=0] (失败)
 * JSON 示例: {"lat": 22.123, "lon": 113.567, "alt_m": 49.80, "yaw_deg": -175.20}
 * (说明: lat = 纬度 (十进制度), lon = 经度 (十进制度), alt_m = 相对高度 (米), yaw_deg = 偏航角 (度))
 *
 *
 * 10. 飞行控制: 飞到指定GPS (Waypoint)
 * 命令: "fc_pos_gps [lat] [lon] [alt]"
 * 用途: "绝对3D坐标"模式。飞到绝对的GPS坐标 (十进制度) 和相对高度 (米) (内部使用速度+时间实现)。
 * 参数: [lat] - 目标纬度 (e.g., 22.12345)
 *       [lon] - 目标经度 (e.g., 113.56789)
 *       [alt] - 目标相对高度 (e.g., 50.0) (基于起飞点)
 * 示例: "fc_pos_gps 22.123 113.567 50.0"
 * 响应: [4字节大小=1] (成功=任务已启动) | [4字节大小=0] (失败)
 * ======================================================================
 */
#include "psdk_server.hpp"
#include <condition_variable>

/* 全局变量用于优雅关闭 ---------------------------------------------------*/
// 用于 Ctrl+C 优雅关闭的全局变量
static std::condition_variable s_shutdown_cv;
static std::mutex s_shutdown_mutex;

/**
 * @brief 捕获 Ctrl+C (SIGINT) 信号
 */
void SigintHandler(int sig)
{
    USER_LOG_INFO("Main: SIGINT (Ctrl+C) received. Shutting down...");
    // 唤醒主线程
    s_shutdown_cv.notify_one();
}

int main(int argc, char **argv)
{
    // 1. 注册 Ctrl+C 信号处理器
    signal(SIGINT, SigintHandler);

    // 2. 初始化 PSDK Application
    Application application(argc, argv);

    // 3. 定义服务器配置
    const int SERVER_PORT = 8899; // 确保端口未被占用

    USER_LOG_INFO("Main: Creating PSDKServer on port %d...", SERVER_PORT);

    // 4. 实例化 PSDKServer
    // (构造函数将自动初始化 Liveview 并启动视频流)
    PSDKServer server(SERVER_PORT);

    // 5. 启动服务器
    if (!server.start())
    {
        USER_LOG_ERROR("Main: Failed to start PSDKServer. Exiting.");
        return -1;
    }

    USER_LOG_INFO("Main: Server is running. Press Ctrl+C to stop.");

    // 6. 使主线程休眠，直到收到 Ctrl+C
    std::unique_lock<std::mutex> lock(s_shutdown_mutex);
    s_shutdown_cv.wait(lock); // 线程将在此处永久休眠，直到被 notify

    // 7. 收到信号，开始关闭
    USER_LOG_INFO("Main: Stopping server...");
    server.stop();

    USER_LOG_INFO("Main: Exiting. Destructors will now run...");

    // PSDKServer 的析构函数将在此处自动调用
    // (它会自动销毁内部的 LiveviewCameraHandler，
    // 从而停止视频流并释放解码器)
    return 0;
}
/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
