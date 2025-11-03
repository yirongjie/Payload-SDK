/**
 ********************************************************************
 * @file    psdk_server.cpp
 * @brief   PSDKServer 类的实现
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "psdk_server.hpp"
#include "liveview_camera_handler.hpp" // 包含 camera 类的完整定义
#include "flight_controller_handler.hpp"
#include "waypoint_mission_handler.hpp"
#include "dji_logger.h"
#include "dji_platform.h" // <-- [新增] 用于获取 OsalHandler (GetTimeMs, TaskSleepMs)
#include <cmath>          // <-- 用于 sqrt (计算距离)
#include <iomanip>

// C++ 网络与线程库
#include <chrono>
#include <sstream>
#include <string.h> // for memset
#include <vector>
#include <iostream>
#include <fstream>
#include <time.h>
#include <stdio.h> // for snprintf
#include <iomanip>

// POSIX Socket 库
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> // for read, close, write
#include <memory>
#include <iomanip> // <-- 用于 std::setprecision (日志)

// +++++++++ +++++++++
// #define M_PI 3.14159265358979323846
#define EARTH_RADIUS 6371000.0 // meters

/**
 * @brief (辅助函数) 将两个 GPS 坐标点的差异转换为 N/E 方向的米
 */
void GpsToMeters(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg, float &out_north_m, float &out_east_m)
{
    double lat1_rad = lat1_deg * M_PI / 180.0;
    double lat2_rad = lat2_deg * M_PI / 180.0;
    double dLat_rad = (lat2_deg - lat1_deg) * M_PI / 180.0;
    double dLon_rad = (lon2_deg - lon1_deg) * M_PI / 180.0;

    // 计算北向距离 (米)
    out_north_m = (float)(dLat_rad * EARTH_RADIUS);

    // 计算东向距离 (米)
    out_east_m = (float)(dLon_rad * EARTH_RADIUS * cos((lat1_rad + lat2_rad) / 2.0));
}
// +++++++++++++++++++++++++

// 宏定义
#define TEST_LIVEVIEW_STREAM_FILE_PATH_STR_MAX_SIZE 256

// [新增] 定义响应码
#define RESPONSE_CODE_FAIL 0
#define RESPONSE_CODE_ACCEPTED 1
#define RESPONSE_CODE_COMPLETED 2

/******************************************************************************
 * PSDKServer 类实现
 *****************************************************************************/

PSDKServer::PSDKServer(int port, bool saveLocalCopy)
    : m_port(port),
      m_server_fd(-1),
      m_running(false),
      m_saveLocalCopy(saveLocalCopy) // 初始化标志位
{
    // 1. 初始化相机
    USER_LOG_INFO("PSDKServer: Initializing Camera Handler...");
    // 在构造函数中创建内部的 LiveviewCameraHandler
    E_DjiMountPosition mountPosition = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1; // XIUGAI
    m_cameraHandler.reset(new LiveviewCameraHandler(mountPosition));

    if (m_cameraHandler == nullptr)
    {
        USER_LOG_ERROR("PSDKServer: Failed to create LiveviewCameraHandler!");
    }

    // 2. 初始化飞行控制器
    USER_LOG_INFO("PSDKServer: Initializing Flight Controller...");
    m_flightHandler.reset(new FlightControllerHandler());
    if (m_flightHandler == nullptr)
    {
        USER_LOG_ERROR("PSDKServer: Failed to create FlightControllerHandler!");
    }
    else if (!m_flightHandler->init())
    {
        USER_LOG_ERROR("PSDKServer: Failed to initialize FlightControllerHandler! Flight control is DISABLED.");
        m_flightHandler.reset(); // 初始化失败，释放它
    }

    
    // 3. 初始化航点任务处理器
    USER_LOG_INFO("PSDKServer: Initializing Waypoint Mission Handler...");
    m_waypointHandler.reset(new WaypointMissionHandler());
    if (m_waypointHandler == nullptr)
    {
        USER_LOG_ERROR("PSDKServer: Failed to create WaypointMissionHandler!");
    }
    else if (!m_waypointHandler->init())
    {
        USER_LOG_ERROR("PSDKServer: Failed to initialize WaypointMissionHandler! Waypoint missions DISABLED.");
        m_waypointHandler.reset(); // 初始化失败，释放它
    }
    
    //

    if (m_saveLocalCopy)
    {
        USER_LOG_INFO("PSDKServer: Local save for ground truth is ENABLED.");
    }
    else
    {
        USER_LOG_INFO("PSDKServer: Local save for ground truth is DISABLED.");
    }
}

PSDKServer::~PSDKServer()
{
    stop();
    // m_cameraHandler (unique_ptr) 将在此处自动销毁，
    // 调用 LiveviewCameraHandler 的析构函数
    USER_LOG_INFO("PSDKServer: Destroyed.");
}

bool PSDKServer::start()
{
    if (m_cameraHandler == nullptr)
    {
        USER_LOG_ERROR("PSDKServer: Cannot start, camera handler is invalid.");
        return false;
    }

    struct sockaddr_in address;
    int opt = 1;

    // 1. 创建 socket 文件描述符
    if ((m_server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        USER_LOG_ERROR("PSDKServer: socket failed");
        return false;
    }

    // 2. 允许端口重用
    if (setsockopt(m_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)))
    {
        USER_LOG_ERROR("PSDKServer: setsockopt failed");
        return false;
    }

    // 3. 绑定地址
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY; // 监听所有 IP
    address.sin_port = htons(m_port);

    if (bind(m_server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        USER_LOG_ERROR("PSDKServer: bind failed on port %d", m_port);
        return false;
    }

    // 4. 开始监听
    if (listen(m_server_fd, 5) < 0) // 5 个连接的队列
    {
        USER_LOG_ERROR("PSDKServer: listen failed");
        return false;
    }

    // 5. 启动监听线程
    m_running = true;
    m_listenThread = std::thread(&PSDKServer::listenLoop, this);

    USER_LOG_INFO("PSDKServer: Started successfully on port %d", m_port);
    return true;
}

void PSDKServer::stop()
{
    m_running = false;

    if (m_server_fd != -1)
    {
        // 强制关闭 socket，这将使 accept() 调用失败
        shutdown(m_server_fd, SHUT_RDWR);
        close(m_server_fd);
        m_server_fd = -1;
    }

    if (m_listenThread.joinable())
    {
        m_listenThread.join();
    }
    USER_LOG_INFO("PSDKServer: Stopped.");
}

void PSDKServer::listenLoop()
{
    while (m_running)
    {
        struct sockaddr_in address;
        int addrlen = sizeof(address);
        int client_socket = -1;

        // 6. 接受新连接 (阻塞)
        client_socket = accept(m_server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);

        if (client_socket < 0)
        {
            if (m_running) // 只有在服务仍在运行时才是真错误
            {
                USER_LOG_ERROR("PSDKServer: accept failed");
            }
            continue;
        }

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &address.sin_addr, client_ip, INET_ADDRSTRLEN);
        USER_LOG_INFO("PSDKServer: New connection from %s", client_ip);

        // 7. 为每个客户端分离一个新线程进行处理
        std::thread(&PSDKServer::handleClient, this, client_socket).detach();
    }
}

/**
 * @brief 发送一个简单的响应码 (0=失败, 1=接受, 2=完成)
 */
void PSDKServer::sendSimpleResponse(int client_socket, uint32_t response_code)
{
    uint32_t net_code = htonl(response_code);
    if (send(client_socket, &net_code, sizeof(net_code), 0) != sizeof(net_code))
    {
        USER_LOG_WARN("PSDKServer: Failed to send simple response code.");
    }
}

/**
 * @brief [修改] 在一个单独的线程中执行长时飞行任务
 */
void PSDKServer::executeLongFlightTask(std::string cmd_str, int client_socket)
{
    // [NEW] 此函数在一个新线程中运行。
    // [NEW] 在此获取飞行锁，并在函数返回时自动释放。
    std::unique_lock<std::mutex> flight_lock(m_flight_lock);

    std::stringstream ss(cmd_str);
    std::string cmd_name;
    ss >> cmd_name; // 提取命令名

    USER_LOG_INFO("PSDKServer: Flight lock acquired for long task '%s'.", cmd_name.c_str());

    // [新增] 标记任务是否成功
    bool task_success = false;

    // ----------------------------------------------------
    // 逻辑: "fc_pos" / "fc_pos_gnd"
    // ----------------------------------------------------
    if (cmd_name == "fc_pos" || cmd_name == "fc_pos_gnd")
    {
        float x_dist_m = 0, y_dist_m = 0, z_dist_m = 0, yaw_angle_deg = 0;
        if (!(ss >> x_dist_m >> y_dist_m >> z_dist_m >> yaw_angle_deg))
        {
            USER_LOG_WARN("PSDKServer (Task): Invalid %s args.", cmd_name.c_str());
        }
        else if (!m_flightHandler)
        {
            USER_LOG_ERROR("PSDKServer (Task): Flight handler not initialized for %s.", cmd_name.c_str());
        }
        else
        {
            float start_alt_rel = 0, start_yaw_deg = 0;
            T_DjiReturnCode ret_alt = m_flightHandler->getRelativeAltitude(start_alt_rel);
            T_DjiReturnCode ret_yaw = m_flightHandler->getYawAngle(start_yaw_deg);

            if (ret_alt != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || ret_yaw != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                USER_LOG_ERROR("PSDKServer (Task): fc_pos failed, cannot get current Alt/Yaw state.");
            }
            else
            {
                float target_alt_rel = start_alt_rel + z_dist_m;
                float target_yaw_deg = start_yaw_deg + yaw_angle_deg;
                USER_LOG_INFO("PSDKServer (Task): Executing '%s' (X:%.1fm, Y:%.1fm, Z:%.1fm, Yaw:%.1fdeg)",
                              cmd_name.c_str(), x_dist_m, y_dist_m, z_dist_m, yaw_angle_deg);

                const float FIXED_XY_SPEED_MPS = 1.0, FIXED_Z_SPEED_MPS = 0.5, FIXED_YAW_RATE_DPS = 15.0;
                float dist_xy = std::sqrt(x_dist_m * x_dist_m + y_dist_m * y_dist_m);
                uint32_t duration_xy_ms = (dist_xy < 0.1) ? 0 : (uint32_t)((dist_xy / FIXED_XY_SPEED_MPS) * 1000.0);
                uint32_t duration_z_ms = (std::abs(z_dist_m) < 0.1) ? 0 : (uint32_t)((std::abs(z_dist_m) / FIXED_Z_SPEED_MPS) * 1000.0);
                uint32_t duration_yaw_ms = (std::abs(yaw_angle_deg) < 1.0) ? 0 : (uint32_t)((std::abs(yaw_angle_deg) / FIXED_YAW_RATE_DPS) * 1000.0);
                uint32_t duration_ms = std::max(duration_xy_ms, std::max(duration_z_ms, duration_yaw_ms));
                if (duration_ms == 0)
                    duration_ms = 1000;

                USER_LOG_INFO("PSDKServer (Task): -> Max duration %u ms", duration_ms);

                float vel_x = (duration_xy_ms == 0) ? 0.0 : (x_dist_m / (duration_xy_ms / 1000.0));
                float vel_y = (duration_xy_ms == 0) ? 0.0 : (y_dist_m / (duration_xy_ms / 1000.0));
                float vel_z = (duration_z_ms == 0) ? 0.0 : (z_dist_m > 0 ? FIXED_Z_SPEED_MPS : -FIXED_Z_SPEED_MPS);
                float yaw_rate = (duration_yaw_ms == 0) ? 0.0 : (yaw_angle_deg > 0 ? FIXED_YAW_RATE_DPS : -FIXED_YAW_RATE_DPS);

                FlightControllerHandler::HorizontalCoordinate h_coord = (cmd_name == "fc_pos") ? FlightControllerHandler::HorizontalCoordinate::BODY : FlightControllerHandler::HorizontalCoordinate::GROUND;
                T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
                uint32_t startTime = 0, currentTime = 0, elapsedTimeInMs = 0;
                osal->GetTimeMs(&startTime);
                const int loop_delay_ms = 50; // 20Hz

                while (elapsedTimeInMs < duration_ms)
                {
                    if (!m_running.load())
                        break;
                    float current_vel_x = (elapsedTimeInMs > duration_xy_ms) ? 0.0 : vel_x;
                    float current_vel_y = (elapsedTimeInMs > duration_xy_ms) ? 0.0 : vel_y;
                    float current_vel_z = (elapsedTimeInMs > duration_z_ms) ? 0.0 : vel_z;
                    float current_yaw_rate = (elapsedTimeInMs > duration_yaw_ms) ? 0.0 : yaw_rate;
                    m_flightHandler->executeJoystickControl(current_vel_x, current_vel_y, current_vel_z, current_yaw_rate, FlightControllerHandler::HorizontalLogic::VELOCITY, h_coord);
                    osal->TaskSleepMs(loop_delay_ms);
                    osal->GetTimeMs(&currentTime);
                    elapsedTimeInMs = currentTime - startTime;
                }

                if (m_running.load())
                {
                    USER_LOG_INFO("PSDKServer (Task): '%s' complete. Sending hover.", cmd_name.c_str());
                    m_flightHandler->executeJoystickControl(0, 0, 0, 0, FlightControllerHandler::HorizontalLogic::VELOCITY, FlightControllerHandler::HorizontalCoordinate::BODY);
                    task_success = true; // [新增] 标记成功
                }
            }
        }
    }
    // ----------------------------------------------------
    // 逻辑: "fc_seq"
    // ----------------------------------------------------
    else if (cmd_name == "fc_seq")
    {
        float speed_cm_s = 0;
        int n_steps = 0;
        bool parse_success = true;
        if (!(ss >> speed_cm_s >> n_steps) || n_steps <= 0 || n_steps > 100)
        {
            USER_LOG_WARN("PSDKServer (Task): Invalid fc_seq args.");
            parse_success = false;
        }

        std::vector<std::vector<float>> steps;
        if (parse_success)
        {
            for (int i = 0; i < n_steps; ++i)
            {
                float dx, dy, dz, dyaw;
                if (!(ss >> dx >> dy >> dz >> dyaw))
                {
                    USER_LOG_WARN("PSDKServer (Task): fc_seq parsing step %d failed.", i + 1);
                    parse_success = false;
                    break;
                }
                steps.push_back({dx, dy, dz, dyaw});
            }
        }

        if (parse_success && m_flightHandler)
        {
            USER_LOG_INFO("PSDKServer (Task): Executing 'fc_seq' (%d steps)...", n_steps);
            const float SPEED_XY_CM_S = std::max(10.0f, speed_cm_s);
            const float SPEED_Z_CM_S = std::max(10.0f, speed_cm_s * 0.5f);
            const float RATE_YAW_DPS = std::max(15.0f, speed_cm_s * 0.3f);
            const int loop_delay_ms = 50;
            const float CONTROL_TICK_S = 0.05f;
            T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();

            for (int i = 0; i < n_steps; ++i)
            {
                if (!m_running.load())
                    break;
                const auto &step = steps[i];
                float dx_cm = step[0], dy_cm_vla = step[1], dz_cm_vla = step[2], dyaw_deg = step[3];
                float dist_xy_cm = std::sqrt(dx_cm * dx_cm + dy_cm_vla * dy_cm_vla);
                float duration_xy_s = (dist_xy_cm > 0.1f) ? (dist_xy_cm / SPEED_XY_CM_S) : 0;
                float duration_z_s = (std::abs(dz_cm_vla) > 0.1f) ? (std::abs(dz_cm_vla) / SPEED_Z_CM_S) : 0;
                float duration_yaw_s = (std::abs(dyaw_deg) > 0.5f) ? (std::abs(dyaw_deg) / RATE_YAW_DPS) : 0;
                float step_duration_s = std::max(duration_xy_s, std::max(duration_z_s, duration_yaw_s));
                step_duration_s = std::max(step_duration_s, CONTROL_TICK_S);

                float vel_x_m4d_mps = (dx_cm / step_duration_s) / 100.0f;
                float vel_y_m4d_mps = (dy_cm_vla / step_duration_s) / 100.0f;
                float vel_z_m4d_mps = -(dz_cm_vla / step_duration_s) / 100.0f;
                float vel_yaw_m4d_dps = dyaw_deg / step_duration_s;

                USER_LOG_INFO("PSDKServer (Task): fc_seq step %d/%d: (dur: %.2fs)", i + 1, n_steps, step_duration_s);

                uint32_t startTime = 0, currentTime = 0, elapsedTimeInMs = 0;
                osal->GetTimeMs(&startTime);
                uint32_t duration_ms = (uint32_t)(step_duration_s * 1000.0f);

                while (elapsedTimeInMs < duration_ms)
                {
                    if (!m_running.load())
                        break;
                    m_flightHandler->executeJoystickControl(vel_x_m4d_mps, vel_y_m4d_mps, vel_z_m4d_mps, vel_yaw_m4d_dps, FlightControllerHandler::HorizontalLogic::VELOCITY, FlightControllerHandler::HorizontalCoordinate::BODY);
                    osal->TaskSleepMs(loop_delay_ms);
                    osal->GetTimeMs(&currentTime);
                    elapsedTimeInMs = currentTime - startTime;
                }
            }
            
            if (m_running.load())
            {
                 // 循环结束，发送悬停 (fc_seq 特有)
                 USER_LOG_INFO("PSDKServer (Task): 'fc_seq' complete. Hovering.");
                 m_flightHandler->executeJoystickControl(0, 0, 0, 0, FlightControllerHandler::HorizontalLogic::VELOCITY, FlightControllerHandler::HorizontalCoordinate::BODY);
                 task_success = true; // [新增] 标记成功
            }
        }
    }
    // ----------------------------------------------------
    // 逻辑: "fc_pos_gps"
    // ----------------------------------------------------
    else if (cmd_name == "fc_pos_gps")
    {
        double target_lat = 0, target_lon = 0;
        float target_alt_rel = 0;
        if (!(ss >> target_lat >> target_lon >> target_alt_rel))
        {
            USER_LOG_WARN("PSDKServer (Task): Invalid fc_pos_gps args.");
        }
        else if (!m_flightHandler)
        {
            USER_LOG_ERROR("PSDKServer (Task): Flight handler not initialized for fc_pos_gps.");
        }
        else
        {
            double current_lat = 0, current_lon = 0;
            float current_alt_rel = 0;
            T_DjiReturnCode ret_gps = m_flightHandler->getGpsPosition(current_lat, current_lon);
            T_DjiReturnCode ret_alt = m_flightHandler->getRelativeAltitude(current_alt_rel);

            if (ret_gps != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || ret_alt != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                USER_LOG_ERROR("PSDKServer (Task): fc_pos_gps failed, cannot get current 3D position.");
            }
            else
            {
                float north_m = 0, east_m = 0;
                GpsToMeters(current_lat, current_lon, target_lat, target_lon, north_m, east_m);
                float vertical_m = target_alt_rel - current_alt_rel;
                USER_LOG_INFO("PSDKServer (Task): Executing 'fc_pos_gps' (N:%.1f, E:%.1f, Up:%.1f)", north_m, east_m, vertical_m);

                const float FIXED_TOTAL_SPEED_MPS = 1.0;
                float distance_total_m = std::sqrt(north_m * north_m + east_m * east_m + vertical_m * vertical_m);
                uint32_t duration_ms = 0;
                float vel_n = 0.0, vel_e = 0.0, vel_z = 0.0;

                if (distance_total_m < 0.1)
                {
                    duration_ms = 1000;
                }
                else
                {
                    duration_ms = (uint32_t)((distance_total_m / FIXED_TOTAL_SPEED_MPS) * 1000.0);
                    vel_n = (north_m / distance_total_m) * FIXED_TOTAL_SPEED_MPS;
                    vel_e = (east_m / distance_total_m) * FIXED_TOTAL_SPEED_MPS;
                    vel_z = (vertical_m / distance_total_m) * FIXED_TOTAL_SPEED_MPS;
                    USER_LOG_INFO("PSDKServer (Task): -> 3D Speed (N:%.2f, E:%.2f, Up:%.2f) m/s, Duration %u ms", vel_n, vel_e, vel_z, duration_ms);
                }

                T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
                uint32_t startTime = 0, currentTime = 0, elapsedTimeInMs = 0;
                osal->GetTimeMs(&startTime);
                const int loop_delay_ms = 50;

                while (elapsedTimeInMs < duration_ms)
                {
                    if (!m_running.load())
                        break;
                    m_flightHandler->executeJoystickControl(vel_n, vel_e, vel_z, 0, FlightControllerHandler::HorizontalLogic::VELOCITY, FlightControllerHandler::HorizontalCoordinate::GROUND);
                    osal->TaskSleepMs(loop_delay_ms);
                    osal->GetTimeMs(&currentTime);
                    elapsedTimeInMs = currentTime - startTime;
                }

                if (m_running.load())
                {
                    USER_LOG_INFO("PSDKServer (Task): 'fc_pos_gps' complete. Hovering.");
                    m_flightHandler->executeJoystickControl(0, 0, 0, 0, FlightControllerHandler::HorizontalLogic::VELOCITY, FlightControllerHandler::HorizontalCoordinate::BODY);
                    task_success = true; // [新增] 标记成功
                }
            }
        }
    }
    // ----------------------------------------------------
    // [修改] 逻辑: "fc_pos_wp" (航点V3)
    // ----------------------------------------------------
    else if (cmd_name == "fc_pos_wp")
    {
        std::string kmz_path;
        if (!(ss >> kmz_path))
        {
            USER_LOG_WARN("PSDKServer (Task): Invalid 'fc_pos_wp' args. Format: fc_pos_wp [path_to_kmz_file]");
        }
        else if (!m_waypointHandler)
        {
            USER_LOG_ERROR("PSDKServer (Task): Waypoint handler (V3) not initialized, cannot execute 'fc_pos_wp'.");
        }
        else
        {
            USER_LOG_INFO("PSDKServer (Task): Executing 'fc_pos_wp' with file: %s", kmz_path.c_str());
            
            // [修改] 此调用现在是阻塞的，直到航点任务完成
            T_DjiReturnCode ret = m_waypointHandler->FlyKmzFile(kmz_path);

            if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                USER_LOG_ERROR("PSDKServer (Task): 'fc_pos_wp' mission FAILED, ret=0x%08llX", ret);
                // task_success 保持 false
            }
            else
            {
                USER_LOG_INFO("PSDKServer (Task): 'fc_pos_wp' mission COMPLETED successfully.");
                task_success = true; // [新增] 标记成功
            }
            
            // [!!!! 在此添加修复 !!!!]
            // 无论任务成功与否，都必须尝试重新获取摇杆控制权
            if (m_flightHandler)
            {
                USER_LOG_INFO("PSDKServer (Task): Mission finished. Re-obtaining joystick control authority...");
                T_DjiReturnCode regain_ret = m_flightHandler->reObtainJoystickCtrlAuthority();
                if (regain_ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                {
                    USER_LOG_ERROR("PSDKServer (Task): FAILED to re-obtain joystick control, ret=0x%08llX", regain_ret);
                    // 如果夺回控制权失败，也应视为任务失败
                    task_success = false;
                }
                else
                {
                     USER_LOG_INFO("PSDKServer (Task): Re-obtained joystick control successfully.");
                }
            }
            else
            {
                USER_LOG_ERROR("PSDKServer (Task): Flight handler is null, cannot regain control!");
                task_success = false; 
            }
            // [!!!! 修复结束 !!!!]
        }
    }

    // --- [新增] 任务结束，发送最终信号 ---
    if (task_success)
    {
        USER_LOG_INFO("PSDKServer (Task): '%s' COMPLETED. Sending signal code 2 (COMPLETED) to client.", cmd_name.c_str());
        sendSimpleResponse(client_socket, RESPONSE_CODE_COMPLETED);
    }
    else
    {
        USER_LOG_WARN("PSDKServer (Task): '%s' FAILED. Sending signal code 0 (FAIL) to client.", cmd_name.c_str());
        sendSimpleResponse(client_socket, RESPONSE_CODE_FAIL);
    }

    USER_LOG_INFO("PSDKServer: Flight lock released for long task '%s'.", cmd_name.c_str());
    // [NEW] 锁在此处自动释放
}

/**
 * @brief (修改版) 处理持久连接上的多个命令
 */
void PSDKServer::handleClient(int client_socket)
{
    char buffer[1024];
    int valread = 0;

    // [NEW] 保持循环，直到客户端断开连接或服务器停止
    while (m_running.load())
    {
        memset(buffer, 0, sizeof(buffer)); // [NEW] 每次循环清空缓冲区

        // 1. 读取客户端命令 (阻塞)
        valread = read(client_socket, buffer, 1023);
        if (valread <= 0)
        {
            // [MODIFIED] 客户端断开连接
            USER_LOG_INFO("PSDKServer: Client disconnected (read returned %d). Breaking loop.", valread);
            break; // [MODIFIED] 退出循环
        }

        std::string cmd_str(buffer, valread);
        std::stringstream ss(cmd_str);
        std::string cmd_name;

        ss >> cmd_name;

        if (cmd_name.empty())
        {
            continue; // [NEW] 收到空命令 (例如只有 \n)，忽略并等待下一个
        }

        // [MODIFIED] 飞行锁不再在此处声明
        // std::unique_lock<std::mutex> flight_lock;

        // 3. 解析命令
        if (cmd_name == "tp")
        {
            // ----------------------------------------------------
            // 拍照: "tp [height]" (非阻塞)
            // ----------------------------------------------------
            int height = 0;
            if (!(ss >> height) || (height != 240 && height != 360 && height != 480 && height != 720 && height != 1080))
            {
                USER_LOG_WARN("PSDKServer: Invalid 'tp' height. (期望 240-1080)");
                sendSimpleResponse(client_socket, RESPONSE_CODE_FAIL); // 0 = 失败
            }
            else
            {
                // (拍照逻辑保持不变)
                std::vector<uint8_t> jpegBuffer;
                if (!m_cameraHandler)
                {
                    USER_LOG_ERROR("PSDKServer: Camera handler is null, cannot take photo.");
                    sendSimpleResponse(client_socket, RESPONSE_CODE_FAIL);
                }
                else
                {
                    T_DjiReturnCode ret = m_cameraHandler->takePhoto(height, jpegBuffer);
                    if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                    {
                        // (可选的本地保存逻辑保持不变)
                        // ...
                        uint32_t net_size = htonl(jpegBuffer.size());
                        send(client_socket, &net_size, sizeof(net_size), 0);
                        send(client_socket, jpegBuffer.data(), jpegBuffer.size(), 0);
                    }
                    else
                    {
                        sendSimpleResponse(client_socket, RESPONSE_CODE_FAIL); // 0 字节表示失败
                    }
                }
            }
        }
        else if (cmd_name == "fc_takeoff" || cmd_name == "fc_land" || cmd_name == "fc_vel" || cmd_name == "fc_vel_gnd")
        {
            // ----------------------------------------------------
            // [NEW] "瞬时" 飞行命令 (fc_takeoff, fc_land, fc_vel)
            // ----------------------------------------------------
            bool fc_success = false;
            { // [NEW] 创建一个新作用域以持有飞行锁
                std::unique_lock<std::mutex> flight_lock(m_flight_lock);
                USER_LOG_INFO("PSDKServer: Flight lock acquired for short task '%s'.", cmd_name.c_str());

                if (!m_flightHandler)
                {
                    USER_LOG_ERROR("PSDKServer: Flight handler not initialized, cannot %s.", cmd_name.c_str());
                }
                else
                {
                    T_DjiReturnCode ret;
                    if (cmd_name == "fc_takeoff")
                    {
                        ret = m_flightHandler->takeoff();
                    }
                    else if (cmd_name == "fc_land")
                    {
                        ret = m_flightHandler->land();
                    }
                    else // fc_vel 或 fc_vel_gnd
                    {
                        float x = 0, y = 0, z = 0, yaw = 0;
                        if (!(ss >> x >> y >> z >> yaw))
                        {
                            USER_LOG_WARN("PSDKServer: Invalid %s args", cmd_name.c_str());
                            ret = DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
                        }
                        else
                        {
                            FlightControllerHandler::HorizontalCoordinate h_coord = (cmd_name == "fc_vel") ? FlightControllerHandler::HorizontalCoordinate::BODY : FlightControllerHandler::HorizontalCoordinate::GROUND;
                            ret = m_flightHandler->executeJoystickControl(x, y, z, yaw, FlightControllerHandler::HorizontalLogic::VELOCITY, h_coord);
                        }
                    }

                    if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                    {
                        fc_success = true;
                    }
                    else
                    {
                        USER_LOG_ERROR("PSDKServer: Command '%s' failed, ret=0x%08llX", cmd_name.c_str(), ret);
                    }
                }
                USER_LOG_INFO("PSDKServer: Flight lock released for short task '%s'.", cmd_name.c_str());
            } // [NEW] 飞行锁在此处释放

            sendSimpleResponse(client_socket, fc_success ? RESPONSE_CODE_ACCEPTED : RESPONSE_CODE_FAIL);
        }
        // else if (cmd_name == "fc_regain_ctrl") // [!!!! 在此添加新块 !!!!]
        // {
        //     // ----------------------------------------------------
        //     // [NEW] "瞬时" 飞行命令 (fc_regain_ctrl)
        //     // ----------------------------------------------------
        //     bool fc_success = false;
        //     { 
        //         std::unique_lock<std::mutex> flight_lock(m_flight_lock);
        //         USER_LOG_INFO("PSDKServer: Flight lock acquired for short task '%s'.", cmd_name.c_str());

        //         if (!m_flightHandler)
        //         {
        //             USER_LOG_ERROR("PSDKServer: Flight handler not initialized, cannot %s.", cmd_name.c_str());
        //         }
        //         else
        //         {
        //             // 调用我们新创建的函数
        //             T_DjiReturnCode ret = m_flightHandler->reObtainJoystickCtrlAuthority();
        //             if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        //             {
        //                 fc_success = true;
        //             }
        //             else
        //             {
        //                 USER_LOG_ERROR("PSDKServer: Command '%s' failed, ret=0x%08llX", cmd_name.c_str(), ret);
        //             }
        //         }
        //         USER_LOG_INFO("PSDKServer: Flight lock released for short task '%s'.", cmd_name.c_str());
        //     } 

        //     sendSimpleResponse(client_socket, fc_success ? RESPONSE_CODE_ACCEPTED : RESPONSE_CODE_FAIL);
        // }
        else if (cmd_name == "fc_pos" || cmd_name == "fc_pos_gnd" || cmd_name == "fc_seq" || cmd_name == "fc_pos_gps"|| cmd_name == "fc_pos_wp")
        {
            // ----------------------------------------------------
            // [修改] "长时" 飞行命令 (fc_pos, fc_seq, fc_pos_gps, fc_pos_wp)
            // ----------------------------------------------------

            // 1. 立即响应 Python 客户端，告诉它任务已接受
            sendSimpleResponse(client_socket, RESPONSE_CODE_ACCEPTED); // 1 = Accepted

            // 2. 将完整的命令字符串 (cmd_str) 和 client_socket 移交给新线程执行
            USER_LOG_INFO("PSDKServer: Offloading long task '%s' to worker thread.", cmd_name.c_str());
            std::thread(&PSDKServer::executeLongFlightTask, this, cmd_str, client_socket).detach();

            // [REMOVED] 所有阻塞循环, close() 和 return
        }
        else if (cmd_name == "get_gps" || cmd_name == "get_pose")
        {
            // ----------------------------------------------------
            // 状态获取: (非阻塞，非飞行)
            // ----------------------------------------------------
            if (!m_flightHandler)
            {
                USER_LOG_ERROR("PSDKServer: Flight handler not initialized, cannot %s.", cmd_name.c_str());
                sendSimpleResponse(client_socket, RESPONSE_CODE_FAIL);
            }
            else if (cmd_name == "get_gps")
            {
                double lat, lon;
                T_DjiReturnCode ret = m_flightHandler->getGpsPosition(lat, lon);
                if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                {
                    std::stringstream json_ss;
                    json_ss << std::fixed << std::setprecision(8);
                    json_ss << "{\"lat\": " << lat << ", \"lon\": " << lon << "}";
                    std::string json_response = json_ss.str();
                    uint32_t net_size = htonl(json_response.length());
                    send(client_socket, &net_size, sizeof(net_size), 0);
                    send(client_socket, json_response.c_str(), json_response.length(), 0);
                }
                else
                {
                    sendSimpleResponse(client_socket, RESPONSE_CODE_FAIL);
                }
            }
            else // get_pose
            {
                double lat, lon;
                float alt_m, yaw_deg;
                T_DjiReturnCode ret_gps = m_flightHandler->getGpsPosition(lat, lon);
                T_DjiReturnCode ret_alt = m_flightHandler->getRelativeAltitude(alt_m);
                T_DjiReturnCode ret_yaw = m_flightHandler->getYawAngle(yaw_deg);

                if (ret_gps == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && ret_alt == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && ret_yaw == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                {
                    std::stringstream json_ss;
                    json_ss << std::fixed << std::setprecision(8);
                    json_ss << "{\"lat\": " << lat << ", \"lon\": " << lon;
                    json_ss << std::setprecision(2);
                    json_ss << ", \"alt_m\": " << alt_m << ", \"yaw_deg\": " << yaw_deg << "}";
                    std::string json_response = json_ss.str();
                    uint32_t net_size = htonl(json_response.length());
                    send(client_socket, &net_size, sizeof(net_size), 0);
                    send(client_socket, json_response.c_str(), json_response.length(), 0);
                }
                else
                {
                    sendSimpleResponse(client_socket, RESPONSE_CODE_FAIL);
                }
            }
        }
        else
        {
            // ----------------------------------------------------
            // 无效命令
            // ----------------------------------------------------
            USER_LOG_WARN("PSDKServer: Received invalid or unknown command: %s", cmd_str.c_str());
            sendSimpleResponse(client_socket, RESPONSE_CODE_FAIL); // 0 字节表示失败
        }
        USER_LOG_INFO("PSDKServer: Flight CMD '%s' finished processing in handleClient.", cmd_name.c_str());

        // [REMOVED] close(client_socket)
        // [REMOVED] flight_lock release log

    } // [NEW] 循环到这里，等待下一个 read()

    // 4. [NEW] 循环结束后 (客户端断开连接)，关闭套接字
    close(client_socket);
    USER_LOG_INFO("PSDKServer: Connection closed.");

    // [REMOVED] 不再需要单独的飞行锁释放日志
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/