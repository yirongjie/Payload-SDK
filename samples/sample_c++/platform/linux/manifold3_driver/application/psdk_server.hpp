/**
 ********************************************************************
 * @file    psdk_server.hpp
 * @brief   PSDK 拍照服务 (TCP 服务器)
 *
 *********************************************************************
 */

#ifndef PSDK_SERVER_HPP
#define PSDK_SERVER_HPP

// PSDK 类型
#include "dji_typedef.h"

// C++ 核心库
#include <atomic>
#include <thread>
#include <string>
#include <memory> // 用于 std::unique_ptr
#include <mutex>

// 向前声明
class LiveviewCameraHandler;
class FlightControllerHandler;
class WaypointMissionHandler;

/**
 * @class PSDKServer
 * @brief 运行一个 TCP 服务器，监听拍照和飞行请求
 */
class PSDKServer
{
public:
    /**
     * @brief 构造函数
     * @param port 监听的端口 (例如 8899)
     * @param saveLocalCopy 是否在服务器本地保存一份副本, 默认为 false
     */
    PSDKServer(int port, bool saveLocalCopy = false);
    ~PSDKServer();

    // 禁用拷贝和赋值
    PSDKServer(const PSDKServer &) = delete;
    PSDKServer &operator=(const PSDKServer &) = delete;

    /**
     * @brief 启动服务器（在单独的线程中）
     */
    bool start();

    /**
     * @brief 停止服务器
     */
    void stop();

private:
    void listenLoop();
    void handleClient(int client_socket);

    /**
     * @brief 发送一个简单的响应码 (0=失败, 1=成功, 2=任务完成)
     */
    void sendSimpleResponse(int client_socket, uint32_t response_code);

    /**
     * @brief [修改] 在一个单独的线程中执行长时飞行任务 (非阻塞)
     * @param cmd_str 客户端发送的完整命令字符串
     * @param client_socket [新增] 用于发送最终完成信号的套接字
     */
    void executeLongFlightTask(std::string cmd_str, int client_socket);

    int m_port;
    int m_server_fd;
    std::thread m_listenThread;
    std::atomic<bool> m_running;
    bool m_saveLocalCopy; // 标志位：是否本地保存图片

    std::mutex m_flight_lock;

    // PSDKServer 现在拥有 LiveviewCameraHandler
    std::unique_ptr<LiveviewCameraHandler> m_cameraHandler;

    std::unique_ptr<FlightControllerHandler> m_flightHandler;

    std::unique_ptr<WaypointMissionHandler> m_waypointHandler;
};

#endif // PSDK_SERVER_HPP