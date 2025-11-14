/**
 ********************************************************************
 * @file    camera_handler.hpp
 * @brief   PSDK 相机拍照功能 (基于 Camera Manager，下载到磁盘中介)
 *
 * @note    此文件用于替代 LiveviewCameraHandler.hpp
 *********************************************************************
 */

#ifndef CAMERA_HANDLER_HPP
#define CAMERA_HANDLER_HPP

#include "dji_typedef.h"
#include "dji_camera_manager.h" 

// C++ 核心库
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>
#include <stdio.h> // For FILE and snprintf

/**
 * @brief C-style Download Data Callback (writes to disk)
 */
extern "C" T_DjiReturnCode DjiTest_CameraManagerDownloadFileDataCallback_File(
    T_DjiDownloadFilePacketInfo packetInfo,
    const uint8_t *data,
    uint16_t len);


/**
 * @class CameraHandler
 * @brief 封装了 PSDK Camera Manager 的拍照和下载功能（使用磁盘作为中介）
 */
class CameraHandler
{
public:
    CameraHandler(E_DjiMountPosition mountPosition);
    ~CameraHandler();

    // 禁用拷贝和赋值
    CameraHandler(const CameraHandler &) = delete;
    CameraHandler &operator=(const CameraHandler &) = delete;

    /**
     * @brief 执行拍照 -> 等待 -> 下载的同步动作，以替换 LiveviewCameraHandler
     * @param resizeHeight [in] 客户端请求的目标高度 (未实现缩放，仅为兼容性保留)
     * @param outJpegBuffer [out] 下载后的 JPEG 数据将被写入此 vector
     * @return T_DjiReturnCode 操作结果
     */
    T_DjiReturnCode takePhoto(int resizeHeight, std::vector<uint8_t> &outJpegBuffer);

private:
    E_DjiMountPosition m_mountPosition;
    
    // --- 静态实例和文件共享 ---
    static CameraHandler *s_instance; 
    
    // 友元声明，允许 C 回调访问和同步下载状态
    friend T_DjiReturnCode DjiTest_CameraManagerDownloadFileDataCallback_File(
        T_DjiDownloadFilePacketInfo packetInfo,
        const uint8_t *data,
        uint16_t len);
};

#endif // CAMERA_HANDLER_HPP