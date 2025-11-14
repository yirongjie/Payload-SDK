/**
 ********************************************************************
 * @file    liveview_camera_handler.hpp
 * @brief   PSDK 实时图传解码与拍照功能
 *
 *********************************************************************
 */

#ifndef LIVEVIEW_CAMERA_HPP
#define LIVEVIEW_CAMERA_HPP

#include "dji_typedef.h"
#include "dji_liveview.h"

// C++ 核心库
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <vector>
#include <string>

// 包含 libavutil 以获取 AVPictureType 的定义
extern "C"
{
#include <libavutil/avutil.h>
}

// 向前声明
typedef struct T_H264DecoderContext T_H264DecoderContext;
namespace cv
{
    class Mat;
} // 向前声明 OpenCV Mat

/**
 * @class LiveviewCameraHandler
 * @brief 封装了 PSDK liveview 的 H.264 解码和 *线程安全* 的拍照功能
 * (修复版：强制清空解码器缓存并捕获 I-Frame)
 */
class LiveviewCameraHandler
{
public:
    LiveviewCameraHandler(E_DjiMountPosition mountPosition);
    ~LiveviewCameraHandler();

    /**
     * @brief 执行一次拍照动作 (瞬时)
     * @param resizeHeight [in] 客户端请求的目标高度 (例如 480, 720, 1080)
     * @param outJpegBuffer [out] 编码后的 JPEG 数据将被写入此 vector
     * @return T_DjiReturnCode 操作结果
     */
    T_DjiReturnCode takePhoto(int resizeHeight, std::vector<uint8_t> &outJpegBuffer);

private:
    // --- 内部状态与成员 ---
    T_H264DecoderContext *m_decoder;
    E_DjiMountPosition m_mountPosition;

    // --- 线程同步 ---
    std::atomic<bool> m_captureRequest; // "门锁": true = G" 正在寻找帧
    std::condition_variable m_captureCv;
    std::mutex m_photoMutex;

    /**
     * @brief 0 = 待定, 1 = 成功, -1 = 失败/超时
     */
    std::atomic<int> m_captureStatus;

    // --- 用于传递数据的成员 ---
    int m_requestedHeight;
    std::vector<uint8_t> m_jpegBuffer;

    // --- 私有辅助方法 ---
    T_DjiReturnCode initDecoder();
    void deinitDecoder();

    /**
     * @brief 解码一帧数据
     * @param outFrame [out] 解码后的 OpenCV Mat
     * @param outPictType [out] 解码帧的类型 (I, P, B)
     */
    T_DjiReturnCode processFrame(const uint8_t *buf, uint32_t bufLen, cv::Mat &outFrame, AVPictureType &outPictType);

    T_DjiReturnCode resizeAndEncode(const cv::Mat &frame, int targetHeight, std::vector<uint8_t> &outBuffer);

    // --- 回调实现 ---
    void payloadCameraStreamCallback(const uint8_t *buf, uint32_t bufLen);

    // --- 静态回调包装器 ---
    static void DjiTest_PayloadCameraStreamCallback_Wrapper(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen);
    static LiveviewCameraHandler *s_instance;
};

#endif // LIVEVIEW_CAMERA_HPP