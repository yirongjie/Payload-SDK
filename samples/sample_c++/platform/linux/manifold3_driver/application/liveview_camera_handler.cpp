/**
 ********************************************************************
 * @file    liveview_camera_handler.cpp
 * @brief   LiveviewCameraHandler 类的实现
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "liveview_camera_handler.hpp"

// PSDK 核心库
#include "dji_logger.h"
#include "dji_platform.h"
#include <time.h>

// FFmpeg
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

// OpenCV (现在用于缩放和编码)
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

// C++ 库
#include <chrono>
#include <sstream>
#include <string.h> // for memset

/* T_H264DecoderContext 结构体定义 ------------------------------------------*/
typedef struct T_H264DecoderContext
{
    AVCodec *codec;
    AVCodecContext *context;
    AVFrame *frame;
    AVPacket *packet;
    SwsContext *swsContext;
    AVCodecParserContext *parser;
} T_H264DecoderContext;

/* LiveviewCameraHandler: 静态成员初始化 -----------------------------------*/
LiveviewCameraHandler *LiveviewCameraHandler::s_instance = nullptr;

/* LiveviewCameraHandler: 公共方法 ----------------------------------------*/
LiveviewCameraHandler::LiveviewCameraHandler(E_DjiMountPosition mountPosition)
    : m_mountPosition(mountPosition),
      m_captureRequest(false),
      m_captureStatus(0), // 0 = 待定
      m_requestedHeight(0)
{
    T_DjiReturnCode returnCode;

    m_decoder = new T_H264DecoderContext();
    memset(m_decoder, 0, sizeof(T_H264DecoderContext));

    s_instance = this;

    USER_LOG_INFO("LiveviewCameraHandler: Initializing...");

    returnCode = DjiLiveview_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Liveview init failed, error code: 0x%08X", returnCode);
        return;
    }

    returnCode = initDecoder();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("H.264 Decoder init failed, error code: 0x%08X", returnCode);
        return;
    }

    USER_LOG_INFO("LiveviewCameraHandler: Starting persistent H.264 stream...");
    returnCode = DjiLiveview_StartH264Stream((E_DjiLiveViewCameraPosition)m_mountPosition,
                                             DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                                             DjiTest_PayloadCameraStreamCallback_Wrapper);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Failed to start persistent H.264 stream, error code: 0x%08X", returnCode);
    }
    else
    {
        USER_LOG_INFO("LiveviewCameraHandler: Initialized successfully. Stream is running.");
    }
}

LiveviewCameraHandler::~LiveviewCameraHandler()
{
    USER_LOG_INFO("LiveviewCameraHandler: Deinitializing...");

    USER_LOG_INFO("LiveviewCameraHandler: Stopping persistent H.264 stream...");
    DjiLiveview_StopH264Stream((E_DjiLiveViewCameraPosition)m_mountPosition,
                               DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);

    deinitDecoder();

    DjiLiveview_Deinit();

    if (m_decoder)
    {
        delete m_decoder;
        m_decoder = nullptr;
    }

    if (s_instance == this)
    {
        s_instance = nullptr;
    }

    USER_LOG_INFO("LiveviewCameraHandler: Deinitialized.");
}

/**
 * @brief (已修复) takePhoto - 使用 avcodec_flush_buffers
 */
T_DjiReturnCode LiveviewCameraHandler::takePhoto(int resizeHeight, std::vector<uint8_t> &outJpegBuffer)
{
    USER_LOG_INFO("takePhoto: Requesting frame capture (Height: %d)...", resizeHeight);

    // Mutex 必须在 flush 之前锁定，以防止回调在 flush 期间访问解码器
    std::unique_lock<std::mutex> lock(m_photoMutex);

    m_requestedHeight = resizeHeight;
    m_captureStatus = 0; // 0 = 待定
    m_jpegBuffer.clear();

    // *** 关键修复：清空解码器缓冲区 ***
    // 这会丢弃所有旧的、损坏的、或已缓存的帧
    // 迫使解码器必须等待一个新的 I-frame 才能再次成功解码
    if (m_decoder && m_decoder->context)
    {
        USER_LOG_INFO("takePhoto: Flushing FFmpeg decoder buffers...");
        avcodec_flush_buffers(m_decoder->context);
    }
    // *** 修复结束 ***

    m_captureRequest = true; // 打开"门锁"

    // 等待回调函数发回信号 (成功=1 或 失败=-1)
    if (m_captureCv.wait_for(lock, std::chrono::seconds(2), [this]
                             {
                                 return m_captureStatus.load() != 0; // 等待直到状态不再是 "待定"
                             }))
    {
        // 收到信号
        if (m_captureStatus.load() == 1) // 1 = 成功
        {
            outJpegBuffer = m_jpegBuffer; // 拷贝数据
            USER_LOG_INFO("takePhoto: Capture successful. JPEG size: %lu bytes", outJpegBuffer.size());
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else // -1 = 失败
        {
            USER_LOG_ERROR("takePhoto: Capture reported failure.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    }
    else
    {
        // 超时
        USER_LOG_ERROR("takePhoto: Capture failed (timeout).");
        m_captureRequest = false; // 确保门锁被关闭
        m_captureStatus = -1;     // 标记为失败
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
}

/* LiveviewCameraHandler: 私有方法 ----------------------------------------*/

/**
 * @brief (已修复) 回调函数 - 只接受 I-frame
 */
void LiveviewCameraHandler::payloadCameraStreamCallback(const uint8_t *buf, uint32_t bufLen)
{
    // 1. 检查 "门锁"
    if (!m_captureRequest.load())
    {
        return;
    }

    // 锁定互斥锁，以保护对解码器的访问 (因为 takePhoto 可能会 flush)
    std::lock_guard<std::mutex> lock(m_photoMutex);

    // 再次检查"门锁"，因为它可能在我们等待锁时被关闭了
    if (!m_captureRequest.load())
    {
        return;
    }

    // 2. "门锁"是打开的。尝试解码...
    cv::Mat decodedFrame;
    AVPictureType pictType = AV_PICTURE_TYPE_NONE; // 初始化为 None

    T_DjiReturnCode decodeRet = processFrame(buf, bufLen, decodedFrame, pictType);

    if (decodeRet != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || decodedFrame.empty())
    {
        // 解码失败 (例如: 只是一个P帧, 或流损坏)
        // "门锁" 保持打开，等待下一个数据包
        return;
    }

    // --- 解码成功！我们拿到了一个完整的帧 ---

    // 3. 检查帧类型
    if (pictType != AV_PICTURE_TYPE_I)
    {
        // 这不是一个 I-frame。它可能是我们不想要的 P-frame 或 B-frame。
        // 丢弃它，保持"门锁"打开，等待我们请求的 I-frame。
        USER_LOG_INFO("payloadCameraStreamCallback: Discarded P/B-frame. Waiting for I-frame...");
        return;
    }

    // --- 这是一个 I-frame！ ---
    USER_LOG_INFO("payloadCameraStreamCallback: I-Frame captured!");

    // 4. 尝试缩放和编码
    std::vector<uint8_t> tempJpegBuffer;
    T_DjiReturnCode encodeRet = resizeAndEncode(decodedFrame, m_requestedHeight, tempJpegBuffer);

    if (encodeRet != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        // 编码失败 (例如: 帧是损坏的)
        USER_LOG_WARN("payloadCameraStreamCallback: resizeAndEncode failed, retrying with next frame...");
        // "门锁" 保持打开，等待下一个数据包
        return;
    }

    // --- 编码也成功了！---

    // 5. 立即关闭“门锁”
    m_captureRequest = false;

    // 6. 设置结果 (我们已经持有锁)
    m_jpegBuffer = std::move(tempJpegBuffer);
    m_captureStatus = 1; // 1 = 成功

    // 7. 唤醒 takePhoto 线程
    m_captureCv.notify_one();
}

/**
 * @brief (已修复) 解码一帧, 报告帧类型, 并清空缓冲区
 */
T_DjiReturnCode LiveviewCameraHandler::processFrame(const uint8_t *buf, uint32_t bufLen, cv::Mat &outFrame, AVPictureType &outPictType)
{
    uint8_t *data = (uint8_t *)buf;
    int size = (int)bufLen;
    int ret;
    int consumed_size = 0;

    // 标志，确保我们只返回一帧
    bool frame_captured = false;
    outPictType = AV_PICTURE_TYPE_NONE; // 默认值

    while (size > 0)
    {
        av_frame_unref(m_decoder->frame);
        consumed_size = av_parser_parse2(m_decoder->parser, m_decoder->context, &m_decoder->packet->data, &m_decoder->packet->size, data, size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
        if (consumed_size < 0)
        {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }

        data += consumed_size;
        size -= consumed_size;

        if (m_decoder->packet->size)
        {
            ret = avcodec_send_packet(m_decoder->context, m_decoder->packet);
            av_packet_unref(m_decoder->packet);

            if (ret < 0)
            {
                if (ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
                {
                    USER_LOG_ERROR("Error sending packet to decoder: %d", ret);
                }
            }

            // 循环以清空解码器缓冲区
            while (ret >= 0)
            {
                ret = avcodec_receive_frame(m_decoder->context, m_decoder->frame);

                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                {
                    // 这是正常情况，意味着解码器需要更多数据
                    break;
                }
                else if (ret < 0)
                {
                    USER_LOG_ERROR("Error receiving frame: %d", ret);
                    return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
                }

                // 成功解码一帧

                // 如果我们 *尚未* 捕获此请求的帧
                if (!frame_captured)
                {
                    m_decoder->swsContext = sws_getCachedContext(m_decoder->swsContext, m_decoder->context->width, m_decoder->context->height, m_decoder->context->pix_fmt, m_decoder->context->width, m_decoder->context->height, AV_PIX_FMT_BGR24, SWS_BILINEAR, NULL, NULL, NULL);
                    if (!m_decoder->swsContext)
                    {
                        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
                    }

                    outFrame.create(m_decoder->context->height, m_decoder->context->width, CV_8UC3);
                    uint8_t *dst_data[1] = {outFrame.data};
                    int dst_linesize[1] = {(int)outFrame.step};

                    sws_scale(m_decoder->swsContext, m_decoder->frame->data, m_decoder->frame->linesize, 0, m_decoder->context->height, dst_data, dst_linesize);

                    frame_captured = true;                     // 标记我们已经拿到了
                    outPictType = m_decoder->frame->pict_type; // 存储帧类型
                }

                // 关键修复: 无论我们是否使用了这一帧，都必须 unref 它
                av_frame_unref(m_decoder->frame);
            }
        }
    }

    // 只有当我们成功捕获了一帧时才返回 SUCCESS
    if (frame_captured)
    {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    else
    {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR; // 没有完整的帧
    }
}

/**
 * @brief 缩放图像并将其编码为内存中的 JPEG
 */
T_DjiReturnCode LiveviewCameraHandler::resizeAndEncode(const cv::Mat &frame, int targetHeight, std::vector<uint8_t> &outBuffer)
{
    try
    {
        cv::Mat resizedFrame;

        // 1. 计算新的宽度以保持宽高比
        double aspectRatio = (double)frame.cols / (double)frame.rows;
        int targetWidth = (int)(targetHeight * aspectRatio);

        // 2. 执行缩放 (INTER_AREA 适用于缩小)
        cv::resize(frame, resizedFrame, cv::Size(targetWidth, targetHeight), 0, 0, cv::INTER_AREA);

        // 3. 将缩放后的图像编码为 JPEG 存入内存
        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(90); // 90% 质量

        if (cv::imencode(".jpg", resizedFrame, outBuffer, params))
        {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else
        {
            USER_LOG_ERROR("cv::imencode failed!");
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    }
    catch (const cv::Exception &ex)
    {
        USER_LOG_ERROR("OpenCV Exception in resizeAndEncode: %s", ex.what());
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
}

T_DjiReturnCode LiveviewCameraHandler::initDecoder()
{
    m_decoder->codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!m_decoder->codec)
    {
        USER_LOG_ERROR("H.264 codec not found.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    m_decoder->context = avcodec_alloc_context3(m_decoder->codec);
    if (!m_decoder->context)
    {
        USER_LOG_ERROR("Could not allocate AVCodecContext.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    m_decoder->context->thread_count = 4;
    // 允许解码器处理损坏的流
    m_decoder->context->flags |= AV_CODEC_FLAG_TRUNCATED;

    if (avcodec_open2(m_decoder->context, m_decoder->codec, NULL) < 0)
    {
        USER_LOG_ERROR("Could not open H.264 codec.");
        avcodec_free_context(&m_decoder->context);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    m_decoder->parser = av_parser_init(AV_CODEC_ID_H264);
    if (!m_decoder->parser)
    {
        USER_LOG_ERROR("Could not initialize H.264 parser.");
        avcodec_close(m_decoder->context);
        avcodec_free_context(&m_decoder->context);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    m_decoder->frame = av_frame_alloc();
    m_decoder->packet = av_packet_alloc();
    if (!m_decoder->frame || !m_decoder->packet)
    {
        USER_LOG_ERROR("Could not allocate frame or packet.");
        av_parser_close(m_decoder->parser);
        avcodec_close(m_decoder->context);
        avcodec_free_context(&m_decoder->context);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    USER_LOG_INFO("H.264 Decoder initialized successfully.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void LiveviewCameraHandler::deinitDecoder()
{
    if (m_decoder->parser)
    {
        av_parser_close(m_decoder->parser);
        m_decoder->parser = NULL;
    }
    if (m_decoder->swsContext)
    {
        sws_freeContext(m_decoder->swsContext);
        m_decoder->swsContext = NULL;
    }
    if (m_decoder->packet)
    {
        av_packet_free(&m_decoder->packet);
    }
    if (m_decoder->frame)
    {
        av_frame_free(&m_decoder->frame);
    }
    if (m_decoder->context)
    {
        avcodec_close(m_decoder->context);
        avcodec_free_context(&m_decoder->context);
    }
    m_decoder->codec = NULL;
    USER_LOG_INFO("H.264 Decoder deinitialized.");
}

/* 静态回调包装器 --------------------------------------------------------*/
void LiveviewCameraHandler::DjiTest_PayloadCameraStreamCallback_Wrapper(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen)
{
    if (s_instance)
    {
        s_instance->payloadCameraStreamCallback(buf, bufLen);
    }
}