/**
 ********************************************************************
 * @file    camera_handler.cpp
 * @brief   CameraHandler 类的实现 (基于 Camera Manager，使用磁盘作为中介)
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "camera_handler.hpp"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_camera_manager.h" 

// C++ 核心库
#include <string.h>
#include <algorithm>
#include <chrono>
#include <vector>
#include <iostream>
#include <fstream>
#include <mutex>
#include <condition_variable>

// POSIX 文件操作
#include <stdio.h>  // for FILE, fopen, fwrite, fclose, snprintf
#include <unistd.h> // for remove() - DELETE file on disk
#include <sys/stat.h> // for mkdir


#include <opencv2/opencv.hpp>

// 外部 C 函数声明 (来自 test_camera_manager.h)
extern "C" {
    // 对应用户操作 7 (Shoot Single Photo)
    T_DjiReturnCode DjiTest_CameraManagerStartShootSinglePhoto(E_DjiMountPosition position);
}

// -----------------------------------------------------------------------------
// 静态成员和回调函数实现 (全局状态)
// -----------------------------------------------------------------------------

CameraHandler *CameraHandler::s_instance = nullptr;

// 使用静态/全局内存来存储文件列表
static T_DjiCameraManagerFileList s_latestFileList; 

// --- C-style File Download Statics ---
static FILE *s_downloadMediaFile = NULL;
static char s_downloadFileName[256] = {0};
static std::mutex s_fileDownloadMutex; 
static std::condition_variable s_fileDownloadCv;
static T_DjiReturnCode s_downloadStatus = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN; 
const char* DOWNLOAD_TMP_PATH = "/home/dji/LMFly/UAV-Isaac-GR00T/uav_script/tmp";


/**
 * @brief C-style Download Data Callback (writes to disk)
 */
T_DjiReturnCode DjiTest_CameraManagerDownloadFileDataCallback_File(T_DjiDownloadFilePacketInfo packetInfo,
                                                                  const uint8_t *data, uint16_t len)
{
    // 必须保护静态变量和文件句柄
    std::lock_guard<std::mutex> lock(s_fileDownloadMutex);
    
    // 确定是否为成功事件
    bool isSuccessEvent = (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_END || 
                           packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START_TRANSFER_END);
    
    if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START) {
        
        // 查找原始文件名 
        const char* originalFileName = NULL; 
        for (uint32_t i = 0; i < s_latestFileList.totalCount; ++i) {
            if (s_latestFileList.fileListInfo[i].fileIndex == packetInfo.fileIndex) {
                 originalFileName = s_latestFileList.fileListInfo[i].fileName;
                 break;
            }
        }
        
        // 构建文件路径
        if (originalFileName == NULL) {
            // 使用一个带索引的通用临时名作为回退
            snprintf(s_downloadFileName, sizeof(s_downloadFileName), 
                     "%s/temp_downloaded_file_%d", DOWNLOAD_TMP_PATH, packetInfo.fileIndex); 
        } else {
            // 使用原始文件名
            snprintf(s_downloadFileName, sizeof(s_downloadFileName), 
                     "%s/%s", 
                     DOWNLOAD_TMP_PATH, originalFileName);
        }
        
        USER_LOG_INFO("DownloadCallback: Start saving file %s, total size: %u", 
                      s_downloadFileName, packetInfo.fileSize);
                      
        s_downloadStatus = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        s_downloadMediaFile = fopen(s_downloadFileName, "wb+");
        if (s_downloadMediaFile == NULL) {
            USER_LOG_ERROR("DownloadCallback: Failed to open file %s", s_downloadFileName);
            s_downloadStatus = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
            s_fileDownloadCv.notify_one();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
        fwrite(data, 1, len, s_downloadMediaFile);

    } else if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_TRANSFER) {
        if (s_downloadMediaFile != NULL) {
            fwrite(data, 1, len, s_downloadMediaFile);
        }

    } else if (isSuccessEvent) {
        
        if (s_downloadMediaFile != NULL) {
            fwrite(data, 1, len, s_downloadMediaFile);
            fclose(s_downloadMediaFile);
            s_downloadMediaFile = NULL;
        }
            
        USER_LOG_INFO("DownloadCallback: File %s complete. Final size: %u", 
                      s_downloadFileName, packetInfo.fileSize);
                      
        s_downloadStatus = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        s_fileDownloadCv.notify_one(); // Signal completion
        
    } else {
         // 捕获所有未知的非成功事件
         if (s_downloadMediaFile != NULL) {
             fclose(s_downloadMediaFile);
             s_downloadMediaFile = NULL;
         }
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


/* CameraHandler: 公共方法实现 ----------------------------------------*/

CameraHandler::CameraHandler(E_DjiMountPosition mountPosition)
    : m_mountPosition(mountPosition)
{
    USER_LOG_INFO("CameraHandler: Initializing with Camera Manager (Mount Position %d)", mountPosition);
    s_instance = this;
    
    // 确保下载目录存在 (忽略错误)
    mkdir(DOWNLOAD_TMP_PATH, 0777); 

    // 1. 初始化 Camera Manager
    T_DjiReturnCode ret = DjiCameraManager_Init();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Camera Manager init failed, error code: 0x%08X", ret);
    }
    
    // 2. 注册文件下载回调 (使用 C-style 磁盘写入回调)
    ret = DjiCameraManager_RegDownloadFileDataCallback(m_mountPosition, DjiTest_CameraManagerDownloadFileDataCallback_File);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Register download callback failed, error code: 0x%08X", ret);
    }

    USER_LOG_INFO("CameraHandler: Initialized successfully. Disk download callback registered.");
}

CameraHandler::~CameraHandler()
{
    USER_LOG_INFO("CameraHandler: Deinitializing...");
    DjiCameraManager_ReleaseDownloaderRights(m_mountPosition);
    DjiCameraManager_DeInit();
    s_instance = nullptr;
}


T_DjiReturnCode CameraHandler::takePhoto(int resizeHeight, std::vector<uint8_t> &outJpegBuffer)
{
    T_DjiReturnCode ret;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    outJpegBuffer.clear(); // 清空输出缓冲区

    // =========================================================================
    // 步骤 1: 拍照 (对应输入 7: Shoot Single Photo)
    // =========================================================================
    USER_LOG_INFO("takePhoto: Step 1/7 - Start shooting single photo at position %d", m_mountPosition);
    ret = DjiTest_CameraManagerStartShootSinglePhoto(m_mountPosition);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("takePhoto: Shoot single photo failed, error code: 0x%08X", ret);
        return ret;
    }
    
    // 等待 2.5 秒，确保文件创建完成
    USER_LOG_INFO("takePhoto: Waiting 2.5s for photo file creation...");
    osalHandler->TaskSleepMs(2500);

    // =========================================================================
    // 步骤 2: 获取下载器权限 (对应输入 11 的第一步)
    // =========================================================================
    USER_LOG_INFO("takePhoto: Step 2/7 - Obtaining downloader rights...");
    ret = DjiCameraManager_ObtainDownloaderRights(m_mountPosition);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("takePhoto: Obtain downloader rights failed, error code: 0x%08X", ret);
        return ret;
    }

    // =========================================================================
    // 步骤 3: 下载文件列表 
    // =========================================================================
    USER_LOG_INFO("takePhoto: Step 3/7 - Downloading file list...");
    // 清空静态文件列表，并获取最新列表
    memset(&s_latestFileList, 0, sizeof(s_latestFileList));
    ret = DjiCameraManager_DownloadFileList(m_mountPosition, &s_latestFileList);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("takePhoto: Download file list failed, error code: 0x%08X", ret);
        DjiCameraManager_ReleaseDownloaderRights(m_mountPosition);
        return ret;
    }

    if (s_latestFileList.totalCount == 0) {
        USER_LOG_ERROR("takePhoto: File list is empty after shooting.");
        DjiCameraManager_ReleaseDownloaderRights(m_mountPosition);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    // 修正：假设列表按时间升序排列，最新的文件在列表末尾
    uint32_t latestFileIndexInList = s_latestFileList.totalCount - 1;
    auto latestFile = &s_latestFileList.fileListInfo[latestFileIndexInList]; 
    
    USER_LOG_INFO("takePhoto: Target file (Latest, Index %u): %s (File Index %d, Size %u bytes)", 
                  latestFileIndexInList, latestFile->fileName, latestFile->fileIndex, latestFile->fileSize);
    // =========================================================================
    // 步骤 4: 下载最新的媒体文件
    // =========================================================================
    USER_LOG_INFO("takePhoto: Step 4/7 - Starting download of latest file (Index %d)...", latestFile->fileIndex);

    // 重置下载状态和互斥体，准备接收回调
    {
        std::lock_guard<std::mutex> lock(s_fileDownloadMutex);
        s_downloadStatus = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        memset(s_downloadFileName, 0, sizeof(s_downloadFileName));
    }
    
    // 触发下载 (结果将通过异步回调 DjiTest_CameraManagerDownloadFileDataCallback_File 写入磁盘)
    ret = DjiCameraManager_DownloadFileByIndex(m_mountPosition, latestFile->fileIndex);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("takePhoto: Trigger download failed, error code: 0x%08X", ret);
        DjiCameraManager_ReleaseDownloaderRights(m_mountPosition);
        return ret;
    }
    
    // 等待下载完成的信号 (通过条件变量 s_fileDownloadCv)
    {
        std::unique_lock<std::mutex> lock(s_fileDownloadMutex);
        // 使用 30 秒超时等待下载完成
        if (s_fileDownloadCv.wait_for(lock, std::chrono::seconds(30), 
                                     [] { return s_downloadStatus != DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN; })) 
        {
            ret = s_downloadStatus; // 获取回调中的最终状态
        } else {
            ret = DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
            USER_LOG_ERROR("takePhoto: File download timed out.");
        }
    }
    
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("takePhoto: File download failed or timed out, status: 0x%08X", ret);
        DjiCameraManager_ReleaseDownloaderRights(m_mountPosition);
        return ret;
    }
    
    // =========================================================================
    // 步骤 5: 将下载到磁盘的文件内容读取到 outJpegBuffer
    // =========================================================================
    // USER_LOG_INFO("takePhoto: Step 5/7 - Reading downloaded file into buffer...");
    // std::ifstream file(s_downloadFileName, std::ios::binary | std::ios::ate);
    // if (file.is_open()) {
    //     std::streamsize fileSize = file.tellg();
    //     outJpegBuffer.resize(fileSize);
    //     file.seekg(0, std::ios::beg);
    //     file.read((char*)outJpegBuffer.data(), fileSize);
    //     file.close();
    //     USER_LOG_INFO("takePhoto: Read %zu bytes into outJpegBuffer.", outJpegBuffer.size());
    // } else {
    //     USER_LOG_ERROR("takePhoto: Failed to open downloaded file %s for reading.", s_downloadFileName);
    //     DjiCameraManager_ReleaseDownloaderRights(m_mountPosition);
    //     return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    // }
    // =========================================================================
    // 步骤 5: 将下载到磁盘的文件内容读取、缩放并编码
    // =========================================================================
    USER_LOG_INFO("takePhoto: Step 5/7 - Reading and processing downloaded file...");
    std::ifstream file(s_downloadFileName, std::ios::binary | std::ios::ate);
    
    if (file.is_open()) {
        std::streamsize fileSize = file.tellg();
        // 1. 读取原始文件内容到临时缓冲区
        std::vector<uint8_t> rawFileBuffer(fileSize);
        file.seekg(0, std::ios::beg);
        file.read((char*)rawFileBuffer.data(), fileSize);
        file.close();

        // 2. 检查是否需要缩放 (resizeHeight > 0)
        if (resizeHeight > 0) {
            USER_LOG_INFO("takePhoto: Resizing requested (Height: %d).", resizeHeight);
            try {
                // 3. 解码: 将 JPEG 内存缓冲区解码为 OpenCV Mat
                cv::Mat originalFrame = cv::imdecode(rawFileBuffer, cv::IMREAD_COLOR);
                if (originalFrame.empty()) {
                    USER_LOG_ERROR("takePhoto: cv::imdecode failed. Retaining original image.");
                    outJpegBuffer = std::move(rawFileBuffer);
                } else {
                    USER_LOG_INFO("takePhoto: Original resolution: %d x %d", originalFrame.cols, originalFrame.rows);

                    // 4. 缩放: (逻辑学习自 liveview_camera_handler.cpp::resizeAndEncode)
                    cv::Mat resizedFrame;
                    double aspectRatio = (double)originalFrame.cols / (double)originalFrame.rows;
                    int targetWidth = (int)(resizeHeight * aspectRatio);
                    
                    cv::resize(originalFrame, resizedFrame, cv::Size(targetWidth, resizeHeight), 0, 0, cv::INTER_AREA);

                    // 5. 编码: (逻辑学习自 liveview_camera_handler.cpp::resizeAndEncode)
                    std::vector<int> params;
                    params.push_back(cv::IMWRITE_JPEG_QUALITY);
                    params.push_back(90); // 90% 质量

                    if (cv::imencode(".jpg", resizedFrame, outJpegBuffer, params)) {
                        USER_LOG_INFO("takePhoto: Image resized to %dx%d. New JPEG size: %zu bytes", targetWidth, resizeHeight, outJpegBuffer.size());
                    } else {
                        USER_LOG_ERROR("takePhoto: cv::imencode failed! Retaining original image.");
                        outJpegBuffer = std::move(rawFileBuffer);
                    }
                }
            } catch (const cv::Exception &ex) {
                USER_LOG_ERROR("takePhoto: OpenCV Exception during resize: %s. Retaining original image.", ex.what());
                outJpegBuffer = std::move(rawFileBuffer);
            }
        } else {
            // 6. 不进行缩放 (resizeHeight = -1)，直接将原始文件内容移动到输出缓冲区
            outJpegBuffer = std::move(rawFileBuffer);
            USER_LOG_INFO("takePhoto: Resizing disabled. Read %zu bytes into outJpegBuffer.", outJpegBuffer.size());
        }
        
    } else {
        USER_LOG_ERROR("takePhoto: Failed to open downloaded file %s for reading.", s_downloadFileName);
        DjiCameraManager_ReleaseDownloaderRights(m_mountPosition);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    // =========================================================================
    // 步骤 6: 删除媒体文件 (对应输入 11 的删除功能)
    // =========================================================================
    USER_LOG_INFO("takePhoto: Step 6/7 - Deleting file (Index %d) from storage...", latestFile->fileIndex);
    ret = DjiCameraManager_DeleteFileByIndex(m_mountPosition, latestFile->fileIndex);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("takePhoto: Delete media file failed, error code: 0x%08X. Continuing...", ret);
    }

    // 删除本地临时文件
    if (remove(s_downloadFileName) != 0) {
        USER_LOG_WARN("takePhoto: Failed to delete local temp file: %s", s_downloadFileName);
    }
    
    // =========================================================================
    // 步骤 7: 释放下载器权限
    // =========================================================================
    USER_LOG_INFO("takePhoto: Step 7/7 - Releasing downloader rights.");
    DjiCameraManager_ReleaseDownloaderRights(m_mountPosition);
    
    USER_LOG_INFO("takePhoto: Process complete. Downloaded size: %zu bytes.", outJpegBuffer.size());
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}