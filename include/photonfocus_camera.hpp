#ifndef PHOTONFOCUS_CAMERA_H
#define PHOTONFOCUS_CAMERA_H

#define BUFFER_COUNT 16 // TODO number of buffers from parameters?

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <opencv2/opencv.hpp>

#include "PvDeviceGEV.h"
#include "PvStreamGEV.h"
#include "PvPipeline.h"

// THIS MACRO EXPLOITS RESULT DESCRIPTION FOR THROWING EXCEPTIONS ON EXPR WHICH RETURNS A PvResult
#define CHECK_RESULT(expression) \
do { \
    try { \
        const PvResult& result_ = expression; \
        if (!result_.IsOK()) { \
            throw std::runtime_error("Error in " + std::string(__FILE__) + " at line " + std::to_string(__LINE__) + ": " + result_.GetDescription().GetAscii()); \
        } \
    } catch (const std::exception& e) { \
        throw std::runtime_error("Exception in " + std::string(__FILE__) + " at line " + std::to_string(__LINE__) + ": " + e.what()); \
    } \
} while (false)

namespace AIRLab
{

// this was imported from driver_common, since the latter seems deprecated.
enum {
  RECONFIGURE_CLOSE = 3,
  RECONFIGURE_STOP = 1,
  RECONFIGURE_RUNNING = 0,
};

class PhotonFocusCamera
{
    // FIXME it is necessary to specialize the exceptions...
    PvDevice* device;
    PvStream* stream;
    PvPipeline* pipeline;
    PvString camera_id; // IP

    PvGenParameterArray* device_parameters;
    PvGenParameterArray* stream_parameters;

    std::shared_ptr<std::thread> image_thread;

public:
    std::function<void(const cv::Mat &image)> callback;

    PhotonFocusCamera(const std::string& ip_address);
    ~PhotonFocusCamera();

    void start();
    void stop();

    template <typename ParamType, typename ValueType>
    ValueType getDeviceAttribute(std::string name, ValueType* min = nullptr, ValueType* max = nullptr);

    template <typename ParamType, typename ValueType>
    void setDeviceAttribute(std::string name, ValueType value);

private:
    std::atomic<bool> stop_thread { false };
    std::mutex mtx;

    void open();
    void close();
    void acquireImages();
    void interruptThread();

    PvAccessType getAccessType();
};

}
#endif // PHOTONFOCUS_CAMERA_H