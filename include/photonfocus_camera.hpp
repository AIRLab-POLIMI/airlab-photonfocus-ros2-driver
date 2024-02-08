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

    template <typename ParamType, typename ValueType, typename SetterFunction>
    void setDeviceAttribute(std::string name, ValueType value, SetterFunction setter) {
        if (!device_parameters)
            throw std::runtime_error("Device parameters are not yet initialized.");

        ParamType* parameter = dynamic_cast<ParamType*>(device->GetParameters()->Get(PvString(name.c_str())));

        if (!parameter)
            throw std::runtime_error("Attribute " + name + " does not exist.");

        if (!parameter->IsWritable()) {
            std::cout << name << " is not writable at the time..." << std::endl;
            return;
        }

        // Use the provided setter function to set the attribute value
        (this->*setter)(parameter, value);
    }

    void setDeviceAttributeBool(std::string name, bool value) {
        setDeviceAttribute<PvGenBoolean, bool>(name, value, &PhotonFocusCamera::setBooleanAttribute);
    }

    void setDeviceAttributeLong(std::string name, long value) {
        setDeviceAttribute<PvGenInteger, long>(name, value, &PhotonFocusCamera::setIntegerAttribute);
    }

    void setDeviceAttributeString(std::string name, std::string value) {
        setDeviceAttribute<PvGenEnum, std::string>(name, value, &PhotonFocusCamera::setStringAttribute);
    }

    void setDeviceAttributeDouble(std::string name, double value) {
        setDeviceAttribute<PvGenFloat, double>(name, value, &PhotonFocusCamera::setDoubleAttribute);
    }

private:
    void setBooleanAttribute(PvGenBoolean* parameter, bool value) {
        CHECK_RESULT(parameter->SetValue(value));
    }

    void setIntegerAttribute(PvGenInteger* parameter, long value) {
        CHECK_RESULT(parameter->SetValue(value));
    }

    void setStringAttribute(PvGenEnum* parameter, std::string value) {
        // Check if the value is in the range
        long entries;
        bool is_in = false;
        CHECK_RESULT(parameter->GetEntriesCount(entries));
        for (int i = 0; i < entries && !is_in; i++) {
            const PvGenEnumEntry* entry;
            CHECK_RESULT(parameter->GetEntryByIndex(i, &entry));
            PvString current_value;
            CHECK_RESULT(entry->GetName(current_value));
            if (value == std::string(current_value.GetAscii()))
                is_in = true;
        }

        if (!is_in) {
            return;
        }
        CHECK_RESULT(parameter->SetValue(PvString(value.c_str())));
    }

    void setDoubleAttribute(PvGenFloat* parameter, double value) {
        CHECK_RESULT(parameter->SetValue(value));
    }

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