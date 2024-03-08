/* =====================================================================================================================
 * File: photonfocus_camera.hpp
 * Author: Mirko Usuelli (Ph.D. Candidate, Politecnico di Milano @ AIRLab)
 * Email: mirko.usuell@polimi.it
 * Description: This file contains the pure C++ header for the PhotonFocus camera.
 * ---------------------------------------------------------------------------------------------------------------------
 * Created on: 05/02/2024
 * Last Modified: 12/02/2024
 * =====================================================================================================================
 */
#ifndef PHOTONFOCUS_CAMERA_HPP
#define PHOTONFOCUS_CAMERA_HPP

// Include the standard C++ headers
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>

// Include the OpenCV headers
#include <opencv2/opencv.hpp>

// Include the PvAPI library, eBUS SDK
#include "PvDeviceGEV.h"
#include "PvStreamGEV.h"
#include "PvPipeline.h"

/**
 * This macro is used to check the result of a PvAPI function call and throw an exception if it fails.
 * It is used to avoid writing the same code over and over again.
 * The macro is used as follows:
 *  \code
 *      CHECK_RESULT(PvDevice::CreateAndConnect(device, PvString("
 *  \endcode
 * The macro will throw an exception if the result of the expression is not OK.
 * The exception will contain the file name and line number where the error occurred, as well as the error message.
 * The macro is defined as a do-while loop to avoid problems when used in if-else statements.
 * The macro is also defined as a multi-statement macro to avoid problems when used with if-else statements.
 */
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

// Number of buffers used for image acquisition
#define BUFFER_COUNT 16

// AIRLab namespace
namespace AIRLab {
    // Enumeration to indicate the state of the reconfiguration process
    enum {
      RECONFIGURE_CLOSE = 3,
      RECONFIGURE_STOP = 1,
      RECONFIGURE_RUNNING = 0,
    };

    /**
     * This class is a driver for the PhotonFocus camera. It uses the PvAPI library to communicate with the camera.
     * The class is responsible for opening and closing the camera, setting the camera parameters, and acquiring images.
     * The class also provides a callback mechanism to notify the user when a new image is available.
     */
    class PhotonFocusCamera {
        PvDevice* device;  // Pointer to the device object
        PvStream* stream;  // Pointer to the stream object
        PvPipeline* pipeline;  // Pointer to the pipeline object
        PvString camera_id;  // Camera ID (IP address)

        PvGenParameterArray* device_parameters; // Pointer to the device parameters
        PvGenParameterArray* stream_parameters;  // Pointer to the stream parameters

        std::shared_ptr<std::thread> image_thread;  // Thread used to acquire images

    private:
        std::atomic<bool> stop_thread { false };  // Flag to stop the image acquisition thread
        std::mutex mtx;  // Mutex to protect the image buffer

        /**
         * Open the camera
         */
        void open();

        /**
         * Close the camera
         */
        void close();

        /**
         * Start the image acquisition thread
         */
        void acquireImages();

        /**
         * Stop the image acquisition thread
         */
        void interruptThread();

        /**
         * Get access type
         * @return The access type
         */
        PvAccessType getAccessType();

    public:
        // Callback function to notify the user when a new image is available
        std::function<void(const cv::Mat& img)> callback;

        /**
         * Constructor
         * @param ip_address The IP address of the camera
         */
        PhotonFocusCamera(const std::string& ip_address);

        /**
         * Destructor
         */
        ~PhotonFocusCamera();

        /**
         * Open the camera
         */
        void start();

        /**
         * Close the camera
         */
        void stop();

        /**
         * Get device attribute
         * @param name The name of the attribute
         * @param min Pointer to the minimum value
         * @param max Pointer to the maximum value
         * @return The value of the attribute
         */
        template <typename ParamType, typename ValueType>
        ValueType getDeviceAttribute(std::string name, ValueType* min = nullptr, ValueType* max = nullptr);

        /**
         * Set device attribute
         * @param name The name of the attribute
         * @param value The value of the attribute
         * @param setter The setter function
         */
        template <typename ParamType, typename ValueType, typename SetterFunction>
        void setDeviceAttribute(std::string name, ValueType value, SetterFunction setter);

        /**
         * Set device attribute for boolean
         * @param name The name of the attribute
         * @param value The value of the attribute
         */
        void setDeviceAttributeBool(std::string name, bool value);

        /**
         * Set device attribute for long
         * @param name The name of the attribute
         * @param value The value of the attribute
         */
        void setDeviceAttributeLong(std::string name, long value);

        /**
         * Set device attribute for string
         * @param name The name of the attribute
         * @param value The value of the attribute
         */
        void setDeviceAttributeString(std::string name, std::string value);

        /**
         * Set device attribute for double
         * @param name The name of the attribute
         * @param value The value of the attribute
         */
        void setDeviceAttributeDouble(std::string name, double value);

    private:
        /**
         * Set device attribute for boolean
         * @param parameter The parameter
         * @param value The value of the attribute
         */
        void setBooleanAttribute(PvGenBoolean* parameter, bool value);

        /**
         * Set device attribute for long
         * @param parameter The parameter
         * @param value The value of the attribute
         */
        void setIntegerAttribute(PvGenInteger* parameter, long value);

        /**
         * Set device attribute for string
         * @param parameter The parameter
         * @param value The value of the attribute
         */
        void setStringAttribute(PvGenEnum* parameter, std::string value);

        /**
         * Set device attribute for double
         * @param parameter The parameter
         * @param value The value of the attribute
         */
        void setDoubleAttribute(PvGenFloat* parameter, double value);
    };

}
#endif // PHOTONFOCUS_CAMERA_HPP
