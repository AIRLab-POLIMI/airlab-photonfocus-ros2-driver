/* =====================================================================================================================
 * File: photonfocus_camera.cpp
 * Author: Mirko Usuelli (Ph.D. Candidate, Politecnico di Milano @ AIRLab)
 * Email: mirko.usuell@polimi.it
 * Description: This file contains the ROS 2 implementation for the PhotonFocus camera driver.
 * ---------------------------------------------------------------------------------------------------------------------
 * Created on: 05/02/2024
 * Last Modified: 12/02/2024
 * =====================================================================================================================
 */
#ifndef PHOTONFOCUS_CAMERA_NODE_HPP
#define PHOTONFOCUS_CAMERA_NODE_HPP

// ROS 2 libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

// C++ libraries
#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <condition_variable>
#include <mutex>

// PhotonFocus header
#include "photonfocus_camera.hpp"
#include "airlab_photonfocus_ros2_driver/srv/auto_exposure.hpp"

// Macro function to transform light% to grayscal 0-255 by rounded approximation as integer
#define LIGHT_2_GRAYSCALE(l) ((int)(round(l * 255)))

// AIRLab namespace
namespace AIRLab {

    /**
     * In the context of agriculture, diffuse reflectivity is often more beneficial. 
     * This is because plants use diffuse light more efficiently than direct light. 
     * Diffuse light penetrates deeper into the canopy and creates a more homogeneous light profile, 
     * which can lead to a more efficient use of light1.
     * Moreover, diffuse reflectance spectroscopy has been proposed as a promising method for 
     * quantifying soil organic carbon © and nitrogen (N) stocks, which are crucial for developing 
     * land management strategies to mitigate climate change and sustain food production
     */
    typedef struct sc {
        float black;
        float dark_gray;
        float light_gray;
        float white;
    } spectral_calib_t;

    /**
     * Bands of interest:
     * - VIS: 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 610, 620
     * - NIR: 600, 615, 630, 645, 660, 675, 700, 715, 730, 745, 760, 775, 790, 805, 820, 835, 850, 865, 880, 895, 910, 925, 940, 955, 960, 975
     * Target calibration values are taken at: 
     * - https://www.mapir.camera/en-gb/collections/calibration-target-v2/products/diffuse-reflectance-standard-calibration-target-package-v2
     */
    const std::map<int, spectral_calib_t> MAP_CALIB = {  // sorted as black, dark_gray, light_gray, white
        {0, {0.0200, 0.21000, 0.27000, 0.83000 }},  // default avg calibration target
        {470, { 0.1999, 0.18442, 0.25163, 0.81391 }},
        {480, { 0.1992, 0.18530, 0.25377, 0.82363 }},
        {490, { 0.1993, 0.18600, 0.25534, 0.83391 }},
        {500, { 0.1987, 0.18700, 0.25736, 0.84161 }},
        {510, { 0.1978, 0.18816, 0.25851, 0.84865 }},
        {520, { 0.1979, 0.18948, 0.26031, 0.85430 }},
        {530, { 0.1974, 0.19081, 0.26080, 0.85843 }},
        {540, { 0.1974, 0.19189, 0.26197, 0.86233 }},
        {550, { 0.1973, 0.19304, 0.26258, 0.86459 }},
        {560, { 0.1963, 0.19376, 0.26304, 0.86643 }},
        {570, { 0.1963, 0.19440, 0.26329, 0.86782 }},
        {580, { 0.1950, 0.19533, 0.26366, 0.86816 }},
        {590, { 0.1947, 0.19588, 0.26381, 0.86937 }},
        {600, { 0.1943, 0.19587, 0.26395, 0.87003 }},
        {610, { 0.1941, 0.19433, 0.26370, 0.86938 }},
        {615, { 0.1941, 0.19524, 0.26349, 0.86969 }},
        {620, { 0.1946, 0.19649, 0.26353, 0.86963 }},
        {630, { 0.1945, 0.19822, 0.26312, 0.86980 }},
        {645, { 0.1936, 0.19837, 0.26316, 0.87156 }},
        {660, { 0.1934, 0.19827, 0.26232, 0.87205 }},
        {675, { 0.1937, 0.18130, 0.26074, 0.87212 }},
        {700, { 0.1949, 0.20486, 0.26201, 0.87256 }},
        {715, { 0.1951, 0.20995, 0.26304, 0.87116 }},
        {730, { 0.1956, 0.21288, 0.26274, 0.86999 }},
        {745, { 0.1961, 0.21560, 0.26175, 0.86955 }},
        {760, { 0.1979, 0.21709, 0.26318, 0.86760 }},
        {775, { 0.1978, 0.21921, 0.26735, 0.86759 }},
        {790, { 0.1971, 0.22037, 0.27151, 0.86457 }},
        {805, { 0.2095, 0.22090, 0.26962, 0.86360 }},
        {820, { 0.2338, 0.22641, 0.27786, 0.86652 }},
        {835, { 0.2257, 0.22866, 0.27645, 0.86266 }},
        {850, { 0.1938, 0.22934, 0.27620, 0.86198 }},
        {865, { 0.2241, 0.23219, 0.27746, 0.85682 }},
        {880, { 0.2087, 0.23294, 0.27832, 0.85245 }},
        {895, { 0.2077, 0.23354, 0.28017, 0.84744 }},
        {910, { 0.2076, 0.23537, 0.28119, 0.84452 }},
        {925, { 0.2013, 0.23697, 0.28181, 0.84577 }},
        {940, { 0.2033, 0.23897, 0.28311, 0.84844 }},
        {955, { 0.2044, 0.24063, 0.28458, 0.84975 }},
        {960, { 0.2081, 0.24159, 0.28531, 0.84931 }},
        {975, { 0.2031, 0.24252, 0.28587, 0.84728 }}
    };

    /**
     * @brief PhotonFocusDriver class
     * This class is a ROS 2 node that interfaces with the PhotonFocus camera.
     */
    class PhotonFocusDriver : public rclcpp::Node {
    private:
        std::unique_ptr<AIRLab::PhotonFocusCamera> camera_;                                                 // PhotonFocus camera object
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;                             // raw image publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr hist_publisher_;                              // histogram publisher
        std::string frame_id_;                                                                              // ROS 2 frame ID
        std::string image_topic_;                                                                           // ROS 2 topic
        std::string hist_topic_;                                                                            // ROS 2 topic
        std::string ip_address_;                                                                            // PhotonFocus camera IP address
        std::string config_file_;                                                                           // YAML file path
        std::string tag_;                                                                                   // camera type tag
        cv::Mat hist;                                                                                       // last histogram
        rclcpp::Time last_time_;                                                                            // last time
        bool histagram_enabled_;                                                                            // whether histogram is enabled
        rclcpp::Service<airlab_photonfocus_ros2_driver::srv::AutoExposure>::SharedPtr exposure_service_;   // ROS 2 service
        std::condition_variable cv;                                                                         // condition variable for service parallelism
        std::mutex mtx;                                                                                     // mutex for racing condition
        int width;                                                                                          // width of the image
        int height;                                                                                         // height of the image

    public:
        /**
         * @brief Constructor
         * @details The constructor initializes the ROS 2 node and the PhotonFocus camera.
         */
        PhotonFocusDriver() : Node("airlab_photonfocus") {
            // Declare parameters
            this->declare_parameter<std::string>("topic", "/vis");
            this->declare_parameter<std::string>("frame_id", "vis_camera_link");
            this->declare_parameter<std::string>("ip_address", "10.79.2.78");
            this->declare_parameter<std::string>("config_file_path",
                "/home/airlab/ros2_ws/src/airlab-photonfocus-ros2-driver/config/default_camera.yaml");

            // Get parameter values
            image_topic_ = this->get_parameter("topic").as_string() + "_image_raw";
            hist_topic_ = this->get_parameter("topic").as_string() + "_histogram";
            frame_id_ = this->get_parameter("frame_id").as_string();
            ip_address_ = this->get_parameter("ip_address").as_string();
            config_file_ = this->get_parameter("config_file_path").as_string();

            // Use the parameter values as needed
            RCLCPP_INFO(this->get_logger(), "Topic: %s", image_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
            RCLCPP_INFO(this->get_logger(), "IP Address: %s", ip_address_.c_str());
            RCLCPP_INFO(this->get_logger(), "Config File Path: %s", config_file_.c_str());

            // Publishers
            image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, 1);

            // Service
            exposure_service_ = this->create_service<airlab_photonfocus_ros2_driver::srv::AutoExposure>("auto_exposure", 
                std::bind(&PhotonFocusDriver::autoExposure, this, std::placeholders::_1, std::placeholders::_2));

            // Initialize camera
            camera_ = std::make_unique<AIRLab::PhotonFocusCamera>(ip_address_);
            camera_->start();
            camera_->callback = std::bind(&PhotonFocusDriver::publishImage, this, std::placeholders::_1);

            // Reading params from YAML file
            try {
                std::ifstream file(config_file_);
                if (!file.is_open()) {
                    throw std::runtime_error("YAML file not found");
                }
                YAML::Node yamlParams = YAML::Load(file);
                file.close();

                const YAML::Node& cameraNode = yamlParams["photonfocus_camera"];
                if (!cameraNode) {
                    throw std::runtime_error("photonfocus_camera node not found in YAML file");
                }

                // Define device attributes
                tag_ = cameraNode["type"].as<std::string>();
                width = cameraNode["width"].as<int>();
                height = cameraNode["height"].as<int>();
                long offset_x = cameraNode["offset_x"].as<int>();
                long offset_y = cameraNode["offset_y"].as<int>();
                double exposure_time = cameraNode["exposure_time"].as<double>();
                bool acquisition_frame_rate_enable = cameraNode["acquisition_frame_rate_enable"].as<bool>();
                double acquisition_frame_rate = cameraNode["acquisition_frame_rate"].as<double>();
                histagram_enabled_ = cameraNode["histogram"].as<bool>();

                // Set device attributes
                camera_->setDeviceAttributeLong("Width", width);
                camera_->setDeviceAttributeLong("Height", height);
                camera_->setDeviceAttributeLong("OffsetX", offset_x);
                camera_->setDeviceAttributeLong("OffsetY", offset_y);
                camera_->setDeviceAttributeDouble("ExposureTime", exposure_time);
                camera_->setDeviceAttributeBool("AcquisitionFrameRateEnable", acquisition_frame_rate_enable);
                if (acquisition_frame_rate_enable)
                    camera_->setDeviceAttributeDouble("AcquisitionFrameRate", acquisition_frame_rate);

                RCLCPP_INFO(this->get_logger(), "===== PhotonFocus " + tag_ + " Camera STARTED =====");
            } catch (const YAML::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file: %s", e.what());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }

            // debugging publisher
            if (histagram_enabled_)
                hist_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(hist_topic_, 1);
        }

        /**
         * @brief Destructor
         * @details The destructor stops the camera and releases the resources.
         */
        ~PhotonFocusDriver() {
            camera_->stop();
            camera_.reset();
            RCLCPP_INFO(this->get_logger(), "===== PhotonFocus " + tag_ + " Camera STOPPED =====");
        }

    private:
        /**
         * @brief Publish image
         * @details This function publishes the image to the ROS 2 topic.
         * @param img Image to be published.
         */
        void publishImage(const cv::Mat& img) {  
            // Copy the data from the original image to the new one
            cv_bridge::CvImage cv_image;
            cv_image.encoding = "mono8";
            cv_image.image = img;
            cv_image.header.stamp = this->now();
            sensor_msgs::msg::Image image_ = *cv_image.toImageMsg();
            image_.header.frame_id = frame_id_;

            // Publish image
            image_publisher_->publish(image_);
            
            this->publishHistogram(img);

            cv.notify_one();
        }

        void publishHistogram(const cv::Mat& img) {
            // Compute the histogram
            int histSize = 256;
            float range[] = { 0, 256 };
            const float* histRange = { range };

            cv::calcHist(&img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
            last_time_ = this->now();

            // Publish image
            if (histagram_enabled_) {
                // Draw the histogram
                int hist_w = 512, hist_h = 400;
                int bin_w = cvRound((double)hist_w / histSize);

                cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

                // Normalize the histogram
                cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

                // Draw the histogram
                for (int i = 1; i < histSize; i++) {
                    cv::line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
                        cv::Point(bin_w * (i), hist_h - cvRound(hist.at<float>(i))),
                        cv::Scalar(255, 255, 255), 2, 8, 0);
                }
                cv::Mat grayImage;
                cv::cvtColor(histImage, grayImage, cv::COLOR_BGR2GRAY);

                // Convert image to ROS 2 message
                cv_bridge::CvImage cv_image;
                cv_image.encoding = "mono8";
                cv_image.image = grayImage;
                cv_image.header.stamp = this->now();
                sensor_msgs::msg::Image image_ = *cv_image.toImageMsg();
                image_.header.frame_id = frame_id_;

                hist_publisher_->publish(image_);
            }
        }

        void autoExposure(const std::shared_ptr<airlab_photonfocus_ros2_driver::srv::AutoExposure::Request> request, 
                    std::shared_ptr<airlab_photonfocus_ros2_driver::srv::AutoExposure::Response> response) {
            RCLCPP_INFO(this->get_logger(), "Auto Exposure Service called");

            if (request == nullptr) {
                response = nullptr;
                RCLCPP_ERROR(this->get_logger(), "Auto Exposure Service called with nullptr param!");
                return;
            }
            
            // Cast the int64 to an int
            int query = static_cast<int>(request->wavelength);

            std::map<float, float> exposure_correlation;
            std::map<float, float> exposure_chi_square;

            float min_correlation = std::numeric_limits<float>::max();
            float max_correlation = std::numeric_limits<float>::min();
            float min_chi_square = std::numeric_limits<float>::max();
            float max_chi_square = std::numeric_limits<float>::min();

            cv::Mat reference_hist = cv::Mat::zeros(256, 1, CV_32F);

            // check for the query index
            auto it = MAP_CALIB.find(query);
            if (it != MAP_CALIB.end()) {
                reference_hist.at<float>(LIGHT_2_GRAYSCALE(it->second.black), 0) = 1.0;
                reference_hist.at<float>(LIGHT_2_GRAYSCALE(it->second.dark_gray), 0) = 1.0;
                reference_hist.at<float>(LIGHT_2_GRAYSCALE(it->second.light_gray), 0) = 1.0;
                reference_hist.at<float>(LIGHT_2_GRAYSCALE(it->second.white), 0) = 1.0;
            } else {
                // Handle the case where the key is not found in the map
                reference_hist.at<float>(LIGHT_2_GRAYSCALE(MAP_CALIB.begin()->second.black), 0) = 1.0;
                reference_hist.at<float>(LIGHT_2_GRAYSCALE(MAP_CALIB.begin()->second.dark_gray), 0) = 1.0;
                reference_hist.at<float>(LIGHT_2_GRAYSCALE(MAP_CALIB.begin()->second.light_gray), 0) = 1.0;
                reference_hist.at<float>(LIGHT_2_GRAYSCALE(MAP_CALIB.begin()->second.white), 0) = 1.0;
            }

            for (double exposure = 13.0; exposure <= 66586.0; exposure += 1000.0) {
                camera_->setDeviceAttributeDouble("ExposureTime", exposure);

                auto right_now = this->now();
                std::unique_lock<std::mutex> lck(mtx);
                cv.wait(lck, [&]{ return last_time_ > right_now; });

                cv::Mat converted_hist;
                hist.convertTo(converted_hist, CV_32F);

                // Find maximum value in the histogram
                double minVal, maxVal;
                cv::minMaxLoc(converted_hist, &minVal, &maxVal);
                converted_hist = converted_hist / maxVal;

                float correlation = cv::compareHist(converted_hist, reference_hist, cv::HISTCMP_CORREL);
                float chi_square = cv::compareHist(converted_hist, reference_hist, cv::HISTCMP_CHISQR);

                exposure_correlation[exposure] = correlation;
                exposure_chi_square[exposure] = chi_square;

                min_correlation = std::min(min_correlation, correlation);
                max_correlation = std::max(max_correlation, correlation);
                min_chi_square = std::min(min_chi_square, chi_square);
                max_chi_square = std::max(max_chi_square, chi_square);

                RCLCPP_INFO(this->get_logger(), "Exposure: %f us - Correlation: %f - Chi-Square: %f", exposure, correlation, chi_square);

                // edge cases
                if (exposure == 13.0)
                    exposure = 0.0;
                else if (exposure == 60000.0)
                    exposure = 65586;
            }

            RCLCPP_INFO(this->get_logger(), "================================================");
            std::priority_queue<std::pair<float, float>, std::vector<std::pair<float, float>>, std::greater<>> pq;

            for (auto& [exposure, correlation] : exposure_correlation) {
                correlation = (correlation - min_correlation) / (max_correlation - min_correlation);
                exposure_chi_square[exposure] = (exposure_chi_square[exposure] - min_chi_square) / (max_chi_square - min_chi_square);

                float total = correlation + exposure_chi_square[exposure];
                pq.push({total, exposure});
                RCLCPP_INFO(this->get_logger(), "Exposure: %f us - Score: %f", exposure, total);
            }

            float best_exposure_ = pq.top().second;
            camera_->setDeviceAttributeDouble("ExposureTime", best_exposure_);
            RCLCPP_INFO(this->get_logger(), "================================================");
            RCLCPP_INFO(this->get_logger(), "BEST EXPOSURE TIME: %f", best_exposure_);
            RCLCPP_INFO(this->get_logger(), "================================================");
            response->exposure_time = best_exposure_;
        }
    };
}
#endif // PHOTONFOCUS_CAMERA_NODE_HPP


/**
 * Main function, ROS 2 single thread standard.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AIRLab::PhotonFocusDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
