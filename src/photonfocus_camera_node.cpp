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

// AIRLab namespace
namespace AIRLab {
    /**
     * @brief PhotonFocusDriver class
     * This class is a ROS 2 node that interfaces with the PhotonFocus camera.
     */
    class PhotonFocusDriver : public rclcpp::Node {
    private:
        std::unique_ptr<AIRLab::PhotonFocusCamera> camera_;  // PhotonFocus camera object
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;  // raw image publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr hist_publisher_;  // histogram publisher
        std::string frame_id_;  // ROS 2 frame ID
        std::string image_topic_;  // ROS 2 topic
        std::string hist_topic_;  // ROS 2 topic
        std::string ip_address_;  // PhotonFocus camera IP address
        std::string config_file_;  // YAML file path
        std::string tag_;  // camera type tag
        cv::Mat hist;  // last histogram
        rclcpp::Time last_time_;  // last time
        bool histagram_enabled_;  // whether histogram is enabled
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr exposure_service_;
        std::condition_variable cv;
        std::mutex mtx;
        int width;
        int height;

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
            hist_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(hist_topic_, 1);

            // Service
            exposure_service_ = this->create_service<std_srvs::srv::Empty>("auto_exposure", 
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
            // Convert image to ROS 2 message
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

        void autoExposure(const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
                    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
            RCLCPP_INFO(this->get_logger(), "Auto Exposure Service called");

            std::map<float, float> exposure_correlation;
            std::map<float, float> exposure_chi_square;

            float min_correlation = std::numeric_limits<float>::max();
            float max_correlation = std::numeric_limits<float>::min();
            float min_chi_square = std::numeric_limits<float>::max();
            float max_chi_square = std::numeric_limits<float>::min();

            cv::Mat reference_hist = cv::Mat::zeros(256, 1, CV_32F);
            reference_hist.at<float>(8, 0) = 1.0;
            reference_hist.at<float>(56, 0) = 1.0;
            reference_hist.at<float>(69, 0) = 1.0;
            reference_hist.at<float>(220, 0) = 1.0;

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

            double best_exposure_ = pq.top().second;
            camera_->setDeviceAttributeDouble("ExposureTime", best_exposure_);
            RCLCPP_INFO(this->get_logger(), "================================================");
            RCLCPP_INFO(this->get_logger(), "BEST EXPOSURE TIME: %f", best_exposure_);
            RCLCPP_INFO(this->get_logger(), "================================================");
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
