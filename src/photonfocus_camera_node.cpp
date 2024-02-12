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
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

// C++ libraries
#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

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
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;  // ROS 2 publisher
        std::string frame_id_;  // ROS 2 frame ID
        std::string topic_;  // ROS 2 topic
        std::string ip_address_;  // PhotonFocus camera IP address
        std::string config_file_;  // YAML file path

    public:
        /**
         * @brief Constructor
         * @details The constructor initializes the ROS 2 node and the PhotonFocus camera.
         */
        PhotonFocusDriver() : Node("airlab_photonfocus") {
            // Declare parameters
            this->declare_parameter<std::string>("topic", "/vis_image_raw");
            this->declare_parameter<std::string>("frame_id", "vis_camera_link");
            this->declare_parameter<std::string>("ip_address", "10.79.2.78");
            this->declare_parameter<std::string>("config_file_path",
                "/home/airlab/ros2_ws/src/airlab-photonfocus-ros2-wrapper/config/default_camera.yaml");

            // Get parameter values
            topic_ = this->get_parameter("topic").as_string();
            frame_id_ = this->get_parameter("frame_id").as_string();
            ip_address_ = this->get_parameter("ip_address").as_string();
            config_file_ = this->get_parameter("config_file_path").as_string();

            // Use the parameter values as needed
            RCLCPP_INFO(this->get_logger(), "Topic: %s", topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
            RCLCPP_INFO(this->get_logger(), "IP Address: %s", ip_address_.c_str());
            RCLCPP_INFO(this->get_logger(), "Config File Path: %s", config_file_.c_str());

            // Publisher
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, 10);

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
                long width = cameraNode["width"].as<int>();
                long height = cameraNode["height"].as<int>();
                long offset_x = cameraNode["offset_x"].as<int>();
                long offset_y = cameraNode["offset_y"].as<int>();
                double exposure_time = cameraNode["exposure_time"].as<double>();
                bool acquisition_frame_rate_enable = cameraNode["acquisition_frame_rate_enable"].as<bool>();
                double acquisition_frame_rate = cameraNode["acquisition_frame_rate"].as<double>();

                // Set device attributes
                camera_->setDeviceAttributeLong("Width", width);
                camera_->setDeviceAttributeLong("Height", height);
                camera_->setDeviceAttributeLong("OffsetX", offset_x);
                camera_->setDeviceAttributeLong("OffsetY", offset_y);
                camera_->setDeviceAttributeDouble("ExposureTime", exposure_time);
                camera_->setDeviceAttributeBool("AcquisitionFrameRateEnable", acquisition_frame_rate_enable);
                if (acquisition_frame_rate_enable)
                    camera_->setDeviceAttributeDouble("AcquisitionFrameRate", acquisition_frame_rate);

                RCLCPP_INFO(this->get_logger(), "===== PhotonFocus Camera STARTED =====");
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
            RCLCPP_INFO(this->get_logger(), "===== PhotonFocus Camera STOPPED =====");
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
            publisher_->publish(image_);
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
