#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "photonfocus_camera.hpp"

namespace AIRLab {
    class PhotonFocusDriver : public rclcpp::Node {
    private:
        //sensor_msgs::msg::Image image_;
        std::unique_ptr<AIRLab::PhotonFocusCamera> camera_;
        std::string frame_id_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    public:
        PhotonFocusDriver(const std::string& camera_name, const std::string& ip, const std::string& yaml) : 
            Node(camera_name), camera_(std::make_unique<AIRLab::PhotonFocusCamera>(ip)) {
            // Initialize camera
            camera_->start();
            camera_->callback = std::bind(&PhotonFocusDriver::publishImage, this, std::placeholders::_1);
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/hyperspectral", 10);

            this->declare_parameter<std::string>("frame_id", "photonfocus_camera_link");
            frame_id_ = this->get_parameter("frame_id").as_string();

            // Reading params from YAML file
            try {
                std::ifstream file(yaml);
                if (!file.is_open()) {
                    throw std::runtime_error("YAML file not found");
                }
                YAML::Node yamlParams = YAML::Load(file);
                file.close();

                const YAML::Node& cameraNode = yamlParams["photonfocus_camera"];
                if (!cameraNode) {
                    throw std::runtime_error("photonfocus_camera node not found in YAML file");
                }

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

            RCLCPP_INFO(this->get_logger(), "===== PhotonFocus Camera STARTED =====");
        }

        ~PhotonFocusDriver() {
            camera_->stop();
            camera_.reset();
            RCLCPP_INFO(this->get_logger(), "===== PhotonFocus Camera STOPPED =====");
        }

    private:
        void publishImage(const cv::Mat& img) {
            cv_bridge::CvImage cv_image;
            cv_image.encoding = "mono8";
            cv_image.image = img;
            cv_image.header.stamp = this->now();
            sensor_msgs::msg::Image image_ = *cv_image.toImageMsg();
            image_.header.frame_id = frame_id_;

            publisher_->publish(image_);
        }
    };
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AIRLab::PhotonFocusDriver>("airlab_photonfocus", "10.79.2.78", "/home/airlab/ros2_ws/src/airlab-photonfocus-ros2-wrapper/config/photonfocus_camera.yaml");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
