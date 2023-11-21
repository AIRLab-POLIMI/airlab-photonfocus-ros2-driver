/* =====================================================================================================================
 * File: photonfocus_prisma_node.cpp
 * Author: Mirko Usuelli (Ph.D. Candidate, Politecnico di Milano @ AIRLab)
 * Email: mirko.usuell@polimi.it
 * Description: This file contains the ROS 2 implementation for the PhotonFocus camera prisma.
 * ---------------------------------------------------------------------------------------------------------------------
 * Created on: 27/02/2024
 * Last Modified: 27/02/2024
 * =====================================================================================================================
 */
#ifndef PHOTONFOCUS_PRISMA_NODE_HPP
#define PHOTONFOCUS_PRISMA_NODE_HPP

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


// AIRLab namespace
namespace AIRLab {
    /**
     * @brief PhotonFocusPrisma class
     * This class is a ROS 2 node that splits spectral bands of VIS and NIR PhotonFocus cameras.
     */
    class PhotonFocusPrisma : public rclcpp::Node {
    public:
        // total image size
        const int WIDTH = 2048;
        const int HEIGHT = 1088;

        // mosaicing pattern NxN
        const int VIS_MOSAIC_SIZE = 4;
        const int NIR_MOSAIC_SIZE = 5;

        // sub-images size
        const int VIS_W = WIDTH / VIS_MOSAIC_SIZE;
        const int VIS_H = HEIGHT / VIS_MOSAIC_SIZE;
        const int NIR_W = WIDTH / NIR_MOSAIC_SIZE;
        const int NIR_H = HEIGHT / NIR_MOSAIC_SIZE;

        // limits
        const int VIS_W_LIMIT = WIDTH - VIS_MOSAIC_SIZE;
        const int VIS_H_LIMIT = HEIGHT - VIS_MOSAIC_SIZE;
        const int NIR_W_LIMIT = WIDTH - NIR_MOSAIC_SIZE;
        const int NIR_H_LIMIT = HEIGHT - NIR_MOSAIC_SIZE;

        // spectral bands in nm
        const int VIS_BANDS_ = [ 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 610, 620 ];
        const int NIR_BANDS_ = [ 600, 615, 630, 645, 660, 675, 700, 715, 730, 745, 760, 775, 790, 805, 820, 835, 850, 875, 890, 900, 915, 930, 945, 960, 975 ];

    private:
        // topics
        std::string vis_topic_;
        std::string nir_topic_;

        // frame ids
        std::string vis_frame_id_;
        std::string nir_frame_id_;

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vis_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr nir_sub_;

        // publishers
        std::map<int, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> vis_pubs_;
        std::map<int, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> nir_pubs_;

        // images
        std::map<int, cv::Mat>::SharedPtr> vis_imgs_;
        std::map<int, cv::Mat>::SharedPtr> nir_imgs_;

    public:
        /**
         * @brief Constructor
         * @details The constructor initializes the ROS 2 node and the PhotonFocus camera.
         */
        PhotonFocusPrisma() : Node("airlab_photonfocus_prisma") {
            // params
            this->declare_parameter<std::string>("vis_topic", "/vis_image_raw");
            this->declare_parameter<std::string>("vis_frame_id", "vis_camera_link");
            this->declare_parameter<std::string>("nir_topic", "/nir_image_raw");
            this->declare_parameter<std::string>("nir_frame_id", "nir_camera_link");

            // subs
            vis_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                this->get_parameter("vis_topic").as_string(),  // The topic name
                10,  // The queue size
                std::bind(&PhotonFocusPrisma::vis_callback, this, std::placeholders::_1)  // The callback function
            );
            nir_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                this->get_parameter("nir_topic").as_string(),  // The topic name
                10,  // The queue size
                std::bind(&PhotonFocusPrisma::nir_callback, this, std::placeholders::_1)  // The callback function
            );

            // pubs and imgs
            for (const auto & band : VIS_BANDS_) {
                vis_pubs_[band] = this->create_publisher<sensor_msgs::msg::Image>("/vis_" + std::to_string(band) + "_nm", 10);
                vis_imgs_[band] = cv::Mat::zeros(VIS_H, VIS_W, CV_8UC1);
            }
            for (const auto & band : NIR_BANDS_) {
                nir_pubs_[band] = this->create_publisher<sensor_msgs::msg::Image>("/nir_" + std::to_string(band) + "_nm", 10);
                nir_imgs_[band] = cv::Mat::zeros(NIR_H, NIR_W, CV_8UC1);
            }
        }

        /**
         * @brief Destructor
         * @details The destructor stops the prisma.
         */
        ~PhotonFocusPrisma() = default;

    private:
        /**
         * @brief Publish VIS images
         * @details This function subscribes to the VIS mosaic image and decompose it in their bands.
         * @param img Image to be decomposed.
         */
        void vis_callback(const sensor_msgs::msg::Image::SharedPtr& msg) {
            // Convert the sensor_msgs::Image to cv::Mat
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            cv::Mat& img = cv_ptr->image;

            // de-mosaicing
            for (int r = 0; r <= VIS_H_LIMIT; r += VIS_MOSAIC_SIZE) {
                for (int c = 0; c <= VIS_W_LIMIT; c += VIS_MOSAIC_SIZE) {
                    int i = 0, j = 0;
                    for (auto& [band, vis_img] : vis_imgs_) {
                        vis_img.at<uchar>(r / VIS_MOSAIC_SIZE, c / VIS_MOSAIC_SIZE) = img.at<uchar>(r + ((i++) / VIS_MOSAIC_SIZE), c + ((j++) % VIS_MOSAIC_SIZE));
                    }
                }
            }

            // preparing ROS format
            sensor_msgs::msg::Image image_;
            cv_bridge::CvImage cv_image;
            cv_image.encoding = "mono8";
            cv_image.header.stamp = this->now();
            image_.header.frame_id = vis_frame_id_;

            // publishing each band
            for (auto& [band, vis_img] : vis_imgs_) {
                cv_image.image = vis_img;
                image_ = *cv_image.toImageMsg();
                vis_pubs_[band]->publish(image_);
            }
        }

        /**
         * @brief Publish NIR images
         * @details This function subscribes to the NIR mosaic image and decompose it in their bands
         * @param img Image to be decomposed.
         */
        void nir_callback(const sensor_msgs::msg::Image::SharedPtr& msg) {
            // Convert the sensor_msgs::Image to cv::Mat
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            cv::Mat& img = cv_ptr->image;

            // de-mosaicing
            for (int r = 0; r <= NIR_H_LIMIT; r += NIR_MOSAIC_SIZE) {
                for (int c = 0; c <= NIR_W_LIMIT; c += NIR_MOSAIC_SIZE) {
                    int i = 0, j = 0;
                    for (auto& [band, nir_img] : nir_imgs_) {
                        nir_img.at<uchar>(r / NIR_MOSAIC_SIZE, c / NIR_MOSAIC_SIZE) = img.at<uchar>(r + ((i++) / NIR_MOSAIC_SIZE), c + ((j++) % VIS_MOSAIC_SIZE));
                    }
                }
            }

            // preparing ROS format
            sensor_msgs::msg::Image image_;
            cv_bridge::CvImage cv_image;
            cv_image.encoding = "mono8";
            cv_image.header.stamp = this->now();
            image_.header.frame_id = nir_frame_id_;

            // publishing each band
            for (auto& [band, nir_img] : nir_imgs_) {
                cv_image.image = nir_img;
                image_ = *cv_image.toImageMsg();
                nir_pubs_[band]->publish(image_);
            }
        }
    };
}
#endif // PHOTONFOCUS_PRISMA_NODE_HPP


/**
 * Main function, ROS 2 single thread standard.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AIRLab::PhotonFocusPrisma>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
