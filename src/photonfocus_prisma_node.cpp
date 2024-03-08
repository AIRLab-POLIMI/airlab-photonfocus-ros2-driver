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

        // nir and vis dimensions
        const int VIS_WIDTH = 2048;
        const int VIS_HEIGHT = 1088;
        const int NIR_WIDTH = 2045;
        const int NIR_HEIGHT = 1085;

        // nir and vis offsets
        const int VIS_OFFSET_X = 0;
        const int VIS_OFFSET_Y = 0;
        const int NIR_OFFSET_X = 3;
        const int NIR_OFFSET_Y = 3;

        // mosaicing pattern NxN
        const int VIS_MOSAIC_SIZE = 4;
        const int NIR_MOSAIC_SIZE = 5;

        const int VIS_BANDS = VIS_MOSAIC_SIZE * VIS_MOSAIC_SIZE;
        const int NIR_BANDS = NIR_MOSAIC_SIZE * NIR_MOSAIC_SIZE;

        // sub-images size
        const int VIS_W = 512;  // 512 x 4 = 2048
        const int VIS_H = 272;  // 271 x 4 = 1088
        const int NIR_W = 409;  // 409 x 5 = 2045
        const int NIR_H = 217;  // 217 x 5 = 1085

        // spectral bands in nm
        const std::vector<int> VIS_BANDS_ = { 
            465, 546, 586, 630,
            474, 534, 578, 624,
            485, 522, 562, 608,
            496, 510, 548, 600
        };
        const std::vector<int> NIR_BANDS_ = { 
            915, 930, 945, 960, 975,
            835, 850, 875, 890, 900,
            760, 775, 790, 805, 820,
            675, 700, 715, 730, 745,
            600, 615, 630, 645, 660
        };
    
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

        // mosaic view
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vis_mosaic_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr nir_mosaic_pub_;

        // images
        std::map<int, cv::Mat> vis_imgs_;
        std::map<int, cv::Mat> nir_imgs_;

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

            vis_mosaic_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vis_mosaic", 10);
            nir_mosaic_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/nir_mosaic", 10);
        }

    private:
        /**
         * @brief Publish VIS images
         * @details This function subscribes to the VIS mosaic image and decompose it in their bands.
         * @param img Image to be decomposed.
         */
        void vis_callback(sensor_msgs::msg::Image::SharedPtr msg) {
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
            int new_c = 0;
            int new_r = 0;
            for (int r = VIS_OFFSET_Y; r < VIS_HEIGHT + VIS_OFFSET_Y; r += VIS_MOSAIC_SIZE) {
                new_c = 0;
                for (int c = VIS_OFFSET_X; c < VIS_WIDTH + VIS_OFFSET_X; c += VIS_MOSAIC_SIZE) {
                    int i = 0, j = 0;
                    for (int band : VIS_BANDS_) {
                        // decomposition
                        vis_imgs_[band].at<uchar>(new_r, new_c) = img.at<uchar>(r + i, c + j);

                        // pattern update
                        j = (j + 1) % VIS_MOSAIC_SIZE;
                        if (j == 0)
                            i++;
                    }
                    new_c++;
                }
                new_r++;
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

            // Create the grid image
            cv::Mat gridImage(VIS_H * VIS_MOSAIC_SIZE, VIS_W * VIS_MOSAIC_SIZE, CV_8UC1);

            // Populate the grid
            for (int row = 0; row < VIS_MOSAIC_SIZE; ++row) {
                for (int col = 0; col < VIS_MOSAIC_SIZE; ++col) {
                    int imageIndex = row * VIS_MOSAIC_SIZE + col;
                    if (imageIndex < VIS_BANDS) {
                        cv::Rect roi(col * VIS_W, row * VIS_H, VIS_W, VIS_H);
                        vis_imgs_[VIS_BANDS_[imageIndex]].copyTo(gridImage(roi));

                        // Add text to the ROI
                        std::string text = "Band: " + std::to_string(VIS_BANDS_[imageIndex]) + " nm";
                        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                        double fontScale = 1;
                        int thickness = 2;
                        cv::Point textOrg(roi.x, roi.y + VIS_H - 10);
                        cv::putText(gridImage, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
                    }
                }
            }

            cv_image.image = gridImage;
            image_ = *cv_image.toImageMsg();
            vis_mosaic_pub_->publish(image_);
        }

        /**
         * @brief Publish NIR images
         * @details This function subscribes to the NIR mosaic image and decompose it in their bands
         * @param img Image to be decomposed.
         */
        void nir_callback(sensor_msgs::msg::Image::SharedPtr msg) {
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
            int new_c = 0;
            int new_r = 0;
            for (int r = NIR_OFFSET_Y; r < NIR_HEIGHT + NIR_OFFSET_Y; r += NIR_MOSAIC_SIZE) {
                new_c = 0;
                for (int c = NIR_OFFSET_X; c < NIR_WIDTH + NIR_OFFSET_X; c += NIR_MOSAIC_SIZE) {
                    int i = 0, j = 0;
                    for (int band : NIR_BANDS_) {
                        // decomposition
                        nir_imgs_[band].at<uchar>(new_r, new_c) = img.at<uchar>(r + i, c + j);

                        // pattern update
                        j = (j + 1) % NIR_MOSAIC_SIZE;
                        if (j == 0)
                            i++;
                    }
                    new_c++;
                }
                new_r++;
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

            // Create the grid image
            cv::Mat gridImage(NIR_H * NIR_MOSAIC_SIZE, NIR_W * NIR_MOSAIC_SIZE, CV_8UC1);

            // Populate the grid
            for (int row = 0; row < NIR_MOSAIC_SIZE; ++row) {
                for (int col = 0; col < NIR_MOSAIC_SIZE; ++col) {
                    int imageIndex = row * NIR_MOSAIC_SIZE + col;
                    if (imageIndex < NIR_BANDS) {
                        cv::Rect roi(col * NIR_W, row * NIR_H, NIR_W, NIR_H);
                        nir_imgs_[NIR_BANDS_[imageIndex]].copyTo(gridImage(roi));

                        // Add text to the ROI
                        std::string text = "Band: " + std::to_string(NIR_BANDS_[imageIndex]) + " nm";
                        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                        double fontScale = 1;
                        int thickness = 2;
                        cv::Point textOrg(roi.x, roi.y + NIR_H / 2 - 10);
                        cv::putText(gridImage, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
                    }
                }
            }

            cv_image.image = gridImage;
            image_ = *cv_image.toImageMsg();
            nir_mosaic_pub_->publish(image_);
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
