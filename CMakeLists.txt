# =====================================================================================================================
# File: photonfocus_camera.yaml
# Author: Mirko Usuelli (Ph.D. Candidate, Politecnico di Milano @ AIRLab)
# Email: mirko.usuell@polimi.it
# Description: This file contains the CMakelists.txt for the photonfocus_camera package.
# ---------------------------------------------------------------------------------------------------------------------
# Created on: 05/02/2024
# Last Modified: 12/02/2024
# =====================================================================================================================
cmake_minimum_required(VERSION 3.5)
project(airlab-photonfocus-ros2-driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings
add_compile_options(-Wall -Wextra -Wpedantic -Wno-terminate -Wunused-parameter -W)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_srvs REQUIRED)

# External cmake
include(cmake/FindEbus-sdk.cmake)

# Library
add_library(
  airlab_photonfocus include/photonfocus_camera.hpp src/photonfocus_camera.cpp
)

target_link_libraries(airlab_photonfocus
  ${EBUS_SDK_LIBRARIES} 
  ${OpenCV_LIBS}
  ${CMAKE_THREAD_LIBS_INIT}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${yaml_cpp_vendor_LIBRARIES}
)

target_include_directories(airlab_photonfocus PUBLIC
  include
  ${EBUS_SDK_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${yaml_cpp_vendor_INCLUDE_DIRS}
)

# Test ROS 2
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS 2
ament_target_dependencies(airlab_photonfocus
  rclcpp sensor_msgs yaml_cpp_vendor std_srvs
)

# Add executable
add_executable(photonfocus_camera_node src/photonfocus_camera_node.cpp)
target_link_libraries(photonfocus_camera_node airlab_photonfocus)

add_executable(photonfocus_prisma_node src/photonfocus_prisma_node.cpp)
target_link_libraries(photonfocus_prisma_node airlab_photonfocus)

# Install
install(TARGETS
    photonfocus_camera_node
    photonfocus_prisma_node
    DESTINATION lib/${PROJECT_NAME}
)

# Launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ROS 2
ament_package()
