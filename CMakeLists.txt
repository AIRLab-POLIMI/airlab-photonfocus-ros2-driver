cmake_minimum_required(VERSION 3.5)
project(airlab-photonfocus-ros2-wrapper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# -------------------------------------------------------------------------------------------
include(cmake/FindEbus-sdk.cmake)

add_library(
  airlab_photonfocus src/photonfocus_camera.cpp
)

target_link_libraries(airlab_photonfocus
  ${EBUS_SDK_LIBRARIES} 
  ${OpenCV_LIBS}
  ${CMAKE_THREAD_LIBS_INIT}
)

target_include_directories(airlab_photonfocus PUBLIC
  include
  ${EBUS_SDK_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# create executable
add_executable(camera_test test/camera_test.cpp)

# link libraries to the executable
target_link_libraries(camera_test airlab_photonfocus)

# -------------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
