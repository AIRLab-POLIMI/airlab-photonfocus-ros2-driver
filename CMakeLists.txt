cmake_minimum_required(VERSION 3.5)
project(airlab-photonfocus-ros2-wrapper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# -------------------------------------------------------------------------------------------
# EBUS SDK
#set(EBUS_INCLUDE_DIRS "/opt/jai/ebus_sdk/linux-aarch64-arm/include")
#set(EBUS_LIBRARIES PvSystem PvBase PvDevice PvPersistence PvGenICam PvStream PvTransmitter PvVirtualDevice PvAppUtils PvSerial PvGUI PvBuffer)
#set(EBUS_LIB_DIRS "/opt/jai/ebus_sdk/linux-aarch64-arm/lib")
#link_directories(${EBUS_LIB_DIRS})

#set(EBUS_SDK_PATH "/opt/jai/ebus_sdk/linux-aarch64-arm") # for Jetson, arm-based architecture
#set(EBUS_INCLUDE_DIRS ${EBUS_SDK_PATH}/include)
#set(EBUS_LIBRARIES ${EBUS_SDK_PATH}/lib)
#include_directories(${EBUS_INCLUDE_DIRS})
#link_directories(${EBUS_LIBRARIES})

#add_executable(camera_exe src/camera.cpp)
#target_link_libraries(camera_exe PUBLIC ${EBUS_INCLUDE_DIRS})

# Add the path to the eBUS SDK API headers
#include_directories(/opt/jai/ebus_sdk/linux-aarch64-arm/include)

# Create the executable
#add_executable(camera src/camera.cpp)

# Link the eBUS SDK API library
#find_library(EBUS_LIB NAMES ebus PATHS /opt/jai/ebus_sdk/linux-aarch64-arm/lib)
#target_link_libraries(camera ${EBUS_LIB})

include(cmake/Findebus-sdk.cmake)
add_executable(camera src/camera.cpp)
target_link_libraries(camera ${ebus-sdk_LIBRARIES})
target_include_directories(camera PRIVATE ${ebus-sdk_INCLUDE_DIR})

# -------------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
