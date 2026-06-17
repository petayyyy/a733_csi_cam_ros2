cmake_minimum_required(VERSION 3.10)
project(a733_csi_cam_ros2 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    add_compile_options($<$<CONFIG:Release>:-O2 -pipe -march=armv8-a -mtune=cortex-a53>)
else()
    add_compile_options($<$<CONFIG:Release>:-O2 -pipe>)
endif()

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# ── AWIspApi (Allwinner ISP 3A) — optional ───────────────────────────────────
find_library(AWISP_LIB NAMES AWIspApi libAWIspApi
    PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu)

find_path(AWISP_INCLUDE NAMES AWIspApi.h
    PATHS /usr/include /usr/local/include /usr/include/camera)

if(AWISP_LIB AND AWISP_INCLUDE)
    message(STATUS "AWIspApi found: ${AWISP_LIB}")
    add_compile_definitions(USE_AWI_SP)
else()
    message(WARNING "AWIspApi NOT found — ISP 3A disabled (auto-exposure/AWB unavailable)")
endif()

# ── Node executable ───────────────────────────────────────────────────────────
add_executable(imx219_camera_node src/imx219_camera_node.cpp)

target_include_directories(imx219_camera_node PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    $<$<BOOL:${AWISP_INCLUDE}>:${AWISP_INCLUDE}>
)

ament_target_dependencies(imx219_camera_node
    rclcpp
    std_msgs
    sensor_msgs
    camera_info_manager
    image_transport
    cv_bridge
)

target_link_libraries(imx219_camera_node
    ${OpenCV_LIBS}
    pthread
    $<$<BOOL:${AWISP_LIB}>:${AWISP_LIB}>
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS imx219_camera_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
