cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# OpenVINO 版本与安装路径：如与本机安装不一致，修改下面两处
#   * 本文件
#   * tasks/auto_aim/CMakeLists.txt
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/omniperception)
add_subdirectory(calibration)

# ============================================================================
# 校内赛自瞄赛道
#
# contest              : 主程序（相机 + 识别 + 跟踪 + 瞄准 + 新协议通信）
# contest_gimbal_test  : 只验证通信协议，不依赖相机与神经网络
#
# 链接约定（一个目标只链接其中一个 gimbal 提供者，避免归档重复）：
#   contest             -> io              （io 里已含 gimbal 与 camera）
#   calib_capture       -> io + contest_gimbal
#   contest_gimbal_test -> contest_gimbal  （不含相机 SDK，无需相机驱动）
# ============================================================================
add_executable(contest src/contest.cpp)
target_link_libraries(contest
    ${OpenCV_LIBS} fmt::fmt yaml-cpp
    auto_aim omniperception tools io)

add_executable(contest_gimbal_test tests/contest_gimbal_test.cpp)
target_link_libraries(contest_gimbal_test
    ${OpenCV_LIBS} fmt::fmt yaml-cpp
    tools contest_gimbal)

# ============================================================================
# 说明：原仓库的以下目标依赖 RoboMaster 队内硬件与 ROS2，本赛制不使用，
#       需要时再按原样恢复：
#         standard / mt_standard / standard_mpc / auto_aim_debug_mpc
#         mt_auto_aim_debug / uav / uav_debug / sentry* / auto_buff* / publish*
#       tasks/auto_buff（打符）需要 Ceres，且本赛制不需要打符，因此
#       add_subdirectory(tasks/auto_buff) 未加入。
#       omniperception 仍参与编译：auto_aim/tracker.hpp 依赖它的 DetectionResult。
#
# 调试识别时常用的原仓库测试程序（当前未编译，需要时按下面几行加回来）：
#   add_executable(camera_detect_test tests/camera_detect_test.cpp)
#   target_link_libraries(camera_detect_test
#       ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
#
#   add_executable(detector_video_test tests/detector_video_test.cpp)
#   target_link_libraries(detector_video_test
#       ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
#
#   add_executable(camera_test tests/camera_test.cpp)
#   target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
# ============================================================================
