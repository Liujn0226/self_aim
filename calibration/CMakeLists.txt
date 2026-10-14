cmake_minimum_required(VERSION 3.16)

# ============================================================================
# 校内赛标定程序
#
#   calib_capture : 采集图片 + 云台姿态（USB CDC，替代原 capture.cpp）
#   calib_camera  : 内参标定（张正友法）
#   calib_handeye : 外参标定（手眼标定，输入是 yaw/pitch/roll 而非四元数）
#
# 原仓库的 calibration/*.cpp 依赖 io::CBoard（SocketCAN）与本赛制不兼容，
# 因此这里使用 calibration/ 下的新版实现，不改动原文件。
# ============================================================================

add_executable(calib_capture calib_capture.cpp)
target_link_libraries(calib_capture
    ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io contest_gimbal)

add_executable(calib_camera calibrate_camera.cpp)
target_link_libraries(calib_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)

# calib_handeye 本身不需要串口，但 calibrate_handeye.cpp include 了
# io/gimbal/gimbal.hpp 来取 io::Attitude，而 gimbal.hpp 里有
# #include "serial/serial.h"（因为 Gimbal 类持有 serial::Serial 成员）。
# 所以这个目标必须带上提供 serial include 路径的 contest_gimbal。
#
# 这是权宜之计：真正干净的做法是把协议定义（Attitude / GimbalState /
# 各种 *_s 结构体与常量）抽到一个不依赖 serial 的独立头文件里，
# 让纯数据的使用者不必拖进串口库。等有空再做。
add_executable(calib_handeye calibrate_handeye.cpp)
target_link_libraries(calib_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools contest_gimbal)
