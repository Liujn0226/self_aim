cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# 依赖声明为 PUBLIC：这样任何链接 io 的目标都能自动拿到
# 相机 SDK 的 include 路径、库搜索路径与这几个库。
target_link_libraries(io PUBLIC MvCameraControl MVSDK usb-1.0 yaml-cpp serial fmt::fmt)

# ============================================================================
# 校内赛自瞄专用目标
#
# contest_gimbal 只含新协议的通信层（gimbal.cpp），供 contest_gimbal_test 使用，
# 目的是让"验证串口协议"这一步完全不依赖相机 SDK 与相机驱动。
#
# 说明两点，避免后续误解：
#   1. gimbal.cpp 同时出现在 io 与 contest_gimbal 两个静态库里。
#      这是安全的：静态归档只在需要解析未定义符号时才被抽取对象文件，
#      不会出现重复定义。链接约定：
#        contest             -> io
#        calib_capture       -> io + contest_gimbal
#        contest_gimbal_test -> contest_gimbal（只有它，因此不碰相机 SDK）
#   2. 不再提供"只含相机"的独立目标。io/camera.cpp 同时 #include 了
#      hikrobot 与 mindvision 两家头文件（它是一个二选一的工厂），
#      所以任何编译它的目标都必须同时具备两家的 include 路径，
#      省不掉 MindVision。相机一律走 io。
# ============================================================================
add_library(contest_gimbal STATIC gimbal/gimbal.cpp)
target_include_directories(contest_gimbal PUBLIC gimbal)
target_link_libraries(contest_gimbal PUBLIC yaml-cpp serial fmt::fmt)

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

# 如果找不到 ROS 环境，则跳过 ROS 相关的部分
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND OR NOT sp_msgs_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 添加 ROS2 相关的文件
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

    # 设置 ROS2 依赖
    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io rclcpp std_msgs sp_msgs)
endif()
