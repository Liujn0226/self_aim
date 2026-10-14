cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

# auto_aim 的代码里用了 tools::logger() 等，所以显式声明对 tools 的依赖。
# tools 是 OBJECT 库：链接它的目标会直接吸收其对象文件，
# 因此能保证 tools 的符号出现在链接命令里，不必依赖顶层目标的书写顺序。
target_link_libraries(auto_aim io tools openvino::runtime tinympcstatic)