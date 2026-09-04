cmake_minimum_required(VERSION 3.16.3)

project(sp_vision_rb_aim)

# ---------------------------------------------------------------------------
# C++ 标准 / C++ standard
# 本项目使用 C++17（结构化绑定、std::optional、std::filesystem 等）
# STANDARD_REQUIRED ON 表示：编译器不支持 C++17 时直接报错，而不是静默降级
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# 构建类型 / Build type
# 默认 Release（自瞄对帧率敏感，Debug 版本会慢好几倍）。
# 想调试时无需改动本文件，命令行覆盖即可：
#   cmake -B build -DCMAKE_BUILD_TYPE=Debug
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "-------------------- CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE} --------------------")

# ---------------------------------------------------------------------------
# 第三方依赖 / Third-party dependencies
# Ubuntu 上一次装齐：
#   sudo apt install -y libopencv-dev libfmt-dev libeigen3-dev \
#                       libspdlog-dev libyaml-cpp-dev nlohmann-json3-dev
# 全部 REQUIRED：缺任何一个都无法编译，早报错比链接阶段报错好排查
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# ---------------------------------------------------------------------------
# OpenVINO（神经网络推理，必需 / mandatory）
# 装甲板检测器与数字分类器都 #include <openvino/openvino.hpp>，不是可选项。
#
# 这里用 if(NOT DEFINED ...) 做保护而不是直接 set()，原因很实在：
# OpenVINO 有两种安装方式，路径不同 ——
#   * 官网压缩包安装 -> /opt/intel/openvino_2024.6.0/runtime/cmake
#   * apt 安装       -> /usr/lib/cmake/openvino2024.6.0
# 无条件 set() 会覆盖环境变量，导致 apt 安装的用户直接报
# "Could not find a package configuration file provided by OpenVINO"。
# 有了这个保护，下面两种覆盖方式都能生效：
#   export OpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0
#   cmake -B build -DOpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0
# 都没设置时，才回退到官网压缩包的默认路径。
# ---------------------------------------------------------------------------
if(NOT DEFINED OpenVINO_DIR AND NOT DEFINED ENV{OpenVINO_DIR})
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
endif()
find_package(OpenVINO REQUIRED)

# ---------------------------------------------------------------------------
# 头文件搜索路径 / Include directories
# 这四行都是必需的，缺一行就编译不过：
#   1) EIGEN3_INCLUDE_DIR   -> #include <Eigen/Dense>
#   2) OpenCV_INCLUDE_DIRS  -> #include <opencv2/opencv.hpp>
#   3) PROJECT_SOURCE_DIR   -> 全工程都用「工程根目录相对路径」写 include，
#                              例如 #include "tasks/auto_aim/tracker.hpp"、
#                              #include "io/camera.hpp"、#include "tools/yaml.hpp"。
#                              删掉这行，整个项目一个文件都编不过。
#   4) io/serial/include    -> io/gimbal/gimbal.hpp 里 #include "serial/serial.h"，
#                              用的是内置(vendored)串口库 io/serial/。
# 注意：这些是根作用域的设置，子目录会继承。tools/ 子工程自身不声明任何依赖，
# 完全依赖这里的 find_package + include_directories 状态，所以四行都不能省。
# ---------------------------------------------------------------------------
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/io/serial/include)

# ---------------------------------------------------------------------------
# 子模块 / Sub-projects
#   tools          工具层：数学、滤波、弹道、日志、YAML 解析
#   io             硬件抽象层：相机、云台、串口
#   tasks/auto_aim 功能层：装甲板识别、解算、跟踪、火控
# ---------------------------------------------------------------------------
add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)

# ---------------------------------------------------------------------------
# 应用程序 / Application
# 上车调试用的自瞄主程序
# ---------------------------------------------------------------------------
add_executable(rb_auto_aim_debug src/rb_auto_aim_debug.cpp)
target_link_libraries(rb_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# ---------------------------------------------------------------------------
# 测试程序 / Test
# 用录制视频离线跑整条自瞄链路，不需要接相机和云台
# ---------------------------------------------------------------------------
add_executable(auto_aim_test tests/auto_aim_test.cpp)
target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# ---------------------------------------------------------------------------
# 标定程序 / Calibration
#   capture                      采集标定图像 + 对应四元数
#   calibrate_input              标定数据输入辅助
#   calibrate_camera             相机内参标定
#   calibrate_handeye            手眼标定
#   calibrate_robotworld_handeye 手眼标定（同时求标定板位置）
#   split_video                  视频抽帧
#   test_simple                  重投影简单验证
#   camera_tuning                相机调参助手（实时看清晰度/亮度，调曝光增益）
# 需要连相机/云台的程序才链接 io，纯离线处理图片的只链接 tools。
# ---------------------------------------------------------------------------
add_executable(capture calibration/capture.cpp)
target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

add_executable(calibrate_input calibration/calibrate_input.cpp)
target_link_libraries(calibrate_input ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

add_executable(calibrate_camera calibration/calibrate_camera.cpp)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)

add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)

add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)

add_executable(split_video calibration/split_video.cpp)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

add_executable(test_simple calibration/test_simple.cpp)
target_link_libraries(test_simple ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

# camera_tuning 直接调大恒 SDK（gxiapi），不经过 io::Camera ——
# io::Camera 只在构造时设一次曝光，运行期改不了，而调参需要即时生效。
# 因此这里不链接 io，而是单独链 gxiapi；也因此本程序只支持大恒相机。
add_executable(camera_tuning calibration/camera_tuning.cpp)
target_link_libraries(camera_tuning ${OpenCV_LIBS} fmt::fmt yaml-cpp tools gxiapi)
