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
# compile_commands.json（clangd 的编译数据库）
# 在这里开而不是只靠 make.sh 传 -D：scripts/build_current.sh 在 build/ 未配置时
# 会自己跑一次 cmake，那条路径没带 -DCMAKE_EXPORT_COMPILE_COMMANDS，
# 于是首次用 Alt+, 配置出来的 build/ 里没有 DB，clangd 只能用 .clangd 的兜底
# 参数，工程内部的 #include "tasks/..." 全部解析失败 —— 满屏假红线。
# 写在 CMakeLists 里，make.sh 和 build_current.sh 两条路径都覆盖。
# ---------------------------------------------------------------------------
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
#   detect_test                  相机识别测试（接相机跑识别，识别到就画框）
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

# camera_tuning 直接调厂商 SDK，不经过 io::Camera ——
# io::Camera 只在构造时设一次曝光，运行期改不了，而调参需要即时生效。
#
# 它同时链大恒(gxiapi)和海康(MvCameraControl)：程序里把相机操作抽成
# TuningCamera 接口，两家各实现一份，启动时先枚举海康再枚举大恒。
# 原来只链 gxiapi，插海康相机时报"没有找到相机"——大恒 SDK 枚举不到海康设备。
#
# 不链接 io 的原因同上（运行期改不了参数），另外还有一条：
# io::Camera::initSDK() 无条件调用大恒的 GXInitLib，走 io 就把大恒
# 变成了硬依赖，而本程序需要在只装了一家 SDK 的机器上也能用。
add_executable(camera_tuning calibration/camera_tuning.cpp)

# 海康头文件与库：优先系统安装的 /opt/MVS，回落到仓库内。
# 与上面 hik_grab 的处理保持一致。
if(EXISTS "/opt/MVS/include/MvCameraControl.h")
    target_include_directories(camera_tuning PRIVATE /opt/MVS/include)
    target_link_directories(camera_tuning PRIVATE /opt/MVS/lib/64)
    set_target_properties(camera_tuning PROPERTIES INSTALL_RPATH "/opt/MVS/lib/64"
                                                   BUILD_WITH_INSTALL_RPATH TRUE)
else()
    target_include_directories(camera_tuning PRIVATE ${PROJECT_SOURCE_DIR}/io/hikrobot/include)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        target_link_directories(camera_tuning PRIVATE ${PROJECT_SOURCE_DIR}/io/hikrobot/lib/amd64)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        target_link_directories(camera_tuning PRIVATE ${PROJECT_SOURCE_DIR}/io/hikrobot/lib/arm64)
    endif()
endif()

target_link_libraries(camera_tuning ${OpenCV_LIBS} fmt::fmt yaml-cpp tools gxiapi MvCameraControl)

# hik_grab 海康相机独立取图工具。
# 与 camera_tuning 同属"绕过 io::Camera 直连厂商 SDK"这一类, 但方向相反:
# camera_tuning 只支持大恒(链 gxiapi), hik_grab 只支持海康(链 MvCameraControl)。
#
# 为什么不链接 io:
#   io::Camera::initSDK() 无条件调用 DahengCamera::initSDK(), 也就是说走 io
#   就必须同时装好大恒 SDK; 而本工具的用途正是"在只有海康相机的机器上验证出图",
#   不该被大恒的系统级依赖绑住。同理也不链 auto_aim, 避免拖进 OpenVINO 和权重。
#
# 库来源与 io/CMakeLists.txt 保持一致: 优先用系统安装的 /opt/MVS,
# 没装则回落到仓库内 io/hikrobot/lib/<arch>。
# 注意仓库内那份只有主库 libMvCameraControl.so, 缺 libMvUsb3vTL.so 等传输层插件,
# 单靠它枚举会返回 0x80000006, 所以 /opt/MVS 实际上是必需的, 详见源码末尾说明。
add_executable(hik_grab calibration/hik_grab.cpp)

if(EXISTS "/opt/MVS/include/MvCameraControl.h")
    target_include_directories(hik_grab PRIVATE /opt/MVS/include)
    target_link_directories(hik_grab PRIVATE /opt/MVS/lib/64)
    set_target_properties(hik_grab PROPERTIES INSTALL_RPATH "/opt/MVS/lib/64"
                                              BUILD_WITH_INSTALL_RPATH TRUE)
    message(STATUS "-------------------- hik_grab: 使用 /opt/MVS --------------------")
else()
    target_include_directories(hik_grab PRIVATE ${PROJECT_SOURCE_DIR}/io/hikrobot/include)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        target_link_directories(hik_grab PRIVATE ${PROJECT_SOURCE_DIR}/io/hikrobot/lib/amd64)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        target_link_directories(hik_grab PRIVATE ${PROJECT_SOURCE_DIR}/io/hikrobot/lib/arm64)
    else()
        message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
    endif()
    message(WARNING "/opt/MVS 未安装, hik_grab 回落到仓库内 SDK; "
                    "该 SDK 缺传输层插件, 运行时枚举可能返回 0x80000006")
endif()

target_link_libraries(hik_grab ${OpenCV_LIBS} MvCameraControl)

# ---------------------------------------------------------------------------
# rerun C++ SDK（波形可视化，可选 / optional）
# ---------------------------------------------------------------------------
# 用途：detect_test 把「识别状态」和「装甲板 roll 角」两条数据实时画成波形，
#      代替原来在图像上叠字 —— 叠字只能看到当前帧的瞬时值，
#      而调参真正需要的是「这个值在时间上怎么变化」。
#
# 为什么是 FetchContent 而不是 find_package：
#      rerun 没有进 apt 源，官方推荐方式就是 FetchContent 拉预编译包。
#      包里已含静态库 librerun_c.a，不需要编译 Rust，配置完就能链。
#
# 为什么默认从本地 zip 读（RERUN_SDK_ZIP）：
#      小电脑访问 github.com 会超时（实测直连和 7 个国内镜像全部失败），
#      让 CMake 每次去联网拉包会直接卡死在配置阶段。所以改成
#      「先手动把 zip 下到本地，再指给 CMake」，离线可重复构建。
#
# 为什么整段用 WITH_RERUN 开关包住：
#      队友的机器上可能没有这个 zip。没有 zip 时如果硬拉 FetchContent，
#      整个工程（包括 rb_auto_aim_debug 这些不相关的目标）都配置不过。
#      加开关后，没 zip 就自动退化成「不带波形的原版 detect_test」，
#      其余程序完全不受影响。
#
# 用法：
#      带波形（zip 放在 third_party/ 下，默认路径）：
#          cmake -B build
#      zip 放在别处：
#          cmake -B build -DRERUN_SDK_ZIP=/home/seven/rerun_pkg/rerun_cpp_sdk.zip
#      明确不要波形：
#          cmake -B build -DWITH_RERUN=OFF
# ---------------------------------------------------------------------------
option(WITH_RERUN "Build detect_test with rerun waveform logging" ON)
set(RERUN_SDK_ZIP "${PROJECT_SOURCE_DIR}/third_party/rerun_cpp_sdk.zip"
    CACHE FILEPATH "Path to rerun_cpp_sdk.zip downloaded from GitHub releases")

set(RERUN_AVAILABLE FALSE)
if(WITH_RERUN)
    if(EXISTS "${RERUN_SDK_ZIP}")
        include(FetchContent)

        # ★关掉 rerun 自带的 Arrow 下载编译。
        #
        # rerun_cpp/CMakeLists.txt 里有 option(RERUN_DOWNLOAD_AND_BUILD_ARROW ... ON)，
        # 默认 ON 时配置阶段会去 GitHub 拉 apache-arrow-18.0.0.tar.gz 再从源码编译。
        # 小电脑访问 GitHub 只有 20 KB/s，那个包上百 MB —— 光下载就要几小时，
        # 编译 Arrow 本身还要十几分钟，整个 cmake 会像卡死一样毫无输出。
        #
        # 关掉后 rerun 走 find_package(Arrow REQUIRED)，用系统装的版本：
        #     sudo apt install libarrow-dev      # Ubuntu 26.04 提供 23.0.1
        # 走国内镜像几分钟就装好。
        #
        # 版本差异：rerun 官方绑定 Arrow 18.0.0，apt 给的是 23.0.1。
        # rerun 只用 Arrow 最基础的数组/schema 序列化 API，这部分多年稳定，
        # 且官方源码注释已提到 "upgrading to Arrow 25 or newer"，预期兼容更新版本。
        # 万一真出 ABI 问题，把这两行删掉让它自己下 18.0.0 即可（代价是几小时）。
        set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF CACHE BOOL "" FORCE)
        set(RERUN_ARROW_LINK_SHARED ON CACHE BOOL "" FORCE)

        # URL 支持本地文件路径，不必写成 file:// 协议。
        # 不校验 hash：zip 是用户手动下的，版本可能不同；
        # 真正的完整性问题会在解压阶段直接报错，足够暴露。
        FetchContent_Declare(rerun_sdk URL "${RERUN_SDK_ZIP}")
        FetchContent_MakeAvailable(rerun_sdk)
        set(RERUN_AVAILABLE TRUE)
        message(STATUS "-------------------- rerun: ON (${RERUN_SDK_ZIP}) --------------------")
    else()
        message(WARNING
            "rerun SDK zip not found at ${RERUN_SDK_ZIP}\n"
            "  detect_test will build WITHOUT waveform logging.\n"
            "  Download it from:\n"
            "    https://github.com/rerun-io/rerun/releases/download/0.37.1/rerun_cpp_sdk.zip\n"
            "  then re-run cmake with:\n"
            "    cmake -B build -DRERUN_SDK_ZIP=/path/to/rerun_cpp_sdk.zip")
    endif()
endif()

# detect_test = test_simple 的相机 + auto_aim_test 的识别画框。
# 工程原本没有「接相机 + 跑识别」的组合：test_simple 不识别，
# auto_aim_test 读录像不接相机，rb_auto_aim_debug 要连 C 板串口否则 exit(1)。
# 链接 auto_aim 拿 YOLO/Detector，链接 io 拿相机。
add_executable(detect_test calibration/detect_test.cpp)
target_link_libraries(detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# USE_RERUN 宏让 detect_test.cpp 里的 rerun 代码在缺 SDK 时整段编译掉，
# 这样同一份源码既能带波形跑，也能在没 SDK 的机器上编过。
if(RERUN_AVAILABLE)
    target_link_libraries(detect_test rerun_sdk)
    target_compile_definitions(detect_test PRIVATE USE_RERUN)
endif()
