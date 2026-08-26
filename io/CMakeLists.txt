cmake_minimum_required(VERSION 3.16)

# ============================================================
# io - 硬件抽象层 (Hardware Abstraction Layer)
#
# 本库封装相机与云台通信:
#   - daheng   : 大恒工业相机 (Daheng Galaxy)
#   - hikrobot : 海康工业相机 (HikRobot MVS)
#   - camera   : 相机统一接口, 按 YAML 的 camera_name 分发到上面两家
#   - cboard   : C 板 CAN 通信 (被 tasks/auto_aim/aimer.hpp 间接依赖)
#   - gimbal   : 云台串口通信 (上场程序实际使用的链路)
# ============================================================

# yaml-cpp: 读取 configs/*.yaml 里的相机与云台参数
# apt 包名见 README 的依赖列表
find_package(yaml-cpp REQUIRED)

# serial: 随仓库自带的第三方串口库 (io/serial/), gimbal 依赖它
# 该子目录及其 CMakeLists.txt 为原样引入, 请勿修改
add_subdirectory(serial)

# ------------------------------------------------------------
# 目标 io (静态库)
# 显式列出每个源文件, 不使用 file(GLOB), 便于审查与增量构建正确性
# ------------------------------------------------------------
add_library(io STATIC
    daheng/daheng.cpp
    hikrobot/hikrobot.cpp
    camera.cpp
    cboard.cpp
    gimbal/gimbal.cpp
)

# ------------------------------------------------------------
# daheng (大恒) 头文件
#   daheng/          -> daheng.hpp
#   daheng/include/  -> 厂商 SDK 头文件 (共 2 个)
# 这里只有头文件, 没有 lib/ 目录 —— 动态库要靠系统安装, 见文件末尾的说明
# ------------------------------------------------------------
target_include_directories(io PUBLIC daheng
                                    daheng/include)

# ------------------------------------------------------------
# hikrobot (海康) 头文件与库目录
# 与大恒不同, 海康的 libMvCameraControl.so 已随仓库放好, 位于
#   hikrobot/lib/amd64  与  hikrobot/lib/arm64
# 所以下面的 target_link_directories 直接指向仓库内路径,
# 海康这一侧不需要额外安装系统级 SDK。
#
# 按 CPU 架构选择对应的 .so:
#   x86_64  -> intel / 华硕 / 雷神 nuc
#   aarch64 -> jetson orin nano 等 arm 平台
# 其它架构 (例如 Windows 上的交叉环境) 直接报错终止,
# 早失败比后面出一堆看不懂的链接错误好。
# ------------------------------------------------------------
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
endif()

# ------------------------------------------------------------
# 链接依赖, 逐个说明最后一行 target_link_libraries 里的 5 个库:
#
# 第 1 个 (大恒相机 SDK) 【重点, 新手最容易踩的坑】
#   这个库【不在本仓库内】—— io/daheng/ 下只有源码和头文件, 没有 lib/ 目录。
#   必须先在小电脑上全局安装 Daheng Galaxy SDK:
#     下载 Galaxy_Linux_CN-EN_32bits/64bits, 解压后 cd 进目录运行 ./Galaxy_camera.run
#   没装的话失败方式很坑: cmake 配置能过, 每个 .cpp 都能编过,
#   只有最后一步链接会报 "cannot find -l<该库名>",
#   也就是忙活十来分钟之后在最后一秒才失败。
#   而且大恒不是可选项: camera.cpp 里 DahengCamera::initSDK() 是无条件调用的,
#   无论 YAML 里选的是哪个品牌的相机, 都要链上它。
#
# 第 2 个 (MvCameraControl) 海康相机 SDK
#   与大恒相反, 它来自仓库内 hikrobot/lib/<arch>/, 无需安装系统 SDK。
#
# 第 3 个 (libusb) hikrobot.cpp 用它复位相机 USB 设备
#   调用了 libusb_init / libusb_open_device_with_vid_pid /
#   libusb_reset_device / libusb_close, 需安装 libusb 开发包, 见 README 依赖列表。
#
# 第 4 个 (yaml-cpp) 配置文件解析, 对应文件开头的 find_package。
#
# 第 5 个 (serial) 上面 add_subdirectory 引入的串口库, gimbal 使用。
# ------------------------------------------------------------
target_link_libraries(io gxiapi MvCameraControl usb-1.0 yaml-cpp serial)
