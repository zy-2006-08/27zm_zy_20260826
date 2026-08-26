cmake_minimum_required(VERSION 3.16)

# ==============================================================================
# auto_aim - 自瞄功能层
# 本项目只保留 OpenVINO 推理后端, 没有后端选择开关, 因此这里无条件配置 OpenVINO。
# Only the OpenVINO inference backend is kept, so OpenVINO is configured
# unconditionally here (no backend-selection switches remain).
# ==============================================================================

# OpenVINO 路径: 允许用户通过 -DOpenVINO_DIR=... 或 export OpenVINO_DIR=... 覆盖,
# 只有两者都没设置时才回退到默认安装路径 (与根 CMakeLists 的守卫写法一致)。
# Overridable OpenVINO location: fall back to the default install path only when
# neither a cache variable nor an environment variable is provided.
if(NOT DEFINED OpenVINO_DIR AND NOT DEFINED ENV{OpenVINO_DIR})
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
endif()
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# 下面这个宏定义是必需的, 不是可选项:
# yolo.cpp 把三个后端头文件 (yolos/yolov5.hpp, yolos/yolov8.hpp, yolos/yolo11.hpp)
# 包在 #ifdef OPENVINO_MAKE 里。缺少该宏, yolo.cpp 就会编译出一个没有任何后端的
# auto_aim::YOLO, 而 src/rb_auto_aim_debug.cpp 会去构造它, 必然失败。
# Mandatory, not optional: yolo.cpp guards all three backend includes with
# "#ifdef OPENVINO_MAKE"; without it YOLO compiles with no backend at all.
add_definitions(-DOPENVINO_MAKE)

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
    rv_detector.cpp
    rv_number.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    planner/planner.cpp
)

# planner/tinympc 是随项目携带的第三方 MPC 求解器 (vendored), 原样复制并自带
# CMakeLists, 不要修改其中任何内容。planner/planner.hpp 通过
# #include "tinympc/tiny_api.hpp" 依赖它, 它导出的静态库目标即下面链接的那个。
# Vendored third-party MPC solver, copied verbatim with its own CMakeLists.
# planner/planner.hpp includes "tinympc/tiny_api.hpp"; its static library target
# is the last one linked below.
add_subdirectory(planner/tinympc)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)
