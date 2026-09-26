#!/usr/bin/env bash
# 在 Mac (Apple Silicon) 上原生编译「装甲板识别 + 整车估计 + 瞄准」这条链，
# 喂录制视频跑，不需要相机、串口板、C 板，也不需要虚拟机或 Docker。
#
#   ./mac/build.sh              编译
#   ./mac/build.sh run          编译并跑 demo 视频（带 imshow 窗口）
#   ./mac/build.sh clean        删掉产物
#
# 能编译的原因：
#   - OpenVINO 有 arm64 macOS 版（brew install openvino 自带 arm CPU plugin），
#     yolov5/yolov8/yolo11 权重在 M 系列 CPU 上推理约 6ms。
#   - 这条链只依赖 OpenCV / Eigen / fmt / spdlog / yaml-cpp / OpenVINO，
#     完全不碰海康、迈德威视、大恒的 Linux 专用 .so。
#
# 对比 docker/ 那套方案：容器要靠 QEMU 模拟 x86，同样的模型推理约 530ms，
# 慢约 90 倍，还得开浏览器看 noVNC。日常调这条链就用本脚本。
#
# 编不出来的部分（离线也用不到）：
#   相机驱动、CAN 通信、planner (TinyMPC)、shooter、打符、全向感知。
#   上车、调参、实机测试仍然在小电脑上做。
#
# 本脚本移植自同仓库群的 sp_vision_25_rbclone/mac/build.sh。

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/build-mac"
cd "$ROOT"

if [ "${1:-}" = "clean" ]; then
    rm -rf "$OUT"
    # 连 clangd 用的 DB 软链一起收掉（只删软链，绝不碰真文件）
    [ -L "$ROOT/compile_commands.json" ] && rm -f "$ROOT/compile_commands.json"
    # 历史遗留：早期版本会建 build -> build-mac 的目录级软链，见下方说明。
    # 清理掉，避免它在 Linux 侧变成坏链接挡住 cmake -B build。
    [ -L "$ROOT/build" ] && rm -f "$ROOT/build"
    echo "已删除 $OUT"
    exit 0
fi

for f in opencv@4 eigen fmt spdlog yaml-cpp nlohmann-json openvino libomp; do
    if ! brew --prefix "$f" > /dev/null 2>&1; then
        echo "缺少依赖：$f" >&2
        echo "执行：brew install opencv@4 eigen fmt spdlog yaml-cpp nlohmann-json openvino libomp" >&2
        exit 1
    fi
done

OV=$(brew --prefix openvino)
OCV=$(brew --prefix opencv@4)
OMP=$(brew --prefix libomp)

INC=(
    -I"$ROOT"
    -I"$ROOT/io/serial/include"
    -I"$ROOT/.clangd-stubs"          # linux/can.h 等 Linux 专属头的桩
    -I"$OV/include"
    -I"$OCV/include/opencv4"
    -I"$(brew --prefix eigen)/include/eigen3"
    -I"$(brew --prefix fmt)/include"
    -I"$(brew --prefix spdlog)/include"
    -I"$(brew --prefix yaml-cpp)/include"
    -I"$(brew --prefix nlohmann-json)/include"
    -I"$OMP/include"
)

# -w 关掉警告：Mac 上的库版本比小电脑新，会刷一堆和代码质量无关的 deprecated 提示。
# -include mac/compat.hpp 注入兼容补丁（fmt 版本差异、macOS 的 htons 宏），不改仓库源码。
CXXFLAGS=(-std=c++17 -O2 -w -DOPENVINO_MAKE -Xpreprocessor -fopenmp
          -include "$ROOT/mac/compat.hpp")

SOURCES=(
    tasks/auto_aim/armor.cpp                 # 灯条 / 装甲板几何
    tasks/auto_aim/classifier.cpp            # 数字分类（tiny_resnet）
    tasks/auto_aim/detector.cpp              # 传统识别（阈值 + 轮廓）
    tasks/auto_aim/yolo.cpp                  # 识别器工厂，按 yaml 里的 yolo_name 分发
    tasks/auto_aim/yolos/yolov5.cpp
    tasks/auto_aim/yolos/yolov8.cpp
    tasks/auto_aim/yolos/yolo11.cpp
    tasks/auto_aim/yolos/yolo7.cpp
    tasks/auto_aim/solver.cpp                # PnP 位姿解算 + yaw 优化
    tasks/auto_aim/target.cpp                # 整车估计 EKF
    tasks/auto_aim/tracker.cpp               # 状态机：detecting/tracking/lost
    tasks/auto_aim/aimer.cpp                 # 瞄准点决策（旧版，非 MPC）
    tools/img_tools.cpp
    tools/logger.cpp
    tools/math_tools.cpp
    tools/extended_kalman_filter.cpp
    tools/trajectory.cpp                     # 弹道解算
    tools/plotter.cpp                        # 往 PlotJuggler 发 UDP
    tools/exiter.cpp
    tools/crc.cpp
    io/gimbal/gimbal.cpp                     # aimer/tracker 引用了它，必须链接
    io/serial/src/serial.cc
    io/serial/src/impl/unix.cc
    io/serial/src/impl/list_ports/list_ports_osx.cc
    tests/auto_aim_test.cpp                  # main：读 avi + txt 四元数，跑全链
)

mkdir -p "$OUT/obj"

echo "==> 编译 ${#SOURCES[@]} 个文件"
pids=()
for src in "${SOURCES[@]}"; do
    obj="$OUT/obj/$(echo "$src" | tr '/' '_').o"
    # 源文件比 .o 新才重编，改一个文件不用等全部
    if [ -f "$obj" ] && [ "$obj" -nt "$src" ]; then continue; fi
    g++ "${CXXFLAGS[@]}" "${INC[@]}" -c "$src" -o "$obj" &
    pids+=($!)
done
fail=0
for pid in ${pids[@]+"${pids[@]}"}; do wait "$pid" || fail=1; done
[ "$fail" = 0 ] || { echo "编译失败" >&2; exit 1; }

echo "==> 链接"
OCV_LIBS=()
for l in core imgproc imgcodecs videoio highgui dnn calib3d features2d flann; do
    OCV_LIBS+=(-lopencv_$l)
done

g++ -o "$OUT/auto_aim_test" "$OUT"/obj/*.o \
    -L"$OV/lib" -lopenvino \
    -L"$OCV/lib" "${OCV_LIBS[@]}" \
    -L"$(brew --prefix fmt)/lib" -lfmt \
    -L"$(brew --prefix yaml-cpp)/lib" -lyaml-cpp \
    -L"$(brew --prefix spdlog)/lib" -lspdlog \
    -L"$OMP/lib" -lomp \
    -framework IOKit -framework Foundation

echo "==> 产物：$OUT/auto_aim_test"

# ---- 给 clangd 在工程根建 compile_commands.json 软链 ----
# clangd 自动搜索时只认两个位置（实测 clangd 21）：
#     <工程根>/compile_commands.json      <- 用这个
#     <工程根>/build/compile_commands.json
# build-mac 这个名字它不认。而 .clangd 里刻意没写 CompilationDatabase ——
# 那个字段一旦指向不存在的目录，clangd 会停止搜索并直接失败。
#
# 早期版本这里建的是 build -> build-mac 的【目录级】软链，去命中第二个位置。
# 那个做法出过事：注释里以为 .gitignore 的 build/ 能挡住它，但带斜杠是
# 目录专用模式，挡不住符号链接（git 视其为 mode 120000 的普通文件），
# 于是软链被提交进版本库（commit ad75477），拉到小电脑后成了指向不存在
# 目录的坏链接，cmake -B build 无法创建 build/CMakeFiles/pkgRedirects，
# Alt+, 编译直接失败。
#
# 改成软链【单个文件】到第一个位置，根治：
#   - build/ 和 build-mac/ 各自独立，不再争抢同一个路径名
#   - Linux 的真 build/ 目录永远不会被碰到，不需要 [ -L ] 守卫
#   - .gitignore 已加 compile_commands.json（无斜杠），目录和软链都挡得住
if [ -f "$OUT/compile_commands.json" ]; then
    ln -sfn "build-mac/compile_commands.json" "$ROOT/compile_commands.json"
    echo "==> clangd 软链：compile_commands.json -> build-mac/compile_commands.json"
fi

# compile_commands.json 不是 build.sh 生成的，缺了 clangd 只能用兜底参数
if [ ! -f "$OUT/compile_commands.json" ]; then
    echo "==> 提示：$OUT/compile_commands.json 不存在，clangd 精度会下降" >&2
    echo "    生成：python3 mac/gen_compile_commands.py" >&2
fi

if [ "${1:-}" = "run" ]; then
    # 配置文件里的模型路径写的是 ../assets/xxx（相对于小电脑上的 build/ 目录），
    # 所以要在 build-mac/ 里运行才找得到。
    mkdir -p "$OUT/logs"
    cd "$OUT"
    echo "==> 跑 demo 视频（窗口里按 q 退出）"
    ./auto_aim_test -c=../configs/demo.yaml ../assets/demo/demo
fi
