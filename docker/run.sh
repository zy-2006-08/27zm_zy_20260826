#!/bin/bash
# ============================================================================
#  一键进入容器开发环境（在 Mac / Windows 上代替 WSL）
# ============================================================================
#  用法：
#    bash docker/run.sh            构建镜像（首次）并进入容器
#    bash docker/run.sh build      在容器里直接编译，编完就退出
#    bash docker/run.sh test       编译并运行 auto_aim_test（离线录像）
#
#  首次执行会下载 Ubuntu 镜像和 OpenVINO，约需十几分钟，之后都是秒级。
# ============================================================================
set -e

IMAGE_NAME=sp_vision_rb_aim:dev
CONTAINER_NAME=sp_vision_dev

# 仓库根目录 = 本脚本所在目录的上一层，这样在任何路径下执行都正确
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---------------------------------------------------------------------------
# 镜像不存在时才构建。想强制重建：docker rmi sp_vision_rb_aim:dev
#
# --platform linux/amd64 是硬性要求，不是性能选项：
# OpenVINO 的 apt 源只有 x86_64 包。在 Apple Silicon 上会通过翻译层执行。
# ---------------------------------------------------------------------------
if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
    echo "==> 首次构建镜像，需要下载 Ubuntu 和 OpenVINO，请耐心等待"
    docker build --platform linux/amd64 \
                 -t "$IMAGE_NAME" \
                 -f "$PROJECT_ROOT/docker/Dockerfile" \
                 "$PROJECT_ROOT/docker"
fi

# ---------------------------------------------------------------------------
# 决定容器里跑什么
#
# 构建目录就用 build/，和车上、和 怎么编译运行.md 里写的一致。
#
# ⚠️ 如果你曾在 Mac 上直接 cmake 过（哪怕失败了），build/ 里会留下一份指向
# macOS 编译器的 CMakeCache.txt，容器里再 cmake 会报
#   "CMakeCache.txt directory is different"
# 或者一堆莫名其妙的编译错。遇到就删掉重来：rm -rf build
# 本工程在 macOS 上本来就编不了（OpenVINO 只有 Linux 版），
# 所以 build/ 理应只由容器生成。
# ---------------------------------------------------------------------------
BUILD_CMD='cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && make -C build -j$(nproc)'

case "${1:-shell}" in
    build)
        RUN_ARGS=(bash -lc "$BUILD_CMD")
        INTERACTIVE=(-i)
        ;;
    test)
        # cd build 是必须的：程序内部找配置写的是 ../configs/demo.yaml
        RUN_ARGS=(bash -lc "$BUILD_CMD && cd build && ./auto_aim_test")
        INTERACTIVE=(-it)
        ;;
    shell)
        RUN_ARGS=(bash)
        INTERACTIVE=(-it)
        ;;
    *)
        echo "未知参数: $1"
        echo "可用: build | test | shell"
        exit 1
        ;;
esac

echo "==> 浏览器打开 http://localhost:6080/vnc.html 可看 OpenCV 窗口"

# --rm            退出即删容器，保持干净；源码和 build 产物都在挂载卷里，不会丢
# -v $ROOT:/workspace  挂载源码，Mac 上改代码容器内立刻可见
# -p 6080         noVNC 网页端口
# --shm-size      OpenCV 的 highgui 需要较大共享内存，默认 64MB 偶发崩溃
exec docker run --rm "${INTERACTIVE[@]}" \
    --platform linux/amd64 \
    --name "$CONTAINER_NAME" \
    -v "$PROJECT_ROOT:/workspace" \
    -p 6080:6080 \
    --shm-size=512m \
    "$IMAGE_NAME" \
    "${RUN_ARGS[@]}"
