#!/bin/bash
# ============================================================================
#  容器入口脚本：准备图形环境，然后执行用户传入的命令
# ============================================================================
#  被 Dockerfile 的 ENTRYPOINT 调用，docker run 后面跟的命令会作为 "$@" 传进来。
# ============================================================================
set -e

VNC_PORT=5900
WEB_PORT=6080
SCREEN_GEOMETRY=1600x1000x24

# ---------------------------------------------------------------------------
# 1. Xvfb：虚拟屏幕。没有它 cv::imshow 会直接抛 "cannot connect to X server"。
# ---------------------------------------------------------------------------
if ! pgrep -x Xvfb > /dev/null; then
    Xvfb "$DISPLAY" -screen 0 "$SCREEN_GEOMETRY" -nolisten tcp > /tmp/xvfb.log 2>&1 &

    # 等 X server 真正就绪再往下走。直接启动后续进程会偶发连不上，
    # 因为 Xvfb 创建 socket 需要一点时间。
    for _ in $(seq 50); do
        if xdpyinfo -display "$DISPLAY" > /dev/null 2>&1; then break; fi
        sleep 0.1
    done
fi

# ---------------------------------------------------------------------------
# 2. x11vnc：把虚拟屏幕内容变成 VNC 流
#    -forever  客户端断开后继续服务，不退出（否则关一次浏览器就得重启容器）
#    -nopw     不设密码，仅监听容器内本地，由 noVNC 转发出去
#    -shared   允许多个客户端同时连
# ---------------------------------------------------------------------------
if ! pgrep -x x11vnc > /dev/null; then
    x11vnc -display "$DISPLAY" -rfbport "$VNC_PORT" \
           -forever -shared -nopw -quiet > /tmp/x11vnc.log 2>&1 &
fi

# ---------------------------------------------------------------------------
# 3. noVNC：网页版 VNC 客户端，把 VNC 包在 WebSocket 里给浏览器
# ---------------------------------------------------------------------------
if ! pgrep -f websockify > /dev/null; then
    websockify --web=/usr/share/novnc "$WEB_PORT" \
               "localhost:$VNC_PORT" > /tmp/novnc.log 2>&1 &
fi

# ---------------------------------------------------------------------------
# 4. 大恒相机 SDK 占位库
#    必须在编译之前就位，否则最后链接阶段才报 "cannot find -lgxiapi"。
#    脚本内部会跳过已生成的情况，重复进容器不会重复编译。
#    源码是挂载进来的，所以只能在容器启动时做，不能在 build 镜像时做。
# ---------------------------------------------------------------------------
if [ -d /workspace/io/daheng/include ]; then
    /usr/local/bin/make-daheng-stub.sh
fi

cat <<BANNER

  ────────────────────────────────────────────────────────────────
   容器已就绪（Ubuntu 22.04 / x86_64）

   看画面：在 Mac 浏览器打开  http://localhost:6080/vnc.html
           然后点 Connect（不需要密码）

   编译：  bash make.sh              首次编译，或改了 CMakeLists.txt
           make -C build -j\$(nproc)  日常增量编译

   运行：  cd build && ./auto_aim_test
           （必须先 cd build，程序找配置用的是 ../configs/ 相对路径）
  ────────────────────────────────────────────────────────────────

BANNER

exec "$@"
