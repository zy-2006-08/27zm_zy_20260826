#!/usr/bin/env bash
# ============================================================================
#  按「当前编辑的文件」决定运行哪个可执行文件
# ----------------------------------------------------------------------------
#  本工程一个 CMake 工程里有 10 个 add_executable，但 Alt+M 只能绑一个任务。
#  这个脚本让 Alt+M 跟着编辑器焦点走：打开 camera_tuning.cpp 按 Alt+M 就跑
#  camera_tuning，打开 auto_aim_test.cpp 就跑 auto_aim_test，不用记也不用改配置。
#
#  映射不是写死的，而是启动时从 CMakeLists.txt 里的 add_executable 现解析出来。
#  以后新增 add_executable(foo bar/foo.cpp) 会自动生效，不需要动本脚本。
#
#  用法：run_current.sh <当前文件相对工程根的路径> [传给程序的额外参数...]
#  由 .vscode/tasks.json 的 run / flash 任务调用，一般不手动执行。
# ============================================================================
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REL_FILE="${1:-}"
shift || true

# 当前打开的是日志(.log): Alt+N 显示这份日志的摘要(scripts/log_summary.py), 不启动任何程序。
# 没有这一段时 .log 会回退到下面的默认目标, 也就是把上场程序直接跑起来。
case "$REL_FILE" in
    *.log) exec python3 "$ROOT/scripts/log_summary.py" "$REL_FILE" --open ;;
    *日志摘要.txt) exec python3 "$ROOT/scripts/log_summary.py" --open ;;
esac

# 产物目录：Mac 原生编译在 build-mac，Linux 在 build
if [ "$(uname)" = Darwin ]; then BUILD_DIR="build-mac"; else BUILD_DIR="build"; fi

# 默认程序。打开的文件不是任何可执行文件的源文件时（比如在改 detector.cpp、
# tracker.hpp 这类库代码）跑它 —— 离线自瞄测试覆盖面最广，最常用。
DEFAULT_TARGET="rb_auto_aim_debug"

# ---- 从 CMakeLists.txt 解析 add_executable(<目标> <源文件>) ----
# 用 grep 抓行、sed 剥括号，避免依赖 cmake --build 的元数据（那个要先配置过）。
resolve_target() {
    local rel="$1"
    [ -z "$rel" ] && return 1
    # 只取文件名比较，规避 VSCode 传绝对路径 / 相对路径的差异
    local base
    base="$(basename "$rel")"
    grep -oE '^[[:space:]]*add_executable\([^)]*\)' "$ROOT/CMakeLists.txt" 2>/dev/null \
      | sed -E 's/^[[:space:]]*add_executable\(//; s/\)$//' \
      | while read -r target src rest; do
            [ -z "${src:-}" ] && continue
            if [ "$(basename "$src")" = "$base" ]; then
                echo "$target"
                return 0
            fi
        done
}

TARGET="$(resolve_target "$REL_FILE" | head -1)"

if [ -n "${TARGET:-}" ]; then
    echo "==> 当前文件 $REL_FILE 对应目标：$TARGET"
else
    TARGET="$DEFAULT_TARGET"
    if [ -n "$REL_FILE" ]; then
        echo "==> $REL_FILE 不是任何可执行文件的源文件，回退默认目标：$TARGET"
    else
        echo "==> 没有活动文件，使用默认目标：$TARGET"
    fi
fi

# 设 RUN_CURRENT_DRY=1 只解析不执行，用于验证映射是否正确
if [ -n "${RUN_CURRENT_DRY:-}" ]; then
    echo "[dry-run] 将运行 $BUILD_DIR/$TARGET"
    exit 0
fi

EXE="$ROOT/$BUILD_DIR/$TARGET"
if [ ! -x "$EXE" ]; then
    echo "!! $BUILD_DIR/$TARGET 不存在或不可执行" >&2
    echo "   先编译：make -C $BUILD_DIR $TARGET" >&2
    exit 1
fi

# 相机独占：同一时刻只能一个程序开着相机。切换程序前先清掉上一个，
# 否则大恒 SDK 报 -0x3ec（GX_STATUS_INVALID_ACCESS，设备被占用）。
#
# ⚠️ 不能用 pgrep -f "build/xxx" 来找进程。程序是在 build/ 里以 ./xxx 启动的，
# cmdline 就是 "./xxx"，不含 build/，那样匹配永远是空。这里改成读
# /proc/<pid>/exe 的真实指向来比对绝对路径，不受启动方式影响。
#
# 杀完还要等内核回收 USB 句柄：kill 后立刻开相机仍会拿到 -0x3ec，
# 因为 libusb 的设备节点尚未释放。这里轮询确认没人再持有 USB 节点。
CAMERA_USERS="camera_tuning test_simple capture rb_auto_aim_debug calibrate_input"

# 列出由本工程 build 目录启动、且属于相机程序的进程号
pids_of() {
    local prog="$1"
    local want="$ROOT/$BUILD_DIR/$prog"
    for pid in $(pgrep -x "$prog" 2>/dev/null); do
        local exe
        exe="$(readlink -f "/proc/$pid/exe" 2>/dev/null)"
        [ "$exe" = "$want" ] && echo "$pid"
    done
}

# 是否还有任何进程持有 USB 设备节点（相机走 USB3，节点在 /dev/bus/usb/）
usb_still_held() {
    local pid
    for prog in $CAMERA_USERS; do
        for pid in $(pids_of "$prog"); do
            if ls -l "/proc/$pid/fd" 2>/dev/null | grep -q "bus/usb"; then
                return 0
            fi
        done
    done
    return 1
}

case " $CAMERA_USERS " in
    *" $TARGET "*)
        killed=0
        for prog in $CAMERA_USERS; do
            for pid in $(pids_of "$prog"); do
                if [ "$prog" = "$TARGET" ]; then
                    echo "==> 结束上一个 $prog 实例 (pid $pid)"
                else
                    echo "==> 结束占用相机的 $prog (pid $pid)"
                fi
                kill "$pid" 2>/dev/null
                killed=1
            done
        done
        if [ "$killed" = 1 ]; then
            printf "==> 等待相机释放"
            for i in $(seq 1 12); do
                sleep 0.5
                usb_still_held || { printf " 就绪\n"; break; }
                printf "."
                # 前 3 秒给 SIGTERM 自己收尾的机会，之后强杀
                if [ "$i" = 6 ]; then
                    for prog in $CAMERA_USERS; do
                        for pid in $(pids_of "$prog"); do kill -9 "$pid" 2>/dev/null; done
                    done
                fi
            done
            sleep 1  # USB 层余量
        fi
        ;;
esac

# GUI 程序需要 DISPLAY。SSH 会话里没有，指向小电脑本机的 X。
# Ubuntu 跑 Wayland 时 GNOME 通过 Xwayland 提供 :0，认证文件在 mutter 那份。
#
# 显示号不能写死 :0：x86 小电脑跑 Wayland，Xwayland 占 :0；
# Jetson 跑 Xorg，gdm 登录后桌面在 :1（/tmp/.X11-unix/X1）。
# 所以取 /tmp/.X11-unix 下第一个 X<n> 套接字作为显示号。
if [ -z "${DISPLAY:-}" ] && [ "$(uname)" != Darwin ]; then
    XSOCK="$(ls /tmp/.X11-unix/ 2>/dev/null | grep -E '^X[0-9]+$' | sort -V | head -1)"
    export DISPLAY=":${XSOCK#X}"
    [ "$DISPLAY" = ":" ] && export DISPLAY=:0
    if [ -z "${XAUTHORITY:-}" ]; then
        # Wayland(Xwayland) 的认证文件在 mutter 那份；Xorg 的在 gdm 那份
        XAUTH="$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* /run/user/$(id -u)/gdm/Xauthority 2>/dev/null | head -1)"
        [ -n "$XAUTH" ] && export XAUTHORITY="$XAUTH"
    fi
    echo "==> SSH 会话，已设 DISPLAY=$DISPLAY（窗口出现在小电脑的显示器上）"
fi

# 必须在产物目录里运行：程序内部用 ../configs/xxx.yaml 这类相对路径，
# 在别处跑会找不到配置直接 exit(1)。
cd "$ROOT/$BUILD_DIR"

# 海康 MVS 运行环境。VS Code 任务 / ssh 直接执行命令都是非登录 shell，
# 读不到 /etc/profile.d 和 ~/.bashrc 里 MVS 安装器写的变量，
# 结果海康枚举返回 0x80000006（传输层插件找不到）→"未探测到任何相机"。
# 这里兜底补上：x86 在 /opt/MVS/lib/64，Jetson 在 /opt/MVS/lib/aarch64。
if [ -d /opt/MVS/lib ]; then
    export MVCAM_SDK_PATH="${MVCAM_SDK_PATH:-/opt/MVS}"
    export MVCAM_COMMON_RUNENV="${MVCAM_COMMON_RUNENV:-/opt/MVS/lib}"
    export MVCAM_GENICAM_CLPROTOCOL="${MVCAM_GENICAM_CLPROTOCOL:-/opt/MVS/lib/CLProtocol}"
    export ALLUSERSPROFILE="${ALLUSERSPROFILE:-/opt/MVS/MVFG}"
    for d in /opt/MVS/lib/aarch64 /opt/MVS/lib/64; do
        [ -d "$d" ] && case ":${LD_LIBRARY_PATH:-}:" in *":$d:"*) ;; *) export LD_LIBRARY_PATH="$d${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}";; esac
    done
fi

# ---- 断开 SSH / 拔网线后程序继续跑 ----
# 直接 exec 的话程序挂在这个 SSH 会话上，网线一拔 / WiFi 一断，sshd 关掉会话，
# 程序收到 SIGHUP 就被杀了 —— 车正在小陀螺，自瞄突然没了。
# 所以 SSH 会话里改成放进 screen 后台会话 "aim"：
#   - 终端照常实时看日志（attach 着），按 Ctrl+C 照常结束程序
#   - 拔网线 / 关 VS Code / WiFi 断：程序不受影响继续跑
#   - 重新连上后看回日志：bash scripts/aim.sh attach   (退出查看但不停程序: Ctrl+A 再按 D)
#   - 停程序：bash scripts/aim.sh stop ；下次按 Alt+N 也会自动先停旧的
# 不在 SSH 里（小电脑本机终端 / 开机自启）就照旧前台跑。
# 设 RUN_FOREGROUND=1 可强制前台（老行为）。
if [ -n "${SSH_CONNECTION:-}" ] && [ -z "${RUN_FOREGROUND:-}" ] && [ -z "${STY:-}" ] && command -v screen >/dev/null; then
    screen -S aim -X quit >/dev/null 2>&1 || true
    LOG="$ROOT/$BUILD_DIR/logs/aim_screen.log"
    mkdir -p "$(dirname "$LOG")"
    : > "$LOG"
    # 显式把 DISPLAY/XAUTHORITY 带进 screen，窗口仍出现在小电脑显示器上
    # 环境变量 screen 会从当前 shell 继承（DISPLAY、MVS、LD_LIBRARY_PATH 都在上面 export 过了）
    screen -dmS aim -L -Logfile "$LOG" "./$TARGET" "$@"
    echo "==> 已在后台 screen 会话 'aim' 中运行 ./$TARGET $*"
    echo "==> 现在拔网线 / 断 WiFi 程序都不会停。重连后: bash scripts/aim.sh attach"
    echo
    sleep 0.3
    # 前台 attach，体验和以前一样：能看日志、Ctrl+C 停程序。
    # 会话断了这个 attach 进程会被杀，但 screen 里的程序不受影响。
    exec screen -r aim
fi

echo "==> 运行 ./$TARGET $*"
echo
exec "./$TARGET" "$@"
