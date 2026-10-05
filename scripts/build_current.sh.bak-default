#!/usr/bin/env bash
# ============================================================================
#  按「当前编辑的文件」决定编译哪个目标
# ----------------------------------------------------------------------------
#  和 run_current.sh 配套：打开 camera_tuning.cpp 按 Alt+, 只编 camera_tuning，
#  比全量 make 快得多（全量要链接 10 个可执行文件，每个都上百 MB）。
#
#  打开的是库代码（detector.cpp / camera.hpp 这类）时编默认目标 auto_aim_test，
#  因为库改动会被所有目标依赖，编一个最常用的就能验证改动是否编得过。
#
#  用法：build_current.sh <当前文件相对工程根的路径>
# ============================================================================
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REL_FILE="${1:-}"

if [ "$(uname)" = Darwin ]; then
    # Mac 侧是手写的 build.sh 单体编译脚本，没有 per-target 概念，整体编
    exec "$ROOT/mac/build.sh"
fi

DEFAULT_TARGET="auto_aim_test"

resolve_target() {
    local rel="$1"
    [ -z "$rel" ] && return 1
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
    echo "==> 编译目标：$TARGET（来自 $REL_FILE）"
else
    TARGET="$DEFAULT_TARGET"
    echo "==> ${REL_FILE:-无活动文件} 不对应特定可执行文件，编译默认目标：$TARGET"
fi

# build/ 不存在说明还没 cmake 配置过，先配一次。
# 直接 make 会报 "No such file or directory"，对新克隆的仓库不友好。
if [ ! -f "$ROOT/build/Makefile" ]; then
    echo "==> build/ 未配置，先跑 cmake"
    cmake -S "$ROOT" -B "$ROOT/build" || exit 1
fi

exec make -C "$ROOT/build" "$TARGET" -j"$(nproc)"
