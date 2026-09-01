#!/usr/bin/env python3
"""把 Docker 容器里生成的 compile_commands.json 翻译成 macOS 本机可用的版本。

为什么需要这个
--------------
`build/compile_commands.json` 是在 docker/ 那套 Linux 容器里跑 cmake 生成的，
每条记录的 "directory" 和 "file" 都是容器内路径（/workspace/...），include 路径
也指向 Linux 系统目录（/usr/include/opencv4 等）。

在 macOS 上打开工程时，clangd 会自动发现并加载它，然后报：

    E VFS: failed to set CWD to /workspace/build/tasks/auto_aim: No such file...

结果是 48 个翻译单元一个都建不起来，~/.cache/clangd/index/ 始终为空。
表现出来就是「查找所有引用」只能看到当前文件里的声明和定义，跨文件的真实
调用点全部丢失（例如 Gimbal::state() 明明被 tracker.cpp 和 rb_auto_aim_debug.cpp
调用了 4 次，却显示"2 文件中有 2 个结果"）。

做法
----
逐条重写路径与编译参数，输出到 build-mac/compile_commands.json：

  * /workspace            -> 仓库根目录
  * Linux 库 include      -> 对应的 Homebrew 前缀
  * 编译器                -> 本机 clang++（-std=gnu++17 保留，clang 支持）
  * 追加 .clangd-stubs    -> 补 linux/can.h、sys/epoll.h 这些 Linux 专属头
  * 去掉 -o/-c 与 .o 产物路径，clangd 不需要，留着反而可能触发写盘

保留容器 DB 的价值在于它带了 .clangd 里没有的 include 路径
（io/daheng、io/hikrobot、planner/tinympc、Eigen 子目录）和宏定义
（-DOPENVINO_MAKE、-DOV_THREAD=OV_THREAD_TBB），这些是 cmake 按 target
算出来的，手写容易漏。

用法
----
    python3 mac/gen_compile_commands.py

然后 .clangd 里 CompilationDatabase 指向 build-mac 即可。
容器那份 build/compile_commands.json 不动，Linux 上照旧用。
"""

from __future__ import annotations

import json
import os
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONTAINER_DB = ROOT / "build" / "compile_commands.json"
OUT_DIR = ROOT / "build-mac"
OUT_DB = OUT_DIR / "compile_commands.json"

CONTAINER_ROOT = "/workspace"


def brew_prefix(pkg: str) -> str | None:
    try:
        out = subprocess.run(
            ["brew", "--prefix", pkg],
            capture_output=True, text=True, check=True,
        )
        return out.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def build_include_map() -> dict[str, str]:
    """Linux 系统 include 目录 -> macOS Homebrew 目录。"""
    need = {
        "opencv@4": "include/opencv4",
        "eigen": "include/eigen3",
        "openvino": "include",
        "fmt": "include",
        "spdlog": "include",
        "yaml-cpp": "include",
        "nlohmann-json": "include",
        "libomp": "include",
    }
    prefix: dict[str, str] = {}
    missing = []
    for pkg, sub in need.items():
        p = brew_prefix(pkg)
        if p is None:
            missing.append(pkg)
            continue
        prefix[pkg] = str(Path(p) / sub)

    if missing:
        sys.exit(
            "缺少 Homebrew 依赖: " + " ".join(missing) + "\n"
            "执行: brew install " + " ".join(missing)
        )

    # 容器里出现过的 Linux 路径 -> 本机路径。长路径必须排在前面，
    # 否则 /usr/include 会先把 /usr/include/opencv4 截断掉。
    return {
        "/usr/include/opencv4": prefix["opencv@4"],
        "/usr/include/eigen3": prefix["eigen"],
        "/opt/intel/openvino_2024.6.0/runtime/include": prefix["openvino"],
    }


def translate_flags(args: list[str], inc_map: dict[str, str]) -> list[str]:
    out: list[str] = ["clang++"]  # 换掉容器里的 /usr/bin/c++
    skip_next = False

    for arg in args[1:]:
        if skip_next:
            skip_next = False
            continue

        # clangd 不需要产物路径；-c 后面跟的源文件由 "file" 字段提供
        if arg in ("-o", "-c"):
            skip_next = arg == "-o"
            continue
        if arg.endswith(".o"):
            continue

        # Linux 专属告警开关，clang 不认
        if arg.startswith("-Wno-error=deprecated-declarations"):
            out.append("-Wno-deprecated-declarations")
            continue

        # glibc ABI 宏在 libc++ 下无意义，留着会让部分头文件走错分支
        if arg.startswith("-D_GLIBCXX_USE_CXX11_ABI"):
            continue

        if arg.startswith("-I"):
            path = arg[2:]
            for linux_path, mac_path in inc_map.items():
                if path == linux_path or path.startswith(linux_path + "/"):
                    path = mac_path + path[len(linux_path):]
                    break
            else:
                path = path.replace(CONTAINER_ROOT, str(ROOT))
            # 容器里有 planner/tinympc/.. 这类相对片段，规范化掉
            out.append("-I" + os.path.normpath(path))
            continue

        out.append(arg.replace(CONTAINER_ROOT, str(ROOT)))

    # 补 Linux 专属内核头的桩（socketcan / epoll）
    out.append("-I" + str(ROOT / ".clangd-stubs"))
    return out


def main() -> None:
    if not CONTAINER_DB.exists():
        sys.exit(f"找不到 {CONTAINER_DB}\n先在 Linux/Docker 侧跑过 cmake -B build")

    inc_map = build_include_map()
    entries = json.loads(CONTAINER_DB.read_text())

    result = []
    dropped = []
    for e in entries:
        src = e["file"].replace(CONTAINER_ROOT, str(ROOT))
        src = os.path.normpath(src)
        if not os.path.exists(src):
            dropped.append(e["file"])
            continue

        raw = e["arguments"] if "arguments" in e else shlex.split(e["command"])
        result.append({
            "directory": str(ROOT),
            "file": src,
            "arguments": translate_flags(raw, inc_map),
        })

    OUT_DIR.mkdir(exist_ok=True)
    OUT_DB.write_text(json.dumps(result, indent=2) + "\n")

    print(f"已写入 {OUT_DB}")
    print(f"  翻译 {len(result)} 条 / 原始 {len(entries)} 条")
    if dropped:
        print(f"  跳过 {len(dropped)} 条（源文件在本机不存在）:")
        for f in dropped:
            print(f"    {f}")


if __name__ == "__main__":
    main()
