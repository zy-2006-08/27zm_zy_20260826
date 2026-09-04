#!/usr/bin/env python3
"""
PlotJuggler 的替代品（mac 上没有 PlotJuggler 官方包）。

接收 tools::Plotter 通过 UDP 发到 127.0.0.1:9870 的 JSON，实时画曲线。

用法：
    # 终端1（先开这个，再跑程序）
    .venv-plot/bin/python tools/plot_viewer.py w r x vx

    # 终端2
    cd build && ./auto_aim_test

不给参数时列出所有可用的字段名，方便你挑。
"""

import json
import socket
import sys
from collections import deque

import matplotlib

matplotlib.use("macosx" if sys.platform == "darwin" else "TkAgg")
import matplotlib.pyplot as plt

PORT = 9870
WINDOW = 500  # 每条曲线保留多少个点


def discover(sock):
    """没指定字段时，先收一包看看有哪些 key。"""
    print(f"监听 udp://127.0.0.1:{PORT}，等待数据... （现在去另一个终端跑 ./auto_aim_test）")
    while True:
        raw, _ = sock.recvfrom(65536)
        try:
            keys = sorted(json.loads(raw.decode()).keys())
        except (json.JSONDecodeError, UnicodeDecodeError):
            continue
        print("\n可画的字段：")
        for k in keys:
            print(f"  {k}")
        print(f"\n例如：  .venv-plot/bin/python {sys.argv[0]} w r x vx")
        return


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", PORT))

    fields = sys.argv[1:]
    if not fields:
        discover(sock)
        return

    data = {f: deque(maxlen=WINDOW) for f in fields}

    fig, axes = plt.subplots(len(fields), 1, sharex=True, figsize=(10, 2.2 * len(fields)))
    if len(fields) == 1:
        axes = [axes]
    lines = {}
    for ax, f in zip(axes, fields):
        (lines[f],) = ax.plot([], [], lw=1.2)
        ax.set_ylabel(f)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("frame")
    fig.tight_layout()
    plt.show(block=False)

    print(f"画 {fields}，关掉窗口即退出。")
    # 非阻塞：把「此刻已到达」的包排空就立即重绘。
    # 用 settimeout(0.05) 的话 recvfrom 会一直等到超时才 break，而程序每 ~22ms
    # 发一包，于是内层循环总是跑满 200 次（≈4s）才出来，曲线变成几秒跳一次。
    sock.setblocking(False)
    n = 0

    while plt.fignum_exists(fig.number):
        # 一次把积压的包全收完，避免画图慢导致延迟越积越大
        got = False
        for _ in range(200):
            try:
                raw, _ = sock.recvfrom(65536)
            except BlockingIOError:
                break  # 队列空了，立刻去重绘
            try:
                d = json.loads(raw.decode())
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            for f in fields:
                if f in d and isinstance(d[f], (int, float, bool)):
                    data[f].append((n, float(d[f])))
            n += 1
            got = True

        if got:
            for f in fields:
                if not data[f]:
                    continue
                xs = [p[0] for p in data[f]]
                ys = [p[1] for p in data[f]]
                lines[f].set_data(xs, ys)
                ax = axes[fields.index(f)]
                ax.set_xlim(xs[0], max(xs[-1], xs[0] + 10))
                lo, hi = min(ys), max(ys)
                pad = (hi - lo) * 0.1 or 0.5
                ax.set_ylim(lo - pad, hi + pad)

        plt.pause(0.03)

    sock.close()


if __name__ == "__main__":
    main()
