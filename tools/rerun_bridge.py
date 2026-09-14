#!/usr/bin/env python3
"""UDP(9870) -> Rerun 桥接。

C++ 侧不用改：tools::Plotter 发的 JSON 原样收下，转成 Rerun 的 Scalars。

用法：
    # 终端1（先开）
    ~/.venvs/rerun/bin/python tools/rerun_bridge.py

    # 终端2
    cd build-mac && ./auto_aim_test

指定字段（量纲相近的放一起最好看）：
    ~/.venvs/rerun/bin/python tools/rerun_bridge.py x y z r l h
    ~/.venvs/rerun/bin/python tools/rerun_bridge.py gimbal_yaw cmd_yaw a

默认按量纲自动分图；想全部挤一张图加 --one。
"""

import json
import os
import socket
import sys
import time

import rerun as rr
import rerun.blueprint as rrb

PORT = 9870

VIEWER = os.path.expanduser(
    "~/.venvs/rerun/lib/python3.14/site-packages/rerun_sdk/"
    "rerun_cli/Rerun.app/Contents/MacOS/Rerun")

COLORS = [
    [255, 90, 90], [90, 170, 255], [130, 230, 130], [255, 200, 60],
    [200, 130, 255], [255, 140, 200], [120, 220, 220], [230, 180, 120],
    [160, 200, 255], [255, 110, 110], [180, 255, 180], [220, 220, 120],
]

# 按量纲分组：角度(±57)和位置(±0.5)混在一张图里，大的会把小的压成直线
GROUPS = [
    ("angle_deg", ["a", "gimbal_yaw", "cmd_yaw", "armor_yaw", "armor_yaw_raw"]),
    ("position_m", ["x", "y", "z", "r", "l", "h", "armor_x", "armor_y"]),
    ("velocity", ["vx", "vy", "vz", "w"]),
    ("ekf_check", ["nis", "nees", "residual_yaw", "residual_pitch",
                   "residual_distance", "residual_angle",
                   "recent_nis_failures", "nis_fail", "nees_fail"]),
]


def group_of(field):
    for name, members in GROUPS:
        if field in members:
            return name
    return "other"


def build_blueprint(fields, single):
    values = rrb.TextDocumentView(name="Values", origin="/status")

    if single:
        plots = rrb.TimeSeriesView(
            name="Waveforms", origin="/",
            contents=[f"+ /plot/{f}" for f in fields],
            plot_legend=rrb.PlotLegend(corner=rrb.Corner2D.RightTop),
        )
    else:
        views = []
        for name in [g[0] for g in GROUPS] + ["other"]:
            members = [f for f in fields if group_of(f) == name]
            if members:
                views.append(rrb.TimeSeriesView(
                    name=name, origin="/",
                    contents=[f"+ /plot/{f}" for f in members],
                    plot_legend=rrb.PlotLegend(corner=rrb.Corner2D.RightTop),
                ))
        plots = rrb.Grid(*views) if len(views) > 1 else views[0]

    return rrb.Blueprint(
        rrb.Horizontal(plots, values, column_shares=[5, 1]),
        collapse_panels=True,
    )


def main():
    wanted = [a for a in sys.argv[1:] if not a.startswith("--")]
    single = "--one" in sys.argv

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", PORT))
    sock.settimeout(1.0)

    print(f"监听 udp://127.0.0.1:{PORT} ... 现在去另一个终端跑 ./auto_aim_test")

    while True:
        try:
            raw, _ = sock.recvfrom(65536)
        except socket.timeout:
            print("  还没收到数据...")
            continue
        try:
            first = json.loads(raw.decode())
            break
        except (json.JSONDecodeError, UnicodeDecodeError):
            continue

    rr.init("auto_aim")
    rr.spawn(executable_path=VIEWER if os.path.exists(VIEWER) else None)

    fields = []
    n = 0
    t0 = time.monotonic()

    def register(d):
        """EKF 那些量只在 targets 非空时才发，所以字段要边收边补，不能只看第一包。"""
        new = [k for k, v in d.items()
               if isinstance(v, (int, float, bool))
               and k not in fields
               and (not wanted or k in wanted)]
        if not new:
            return
        for f in new:
            rr.log(f"plot/{f}",
                   rr.SeriesLines(colors=COLORS[len(fields) % len(COLORS)], names=f),
                   static=True)
            fields.append(f)
        # make_active：不加的话 viewer 沿用上次缓存的布局，新字段不会出现
        rr.send_blueprint(build_blueprint(fields, single),
                          make_active=True, make_default=True)
        print(f"字段（{len(fields)}）: {' '.join(fields)}")

    def handle(d):
        nonlocal n
        register(d)
        rr.set_time("frame", sequence=n)
        rr.set_time("t", duration=time.monotonic() - t0)
        for f in fields:
            v = d.get(f)
            if isinstance(v, (int, float, bool)):
                rr.log(f"plot/{f}", rr.Scalars(float(v)))
        rr.log("status", rr.TextDocument(
            "\n".join(f"{f:<22}: {float(d[f]):+.4f}"
                      for f in fields if isinstance(d.get(f), (int, float, bool)))))
        n += 1

    handle(first)
    sock.setblocking(False)

    try:
        while True:
            got = False
            for _ in range(500):
                try:
                    raw, _ = sock.recvfrom(65536)
                except BlockingIOError:
                    break
                try:
                    handle(json.loads(raw.decode()))
                    got = True
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
            if not got:
                time.sleep(0.005)
    except KeyboardInterrupt:
        print(f"\n收到 {n} 帧，退出。")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
