#!/usr/bin/env python3
"""min_detect_count 参数扫描实验。

对每个候选值：改 yaml -> 跑 auto_aim_test -> 收 UDP 曲线 -> 统计。
统计的三个指标：
  tracking占比   越高越好（对真目标）
  重新初始化次数  越低越好（每次都让 EKF 从零重来）
  首次tracking帧  越小越好（反应速度）

用法：  .venv-plot/bin/python tools/sweep_detect_count.py 1 3 5 10 30 60
"""

import json
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
YAML = ROOT / "configs" / "demo.yaml"
BIN = ROOT / "build-mac" / "auto_aim_test"
FRAMES = 400
STATE_NAMES = {0: "lost", 1: "detecting", 2: "tracking", 3: "temp_lost"}


def collect(samples, stop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", 9870))
    sock.settimeout(0.3)
    while not stop.is_set():
        try:
            raw, _ = sock.recvfrom(65536)
        except socket.timeout:
            continue
        try:
            samples.append(json.loads(raw.decode()))
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
    sock.close()


def run_once(value):
    text = YAML.read_text()
    YAML.write_text(re.sub(r"^min_detect_count:.*$", f"min_detect_count: {value}", text, flags=re.M))

    samples, stop = [], threading.Event()
    t = threading.Thread(target=collect, args=(samples, stop), daemon=True)
    t.start()
    time.sleep(0.4)

    subprocess.run(
        [str(BIN), "-s=0", f"-e={FRAMES}"],
        cwd=BIN.parent,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env={"PATH": "/usr/bin:/bin", "DYLD_LIBRARY_PATH": "", "OPENCV_VIDEOIO_PRIORITY_AVFOUNDATION": "1"},
    )

    time.sleep(1.0)
    stop.set()
    t.join(timeout=2)
    return samples


def analyse(samples):
    states = [int(s["state"]) for s in samples if "state" in s]
    if not states:
        return None

    counts = {n: 0 for n in STATE_NAMES}
    for s in states:
        counts[s] += 1

    reinit = sum(1 for a, b in zip(states, states[1:]) if a != 0 and b == 0)
    first = next((i for i, s in enumerate(states) if s == 2), None)

    radii = [s["r"] for s in samples if "r" in s]
    r_ok = sum(1 for r in radii if 0.15 < r < 0.30)

    return {
        "n": len(states),
        "pct": {STATE_NAMES[k]: 100.0 * v / len(states) for k, v in counts.items()},
        "reinit": reinit,
        "first": first,
        "r_health": 100.0 * r_ok / len(radii) if radii else 0.0,
    }


def main():
    values = [int(v) for v in sys.argv[1:]] or [1, 3, 5, 10, 30, 60]
    backup = YAML.with_suffix(".yaml.sweepbak")
    shutil.copy(YAML, backup)

    rows = []
    try:
        for v in values:
            print(f"跑 min_detect_count={v} ...", flush=True)
            res = analyse(run_once(v))
            if res is None:
                print("  没收到数据，跳过")
                continue
            rows.append((v, res))
            p = res["pct"]
            print(
                f"  帧数{res['n']:4}  tracking{p['tracking']:5.1f}%  temp_lost{p['temp_lost']:5.1f}%  "
                f"detecting{p['detecting']:5.1f}%  lost{p['lost']:5.1f}%  "
                f"重新初始化{res['reinit']:3}次  首次tracking@{res['first']}  r健康{res['r_health']:5.1f}%",
                flush=True,
            )
    finally:
        shutil.move(backup, YAML)

    print("\n" + "=" * 100)
    print(f"{'N':>4} | {'tracking%':>9} | {'temp_lost%':>10} | {'输出目标%':>9} | {'重新初始化':>9} | {'首次tracking':>12} | {'r健康%':>7}")
    print("-" * 100)
    for v, r in rows:
        p = r["pct"]
        usable = p["tracking"] + p["temp_lost"]
        print(
            f"{v:>4} | {p['tracking']:>9.1f} | {p['temp_lost']:>10.1f} | {usable:>9.1f} | "
            f"{r['reinit']:>9} | {str(r['first']):>12} | {r['r_health']:>7.1f}"
        )
    print("=" * 100)
    print("输出目标% = tracking + temp_lost，这两个状态都会把 Target 交给上层（tracker.cpp 只挡 lost）")


if __name__ == "__main__":
    main()
