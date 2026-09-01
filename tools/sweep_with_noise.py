#!/usr/bin/env python3
"""在"有误识别"的条件下扫 min_detect_count。

demo.avi 本身很干净，YOLO 几乎不误识别，所以 min_detect_count 的抗噪作用测不出来。
这里把 min_confidence 调低，让 YOLO 开始把背景/反光认成装甲板，
再看不同 min_detect_count 能挡住多少假目标。

判据：
  假目标被放行 = 出现了 tracking，但紧接着很快重新初始化（真目标不会这样）
  短命跟踪段   = 一段 tracking 只持续几帧就崩回 lost，几乎都是假目标造成的

用法：  .venv-plot/bin/python tools/sweep_with_noise.py
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
SHORT_RUN = 8


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


def patch_yaml(detect_count, confidence):
    text = YAML.read_text()
    text = re.sub(r"^min_detect_count:.*$", f"min_detect_count: {detect_count}", text, flags=re.M)
    text = re.sub(r"^min_confidence:.*$", f"min_confidence: {confidence}", text, flags=re.M)
    YAML.write_text(text)


def run_once(detect_count, confidence):
    patch_yaml(detect_count, confidence)
    samples, stop = [], threading.Event()
    t = threading.Thread(target=collect, args=(samples, stop), daemon=True)
    t.start()
    time.sleep(0.4)
    subprocess.run(
        [str(BIN), "-s=0", f"-e={FRAMES}"],
        cwd=BIN.parent,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env={"PATH": "/usr/bin:/bin"},
    )
    time.sleep(1.0)
    stop.set()
    t.join(timeout=2)
    return samples


def analyse(samples):
    states = [int(s["state"]) for s in samples if "state" in s]
    if not states:
        return None

    usable = sum(1 for s in states if s in (2, 3))
    reinit = sum(1 for a, b in zip(states, states[1:]) if a != 0 and b == 0)

    runs, cur = [], 0
    for s in states:
        if s in (2, 3):
            cur += 1
        elif cur:
            runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)

    armor_counts = [s.get("armor_num", 0) for s in samples if "armor_num" in s]
    return {
        "n": len(states),
        "usable": 100.0 * usable / len(states),
        "reinit": reinit,
        "runs": len(runs),
        "short_runs": sum(1 for r in runs if r < SHORT_RUN),
        "longest": max(runs) if runs else 0,
        "avg_armors": sum(armor_counts) / len(armor_counts) if armor_counts else 0,
    }


def main():
    confidences = [0.8, 0.3, 0.1]
    detect_counts = [1, 2, 3, 5, 8]
    backup = YAML.with_suffix(".yaml.noisebak")
    shutil.copy(YAML, backup)

    table = {}
    try:
        for conf in confidences:
            for dc in detect_counts:
                res = analyse(run_once(dc, conf))
                if res is None:
                    continue
                table[(conf, dc)] = res
                print(
                    f"conf={conf:<4} N={dc:<3} 平均识别板数{res['avg_armors']:4.2f}  "
                    f"输出目标{res['usable']:5.1f}%  跟踪段{res['runs']:3}段(其中短命{res['short_runs']:3})  "
                    f"最长{res['longest']:3}帧  重新初始化{res['reinit']:3}",
                    flush=True,
                )
    finally:
        shutil.move(backup, YAML)

    print("\n" + "=" * 96)
    print(f"{'conf':>5} {'N':>3} | {'平均板数':>8} | {'输出目标%':>9} | {'跟踪段数':>8} | {'短命段':>7} | {'最长段':>7} | {'重新初始化':>9}")
    print("-" * 96)
    for (conf, dc), r in table.items():
        print(
            f"{conf:>5} {dc:>3} | {r['avg_armors']:>8.2f} | {r['usable']:>9.1f} | {r['runs']:>8} | "
            f"{r['short_runs']:>7} | {r['longest']:>7} | {r['reinit']:>9}"
        )
    print("=" * 96)
    print(f"短命段 = 持续不到 {SHORT_RUN} 帧的跟踪段，基本都是假目标或抖动造成的")
    print("平均板数上升 = min_confidence 降低后 YOLO 确实开始误识别了")


if __name__ == "__main__":
    main()
