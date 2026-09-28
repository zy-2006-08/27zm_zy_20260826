#!/usr/bin/env python3
"""FIREPROBE 统计: 开火瞬间装甲板偏离正对位置的角度 vs 自转角速度.

用法: python3 scripts/fireprobe_stats.py build/logs/<某次>.log

原理: 开火时刻如果时间对齐正确, 被打的那块板应当正对我方, face 约为 0,
且与转速无关. 若 face 随 vyaw 线性变化, 斜率就是时间误差:
    face(rad) = -vyaw * tau
    tau > 0  -> 开火偏早(提前量过多), 应减小 laser_latency 约 tau
    tau < 0  -> 开火偏晚(提前量不足), 应增大 laser_latency 约 |tau|
只统计 |vyaw| >= 1.57 rad/s (反陀螺分支) 的 CMD 行.
"""
import collections
import math
import re
import statistics as st
import sys

PAT = re.compile(
    r"FIREPROBE (\w+) face=([-\d.]+) id=(-?\d+) yaw_err=([-\d.]+) "
    r"pitch_err=([-\d.]+) vyaw=([-\d.]+) r=([-\d.]+)")


def main(path):
    rows = []
    for line in open(path, errors="ignore"):
        m = PAT.search(line)
        if m and m.group(1) == "CMD":
            face, _, ye, pe, vy, r = (float(x) for x in m.groups()[1:])
            rows.append((face, ye, pe, vy, r))

    spin = [x for x in rows if abs(x[3]) >= 1.57]
    print(f"CMD 共 {len(rows)} 条, 其中小陀螺(|vyaw|>=1.57) {len(spin)} 条")
    if not spin:
        return

    buckets = collections.defaultdict(list)
    for x in spin:
        buckets[round(x[3])].append(x)
    print(f"{'vyaw':>5} {'n':>4} {'face均值':>8} {'face标准差':>9} {'yaw_err':>8} {'pitch_err':>9}")
    for k in sorted(buckets):
        v = buckets[k]
        f = [a[0] for a in v]
        print(f"{k:5d} {len(v):4d} {st.mean(f):8.1f} {st.pstdev(f):9.1f} "
              f"{st.mean(a[1] for a in v):8.2f} {st.mean(a[2] for a in v):9.2f}")

    # 过原点最小二乘: face_rad = -vyaw * tau
    num = sum(-a[3] * math.radians(a[0]) for a in spin)
    den = sum(a[3] * a[3] for a in spin)
    tau = num / den
    print(f"\n拟合时间误差 tau = {tau * 1e3:+.1f} ms  "
          f"({'开火偏早, 减小' if tau > 0 else '开火偏晚, 增大'} laser_latency 约 {abs(tau) * 1e3:.1f} ms)")


if __name__ == "__main__":
    main(sys.argv[1])
