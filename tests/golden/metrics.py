#!/usr/bin/env python3
"""对比两次 golden 回放: python3 tests/golden/metrics.py <dirA> <dirB> [bullet|laser]

按工况统计开火率(工况用 A 目录的第 6 列 vyaw、第 7 列 v 分类, 两边用同一套分类):
  st  静止       v<0.3 且 |vyaw|<0.6
  ls  慢陀螺     0.6<=|vyaw|<2.0 且 v<0.5
  hs  高速陀螺   |vyaw|>=2.0 且 v<0.5
  tr  纯平移     v>=0.3 且 |vyaw|<0.6
  ts  边转边移   |vyaw|>=0.6 且 v>=0.5
"""
import os
import sys

CLASSES = ["st", "ls", "hs", "tr", "ts"]
NAMES = {"st": "静止", "ls": "慢陀螺", "hs": "高速陀螺", "tr": "纯平移", "ts": "边转边移"}


def classify(vyaw, v):
    w = abs(vyaw)
    if w >= 0.6 and v >= 0.5:
        return "ts"
    if w >= 2.0:
        return "hs"
    if w >= 0.6:
        return "ls"
    if v >= 0.3:
        return "tr"
    return "st"


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 8:
                continue
            rows.append((float(p[0]), float(p[1]), int(p[4]), float(p[5]), float(p[6])))
    return rows


def new_acc():
    return {c: {"n": 0, "fa": 0, "fb": 0, "ea": 0, "eb": 0, "diff": 0, "dyaw": 0.0} for c in CLASSES}


def compare(a, b, acc):
    n = min(len(a), len(b))
    pa = pb = 0
    for i in range(n):
        ra, rb = a[i], b[i]
        c = classify(ra[3], ra[4])
        s = acc[c]
        s["n"] += 1
        s["fa"] += ra[2]
        s["fb"] += rb[2]
        if ra[2] and not pa:
            s["ea"] += 1
        if rb[2] and not pb:
            s["eb"] += 1
        pa, pb = ra[2], rb[2]
        if ra[2] != rb[2]:
            s["diff"] += 1
        s["dyaw"] = max(s["dyaw"], abs(ra[1] - rb[1]))
    return len(a), len(b)


def row(name, s):
    n = max(s["n"], 1)
    return "%-10s %8d  %6.2f%% -> %6.2f%% (%+6.2f)  沿 %6d -> %6d  不同 %6.2f%%  |dyaw|max %.4f" % (
        name, s["n"], 100.0 * s["fa"] / n, 100.0 * s["fb"] / n, 100.0 * (s["fb"] - s["fa"]) / n,
        s["ea"], s["eb"], 100.0 * s["diff"] / n, s["dyaw"])


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    da, db = sys.argv[1], sys.argv[2]
    mode = sys.argv[3] if len(sys.argv) > 3 else "bullet"
    files = sorted(f for f in os.listdir(da) if f.endswith("." + mode + ".txt"))
    if not files:
        print("没有 %s 文件: %s" % (mode, da))
        sys.exit(1)
    total = new_acc()
    print("模式 %s   A=%s   B=%s" % (mode, da, db))
    print("%-10s %8s  %-28s %-20s %-12s" % ("工况", "周期", "开火占比 A -> B", "开火上升沿", "fire 不同"))
    for f in files:
        pb = os.path.join(db, f)
        if not os.path.exists(pb):
            print("B 缺文件: %s" % f)
            sys.exit(1)
        acc = new_acc()
        la, lb = compare(load(os.path.join(da, f)), load(pb), acc)
        print("== %s  (行数 A=%d B=%d)" % (f, la, lb))
        for c in CLASSES:
            if acc[c]["n"]:
                print("  " + row(NAMES[c], acc[c]))
            for k in total[c]:
                if k == "dyaw":
                    total[c][k] = max(total[c][k], acc[c][k])
                else:
                    total[c][k] += acc[c][k]
    print("== 合计")
    allc = new_acc()["st"]
    for c in CLASSES:
        print("  " + row(NAMES[c], total[c]))
        for k in allc:
            allc[k] = max(allc[k], total[c][k]) if k == "dyaw" else allc[k] + total[c][k]
    print("  " + row("全部", allc))


if __name__ == "__main__":
    main()
