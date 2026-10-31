#!/usr/bin/env python3
"""日志摘要: 把一份几万行的日志压成一屏中文摘要。

只读日志, 原文件一个字不改, 照样可以整份丢给 AI 分析。

VS Code 里不用敲命令:
  * 看最新一次: 顶上菜单「终端」->「运行任务...」-> 选「看最新日志摘要」
  * 看某一份:   在左边文件列表里打开 build/logs 下的那个 .log, 按 Alt+N
  两种都会在编辑器里打开 build/日志摘要.txt。
命令行: python3 scripts/log_summary.py [日志文件] [--open]
  不给文件 = build/logs 里最新的一份; --open = 另存成 build/日志摘要.txt 并在 VS Code 里打开

统计部分在同目录的 log_stats.py。只用标准库, 兼容 Python 3.8 (Jetson 小电脑上的 Python 可能比较旧)。
"""
from __future__ import annotations

import shutil
import subprocess
import sys
import unicodedata
from datetime import timedelta
from pathlib import Path
from typing import Callable

sys.dont_write_bytecode = True  # 不在 scripts/ 下生成 __pycache__, 免得 git 里多出没用的文件
from log_stats import Avg, Report, Window, read_log  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
LOG_DIR = ROOT / "build" / "logs"
SUMMARY_FILE = ROOT / "build" / "日志摘要.txt"
STEPS = (10, 15, 20, 30, 60, 120, 300, 600)  # 时间表每行几秒: 挑最小的、让表不超过 20 行的那个


def width(s: str) -> int:
    """终端里的显示宽度: 汉字占两格。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def pad(s: str, n: int) -> str:
    return s + " " * max(0, n - width(s))


def cut(s: str, n: int) -> str:
    """超过 n 格就截断, 免得一行折成好几行。"""
    out = ""
    for c in s:
        if width(out + c) > n:
            return out + "…"
        out += c
    return out


def pct(a: int, b: int) -> str:
    return f"{100 * a / b:.0f}%" if b else "-"


def num(x: float | None) -> str:
    return "-" if x is None else f"{x:.1f}"


def mean_and_peak(r: Report, total: Window, pick: Callable[[Window], Avg], unit: str) -> str:
    """平均值, 加上按 1 秒平均后的最大值。"""
    peaks = [m for m in (pick(w).mean() for w in r.seconds.values()) if m is not None]
    if not peaks:
        return "-"
    return f"平均 {num(pick(total).mean())} {unit}，最高 {max(peaks):.1f} {unit}（按 1 秒平均）"


def overview(r: Report, total: Window) -> list[str]:
    """【整体】每项一行。"""
    run = r.duration() - r.main_start if r.main_start is not None else 0.0
    det = pct(total.det_hit, total.det_total)
    if total.det_total:
        det += f"（最低的 2 秒只有 {r.det_min}%，低于 50% 的 2 秒有 {r.det_low} 次）"
    sent = pct(total.sent_on, total.sent)
    if total.sent:
        sent += f" 的时间在控制云台（mode 不为 0）；完全没控的 2 秒有 {r.sent_zero} 次"
    fire = f"视觉下令 {total.fires} 次，电控回传打出 {r.shots} 发"
    if total.fires and not r.shots:
        fire += "（电控没回传弹数，或者没装弹）"
    rows = [
        ("帧率", f"平均 {total.frames / run:.0f} 帧/秒" if total.frames and run > 0 else "-"),
        ("识别率", det),
        ("角点精修", pct(r.refine_ok, r.refine_total) + (" 成功" if r.refine_total else "")),
        ("控制云台", sent),
        ("模式切换", f"{r.mode_switches} 次"),
        ("开火", fire),
        ("相机卡顿", f"{total.lags} 次（两帧之间隔了 0.1 秒以上，跟踪会重来）"),
        ("pitch 限幅", f"{r.pitch_clamps} 次（视觉要的角度超出机械行程时会被限幅）"),
        ("目标距离", mean_and_peak(r, total, lambda w: w.dist, "米")),
        ("敌方转速", mean_and_peak(r, total, lambda w: w.vyaw, "rad/s")),
        ("敌方车速", mean_and_peak(r, total, lambda w: w.speed, "m/s")),
    ]
    return [f"  {pad(k, 11)}{v}" for k, v in rows]


def hints(w: Window, span: float) -> str:
    """一段时间里值得注意的情况。"""
    out = []
    if w.det_total and w.det_hit < 0.5 * w.det_total:
        out.append("识别率低")
    elif w.det_total and w.sent and w.sent_on < 0.2 * w.sent:
        out.append("认到了却没控云台")
    if w.frames and w.frames < 40 * span:
        out.append("帧率低")
    if w.lags:
        out.append(f"相机卡顿 {w.lags} 次")
    return "、".join(out)


def timeline(r: Report) -> list[str]:
    """【按时间看】每段一行, 一眼看出哪一段出了问题。"""
    dur = int(r.duration()) + 1
    step = next((s for s in STEPS if dur / s <= 20), STEPS[-1])
    cols = [("开始", 10), ("帧率", 6), ("识别率", 8), ("控云台", 8), ("开火", 6), ("距离m", 7), ("转速", 6), ("车速", 6)]
    lines = [f"【按时间看】（每 {step} 秒一行；转速单位 rad/s，车速单位 m/s）",
             "  " + "".join(pad(name, n) for name, n in cols) + "提示"]
    assert r.first is not None
    for start in range(0, dur, step):
        w = Window()
        for s in range(start, min(start + step, dur)):
            w.merge(r.seconds.get(s, Window()))
        if not (w.frames or w.det_total or w.sent):
            continue
        span = min(step, dur - start)
        cells = [
            (r.first + timedelta(seconds=start)).strftime("%H:%M:%S"),
            f"{w.frames / span:.0f}" if w.frames else "-",
            pct(w.det_hit, w.det_total),
            pct(w.sent_on, w.sent),
            str(w.fires),
            num(w.dist.mean()),
            num(w.vyaw.mean()),
            num(w.speed.mean()),
        ]
        lines.append("  " + "".join(pad(c, n) for c, (_, n) in zip(cells, cols)) + hints(w, span))
    return lines if len(lines) > 2 else [*lines, "  （这份日志里没有处理过任何一帧）"]


def notes(r: Report) -> list[str]:
    """启动阶段的提示、运行中的警告、最后几行。"""
    out: list[str] = []
    if r.main_start is None:
        out += ["", "!! 程序一帧都没处理就结束了，原因多半在下面的「最后 5 行」里。"]
    if r.startup:
        out += ["", "【启动阶段的提示】（处理第一帧之前）"] + [f"  {cut(s, 110)}" for s in r.startup[:12]]
        out += [f"  ……还有 {len(r.startup) - 12} 条"] if len(r.startup) > 12 else []
    if r.warnings:
        out += ["", "【运行中的警告和错误】（同一种合并计数，具体数字换成了 N）"]
        out += [f"  {n:>6} 次  {cut(s, 100)}" for s, n in r.warnings.most_common(10)]
    out += ["", "【最后 5 行】（看程序是怎么结束的）"] + [f"  {cut(s, 110)}" for s in r.tail]
    return out


def render(r: Report, path: Path) -> list[str]:
    """拼出整份摘要。"""
    total = Window()
    for w in r.seconds.values():
        total.merge(w)
    dur = r.duration()
    assert r.first is not None and r.last is not None
    try:
        shown = path.resolve().relative_to(ROOT)
    except ValueError:
        shown = path
    head = [
        "=" * 64,
        f"日志摘要  {path.name}",
        f"时间  {r.first:%Y-%m-%d %H:%M:%S} 到 {r.last:%H:%M:%S}，共 {int(dur // 60)} 分 {int(dur % 60)} 秒",
        "=" * 64,
        "",
        "【整体】",
    ]
    tail = [
        "",
        f"原始日志（要给 AI 分析就用这份）: {shown}",
        "下次看最新一次: 顶上菜单「终端」→「运行任务...」→「看最新日志摘要」；看某一份: 打开那个 .log 按 Alt+N",
    ]
    return head + overview(r, total) + [""] + timeline(r) + notes(r) + tail


def pick_log(argv: list[str]) -> Path:
    """有参数就用参数给的文件; 没有就挑 build/logs 里最新的一份(按修改时间)。"""
    if argv:
        p = Path(argv[0])
        return p if p.exists() or p.is_absolute() else ROOT / p
    logs = sorted(LOG_DIR.glob("[0-9][0-9][0-9][0-9]-*.log"), key=lambda p: p.stat().st_mtime)
    if not logs:
        sys.exit(f"{LOG_DIR} 里还没有日志, 先跑一次程序。")
    return logs[-1]


def open_in_editor(text: str) -> None:
    """把摘要存成 build/日志摘要.txt, 再用 VS Code 的 code 命令在编辑器里打开它。"""
    SUMMARY_FILE.write_text(text + "\n", encoding="utf-8")
    code = shutil.which("code")
    try:
        opened = code is not None and subprocess.run([code, "-r", str(SUMMARY_FILE)], timeout=10).returncode == 0
    except subprocess.TimeoutExpired:
        opened = False
    where = "已经在编辑器里打开" if opened else "在左边文件列表的 build 文件夹里，双击打开"
    print(f"\n摘要也存成了 build/{SUMMARY_FILE.name}，{where}。")


def main(argv: list[str]) -> int:
    """选日志 -> 读 -> 打印摘要; 带 --open 时再存成文件并在编辑器里打开。"""
    path = pick_log([a for a in argv if a != "--open"])
    try:
        report = read_log(path)
    except FileNotFoundError:
        print(f"找不到日志文件: {path}")
        return 1
    if report.first is None:
        print(f"{path} 里没有带时间的日志行, 可能不是程序日志。")
        return 1
    text = "\n".join(render(report, path))
    print(text)
    if "--open" in argv:
        open_in_editor(text)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
