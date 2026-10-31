"""读一份日志, 按秒统计帧数、识别率、云台指令、开火、距离、转速和警告。

给 log_summary.py 用, 自己不单独运行。只用标准库, 兼容 Python 3.8。
"""
from __future__ import annotations

import re
from collections import Counter, deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

ANSI = re.compile(r"\x1b\[[0-9;]*m")  # 终端颜色码, screen 录下来的 aim_screen.log 里有
HEAD = re.compile(r"^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\] \[(\w+)\] (.*)")
FRAME = re.compile(r"^(fps::|\[\d+\] yolo: )")  # 每处理一帧打一条: 主程序 fps::, 离线测试 [帧号] yolo:
DETECT = re.compile(r"近2秒 (\d+) 帧, 有目标 (\d+) 帧")
REFINE = re.compile(r"角点精修 (\d+)/(\d+) 成功")
SENT = re.compile(r"近2秒发出 (\d+) 帧")
SENT_ON = re.compile(r"\((\d+) 帧\)")
DIST = re.compile(r" d=([-\d.]+)")
VYAW = re.compile(r" vyaw=([-\d.]+)")
SPEED = re.compile(r" v=([-\d.]+)")
NUMBER = re.compile(r"(^|[^A-Za-z_])-?\d+(\.\d+)?")
LEVELS = {"warning": "警告", "error": "错误", "critical": "严重"}
PERIODIC = ("[识别率]", "[Gimbal] 近2秒")  # 每 2 秒一条的汇总, 单独统计, 不算进警告


@dataclass
class Avg:
    """平均值累加器。逐个往里加数, 所以是可变的。"""

    total: float = 0.0
    n: int = 0

    def add(self, x: float) -> None:
        self.total += x
        self.n += 1

    def merge(self, other: Avg) -> None:
        self.total += other.total
        self.n += other.n

    def mean(self) -> float | None:
        return self.total / self.n if self.n else None


@dataclass
class Window:
    """一段时间里的累计量。先按 1 秒一格记, 输出时再合并成更长的段, 所以是可变的。"""

    frames: int = 0
    det_total: int = 0
    det_hit: int = 0
    sent: int = 0
    sent_on: int = 0
    fires: int = 0
    lags: int = 0
    dist: Avg = field(default_factory=Avg)
    vyaw: Avg = field(default_factory=Avg)
    speed: Avg = field(default_factory=Avg)

    def merge(self, o: Window) -> None:
        self.frames += o.frames
        self.det_total += o.det_total
        self.det_hit += o.det_hit
        self.sent += o.sent
        self.sent_on += o.sent_on
        self.fires += o.fires
        self.lags += o.lags
        self.dist.merge(o.dist)
        self.vyaw.merge(o.vyaw)
        self.speed.merge(o.speed)


@dataclass
class Report:
    """整份日志的累计结果。读日志时逐行往里加, 所以是可变的。"""

    first: datetime | None = None
    last: datetime | None = None
    main_start: float | None = None  # 处理第一帧的时刻(秒), 在这之前算启动阶段
    seconds: dict[int, Window] = field(default_factory=dict)
    det_min: int = 100
    det_low: int = 0
    refine_ok: int = 0
    refine_total: int = 0
    sent_zero: int = 0
    mode_switches: int = 0
    shots: int = 0
    pitch_clamps: int = 0
    warnings: Counter[str] = field(default_factory=Counter)
    startup: list[str] = field(default_factory=list)
    tail: deque[str] = field(default_factory=lambda: deque(maxlen=5))

    def duration(self) -> float:
        return (self.last - self.first).total_seconds() if self.first and self.last else 0.0


def add_abs(avg: Avg, pattern: re.Pattern[str], msg: str) -> None:
    """从 msg 里找 pattern 的数, 取绝对值加进 avg。"""
    m = pattern.search(msg)
    if m:
        avg.add(abs(float(m.group(1))))


def add_periodic(r: Report, w: Window, msg: str) -> None:
    """处理每 2 秒一条的汇总行: 识别率、角点精修、云台指令。"""
    if "近2秒" not in msg:
        return
    det, refine, sent = DETECT.search(msg), REFINE.search(msg), SENT.search(msg)
    if msg.startswith("[识别率]") and det:
        total, hit = int(det.group(1)), int(det.group(2))
        w.det_total += total
        w.det_hit += hit
        pct = 100 * hit // total if total else 0
        r.det_min = min(r.det_min, pct)
        r.det_low += pct < 50
    elif refine:
        r.refine_ok += int(refine.group(1))
        r.refine_total += int(refine.group(2))
    elif msg.startswith("[Gimbal] 近2秒") and sent:
        w.sent += int(sent.group(1))
        on = SENT_ON.search(msg)
        if on:
            w.sent_on += int(on.group(1))
        else:
            r.sent_zero += 1


def add_line(r: Report, t: datetime, level: str, msg: str) -> None:
    """处理一行带时间的日志, 按内容记到对应的计数里。"""
    if r.first is None:
        r.first = t
    r.last = t
    sec = (t - r.first).total_seconds()
    w = r.seconds.setdefault(int(sec), Window())
    if FRAME.match(msg):
        w.frames += 1
        r.main_start = sec if r.main_start is None else r.main_start
    elif msg.startswith("RAWPROBE n=") and " i=0 " in msg:
        add_abs(w.dist, DIST, msg)
    elif msg.startswith("PROBE "):
        add_abs(w.vyaw, VYAW, msg)
        add_abs(w.speed, SPEED, msg)
    elif msg.startswith("FIREPROBE CMD"):
        w.fires += 1
    elif msg.startswith("FIREPROBE SHOT"):
        r.shots += 1
    elif msg.startswith("[Gimbal] mode "):
        r.mode_switches += 1
    else:
        add_periodic(r, w, msg)
    w.lags += "Large dt" in msg
    r.pitch_clamps += msg.startswith("[pitch] 越界")
    if level in LEVELS and not msg.startswith(PERIODIC):
        note = f"[{LEVELS[level]}] {msg}"
        if r.main_start is None:
            r.startup += [] if note in r.startup else [note]
        else:
            r.warnings[NUMBER.sub(r"\1N", note)] += 1


def read_log(path: Path) -> Report:
    """逐行读日志, 把关心的数累加进 Report。"""
    r = Report()
    with path.open(encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = (ANSI.sub("", raw) if "\x1b" in raw else raw).rstrip()
            if line:
                r.tail.append(line)
            m = HEAD.match(line)
            if m:
                add_line(r, datetime.fromisoformat(m.group(1)), m.group(2), m.group(3))
    return r
