#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
看电控发来的数据 / Gimbal serial monitor

为什么需要这个工具:
  想确认"电控在不在发数据、发的对不对"时, 直接跑 rb_auto_aim_debug 很难判断 ——
  那个程序还要连相机、加载 OpenVINO 权重, 任何一环出问题都表现为"起不来",
  看不出串口这一层到底是什么状态。这个工具只做一件事: 读串口、解包、显示。

协议来源: io/gimbal/gimbal.hpp 的 GimbalToVision
  帧头 5A 53, 之后 mode(1) color(2) q[4](16) bullet_speed(4) bullet_count(2) crc16(2)

  ⚠️ 实测电控只发 29 字节, 而结构体是 37 字节 —— 电控跳过了
     gimbal_yaw / gimbal_pitch 这两个 float(结构体注释里写明本仓库不消费它们)。
     所以这里默认按 29 字节解, 可用 --frame-len 覆盖。

用法:
  python3 tools/serial_monitor.py              # 终端实时刷新
  python3 tools/serial_monitor.py --rerun      # 额外开 rerun 波形窗口
  python3 tools/serial_monitor.py --once       # 只抓 3 秒, 打印诊断报告
  python3 tools/serial_monitor.py --raw        # 显示原始十六进制
  python3 tools/serial_monitor.py --port /dev/ttyACM1 --baud 460800
"""
import argparse
import collections
import math
import os
import struct
import sys
import time

FRAME_LEN_DEFAULT = 29
HEAD = b"\x5a\x53"

MODE_NAME = {0: "空闲", 1: "自瞄", 2: "小符", 3: "大符", 4: "长焦"}


def parse_args():
    p = argparse.ArgumentParser(description="查看电控发来的数据", formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0", help="串口设备 (默认 /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=460800, help="波特率 (默认 460800, 与 gimbal.cpp 一致)")
    p.add_argument("--frame-len", type=int, default=FRAME_LEN_DEFAULT, help=f"帧长 (默认 {FRAME_LEN_DEFAULT})")
    p.add_argument("--once", action="store_true", help="只抓 3 秒, 打印一次诊断报告后退出")
    p.add_argument("--raw", action="store_true", help="同时显示原始十六进制")
    p.add_argument("--rerun", action="store_true", help="开 rerun 波形窗口(在本机显示器上)")
    p.add_argument("--rerun-save", metavar="文件", help="不开窗口, 存成 .rrd 文件供事后回放")
    p.add_argument("--rerun-fps", type=int, default=60, help="发往 rerun 的最大频率 (默认 60)")
    return p.parse_args()


def find_ports():
    """列出可能的串口, 用于设备不存在时给提示。"""
    out = []
    try:
        for f in sorted(os.listdir("/dev")):
            if f.startswith(("ttyACM", "ttyUSB")):
                out.append(os.path.join("/dev", f))
    except OSError:
        pass
    return out


def hint_no_device(port):
    print(f"\n找不到串口 {port}")
    found = find_ports()
    if found:
        print(f"但发现了这些串口: {', '.join(found)}")
        print(f"换一个试试:  python3 tools/serial_monitor.py --port {found[0]}")
    else:
        print("系统里没有任何 ttyACM* / ttyUSB* 设备。检查:")
        print("  1. 串口板 USB 线插好了吗")
        print("  2. 运行 lsusb, 看有没有 1a86:xxxx (CH340) 之类的串口芯片")


def hint_no_permission(port):
    print(f"\n没有权限打开 {port}")
    print("解决办法(二选一):")
    print("  A. 加入 dialout 组(一次性, 推荐):")
    print("       sudo usermod -aG dialout $USER")
    print("     然后【重新登录】(退出 SSH 再连) —— 组变更不会对已有终端生效")
    print("  B. 临时用 sudo:")
    print("       sudo python3 tools/serial_monitor.py")


def decode(frame):
    """按协议解一帧。"""
    d = {}
    d["mode"] = frame[2]
    d["color"] = struct.unpack("<H", frame[3:5])[0]
    d["q"] = struct.unpack("<4f", frame[5:21])
    d["qnorm"] = sum(x * x for x in d["q"]) ** 0.5
    d["bullet_speed"] = struct.unpack("<f", frame[21:25])[0]
    d["bullet_count"] = struct.unpack("<H", frame[25:27])[0]
    d["crc"] = struct.unpack("<H", frame[27:29])[0] if len(frame) >= 29 else 0
    return d


def quat_to_euler_deg(q):
    """四元数(wxyz)转 yaw/pitch/roll, 单位度。仅为直观显示, 不参与解算。"""
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return [a * 57.29577951308232 for a in (yaw, pitch, roll)]


class RerunSink:
    """
    rerun 波形输出。

    为什么单独包一层而不是直接调 rerun:
      1. 没装 rerun 时整个工具仍要能用 —— 波形只是附加功能, 不该变成硬依赖;
      2. 限流逻辑集中在一处。电控发 800Hz, 全量发给 viewer 没有意义
         (人眼看不出 800Hz 和 60Hz 的区别), 还会让 viewer 吃满内存 ——
         detect_test.cpp 的注释里记过这个坑: 按帧数限流会随帧率失控,
         所以这里按时间限流, 不管串口多快, 每秒只发固定张数。
    """

    def __init__(self, save_path=None, max_fps=60):
        self.ok = False
        self.min_dt = 1.0 / max(1, max_fps)
        self.last_t = 0.0
        try:
            import rerun as rr
        except ImportError:
            print("未装 rerun, 跳过波形。装法: pip3 install rerun-sdk")
            return

        self.rr = rr
        try:
            rr.init("gimbal_serial", spawn=False)
            if save_path:
                rr.save(save_path)
                print(f"rerun 数据将存到 {save_path}")
                print(f"事后查看:  rerun {save_path}")
            else:
                # spawn 会启动 viewer 进程并连上。
                # 已有 viewer 在跑时它只是连上去, 不会重复开窗口。
                rr.spawn(memory_limit="500MB")
                print("已启动 rerun viewer 窗口")

            # 给几路曲线预设样式, 让波形一眼能分清。
            # static=True 表示这是"一次性的元数据", 不随时间变化。
            rr.log("gimbal/yaw", rr.SeriesLines(colors=[0, 220, 80], names="yaw(度)", widths=2.0), static=True)
            rr.log("gimbal/pitch", rr.SeriesLines(colors=[255, 200, 0], names="pitch(度)", widths=2.0), static=True)
            rr.log("gimbal/roll", rr.SeriesLines(colors=[80, 160, 255], names="roll(度)", widths=2.0), static=True)
            rr.log("quat/w", rr.SeriesLines(colors=[255, 255, 255], names="w", widths=1.5), static=True)
            rr.log("quat/x", rr.SeriesLines(colors=[255, 80, 80], names="x", widths=1.5), static=True)
            rr.log("quat/y", rr.SeriesLines(colors=[80, 255, 80], names="y", widths=1.5), static=True)
            rr.log("quat/z", rr.SeriesLines(colors=[80, 80, 255], names="z", widths=1.5), static=True)
            rr.log("health/qnorm", rr.SeriesLines(colors=[255, 120, 0], names="四元数模长(应为1)", widths=2.0), static=True)
            rr.log("health/fps", rr.SeriesLines(colors=[180, 180, 180], names="帧率", widths=1.5), static=True)
            rr.log("status/mode", rr.SeriesLines(colors=[220, 100, 255], names="模式", widths=2.0), static=True)
            rr.log("status/bullet_speed", rr.SeriesLines(colors=[0, 200, 200], names="弹速(m/s)", widths=2.0), static=True)
            rr.log("status/bullet_count", rr.SeriesLines(colors=[255, 160, 160], names="发弹数", widths=2.0), static=True)
            self.ok = True
        except Exception as e:
            print(f"rerun 初始化失败: {e}")
            print("波形功能关闭, 终端显示仍然可用")

    def log(self, t_rel, d, fps):
        """t_rel: 相对开始的秒数, 作为时间轴。"""
        if not self.ok:
            return
        now = time.time()
        if now - self.last_t < self.min_dt:
            return
        self.last_t = now

        rr = self.rr
        rr.set_time("时间", duration=t_rel)

        ypr = quat_to_euler_deg(d["q"])
        rr.log("gimbal/yaw", rr.Scalars(ypr[0]))
        rr.log("gimbal/pitch", rr.Scalars(ypr[1]))
        rr.log("gimbal/roll", rr.Scalars(ypr[2]))

        for name, v in zip("wxyz", d["q"]):
            rr.log(f"quat/{name}", rr.Scalars(v))

        rr.log("health/qnorm", rr.Scalars(d["qnorm"]))
        rr.log("health/fps", rr.Scalars(fps))
        rr.log("status/mode", rr.Scalars(d["mode"]))
        rr.log("status/bullet_speed", rr.Scalars(d["bullet_speed"]))
        rr.log("status/bullet_count", rr.Scalars(d["bullet_count"]))


def main():
    a = parse_args()

    if not os.path.exists(a.port):
        hint_no_device(a.port)
        return 1

    try:
        import serial
    except ImportError:
        print("缺少 pyserial。安装:")
        print("  sudo apt install python3-serial      (推荐)")
        print("  或  pip3 install pyserial")
        return 2

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.05)
    except PermissionError:
        hint_no_permission(a.port)
        return 1
    except Exception as e:
        if "Permission denied" in str(e):
            hint_no_permission(a.port)
        else:
            print(f"\n打开 {a.port} 失败: {e}")
            print("常见原因: 串口被其他程序占用(rb_auto_aim_debug / detect_test / 另一个本工具)")
            print(f"  查占用:  sudo fuser -v {a.port}")
        return 1

    sink = None
    if a.rerun or a.rerun_save:
        sink = RerunSink(a.rerun_save, a.rerun_fps)

    print(f"已打开 {a.port} @ {a.baud}  帧长 {a.frame_len} 字节")
    print("按 Ctrl+C 退出\n")

    buf = bytearray()
    FL = a.frame_len
    total_bytes = 0
    total_frames = 0
    junk = 0
    last_show = 0.0
    last_frame = None
    fps_win = collections.deque(maxlen=200)
    t0 = time.time()
    fps = 0.0

    try:
        while True:
            chunk = ser.read(4096)
            now = time.time()

            if chunk:
                total_bytes += len(chunk)
                buf.extend(chunk)

            while True:
                i = buf.find(HEAD)
                if i < 0:
                    if len(buf) > 8192:
                        junk += len(buf) - 1
                        del buf[:-1]
                    break
                if i > 0:
                    junk += i
                    del buf[:i]
                if len(buf) < FL:
                    break
                frame = bytes(buf[:FL])
                del buf[:FL]
                total_frames += 1
                fps_win.append(now)
                last_frame = frame

                if len(fps_win) >= 2:
                    span = fps_win[-1] - fps_win[0]
                    if span > 0:
                        fps = (len(fps_win) - 1) / span
                if sink is not None:
                    sink.log(now - t0, decode(frame), fps)

            if a.once and now - t0 >= 3.0:
                break

            if not a.once and now - last_show >= 0.2:
                last_show = now
                os.system("clear")
                print(f"串口 {a.port} @ {a.baud}        按 Ctrl+C 退出")
                if sink is not None and sink.ok:
                    print("rerun 波形已开启" + (f" (存盘 {a.rerun_save})" if a.rerun_save else " (窗口)"))
                print("=" * 60)

                if total_bytes == 0:
                    print(f"\n  等待数据... 已等 {now - t0:.0f} 秒, 一个字节都没收到\n")
                    print("  如果一直是 0, 看下面几点:")
                    print("    1. 电控程序在跑吗? 它是不是要先进某个模式才发数据?")
                    print("    2. 串口板的 TX/RX 有没有接反")
                    print("    3. 电控上电了吗")
                    print(f"    4. 波特率对不对(当前 {a.baud})")
                elif total_frames == 0:
                    print(f"\n  收到 {total_bytes} 字节, 但解不出帧(找不到帧头 5A 53)\n")
                    print("  最可能是波特率不对 —— 波特率错了数据就是乱码。")
                    print(f"  当前 {a.baud}, 工程用的也是这个值(io/gimbal/gimbal.cpp:25)")
                    print("\n  原始数据前 32 字节:")
                    print("   ", " ".join(f"{b:02X}" for b in bytes(buf[:32])))
                else:
                    d = decode(last_frame)
                    ypr = quat_to_euler_deg(d["q"])
                    mode_s = MODE_NAME.get(d["mode"], f"未知({d['mode']})")
                    color_s = "红" if d["color"] == 0 else ("蓝" if d["color"] == 1 else f"?{d['color']}")

                    print(f"  帧率      {fps:6.1f} 帧/秒        累计 {total_frames} 帧")
                    print(f"  模式      {d['mode']}  {mode_s}")
                    print(f"  我方颜色  {d['color']}  {color_s}          (敌方是它的反面)")
                    print()
                    print(f"  四元数    w{d['q'][0]:+8.4f}  x{d['q'][1]:+8.4f}  y{d['q'][2]:+8.4f}  z{d['q'][3]:+8.4f}")
                    print(f"  模长      {d['qnorm']:.4f}   {'正常' if abs(d['qnorm']-1) < 0.05 else '异常! 应接近 1.0'}")
                    print(f"  换算角度  yaw{ypr[0]:+8.2f}  pitch{ypr[1]:+8.2f}  roll{ypr[2]:+8.2f}  (度)")
                    print()
                    print(f"  弹速      {d['bullet_speed']:.2f} m/s")
                    print(f"  发弹数    {d['bullet_count']}")
                    print(f"  CRC       0x{d['crc']:04X}")

                    if a.raw and last_frame is not None:
                        print()
                        print(f"  原始帧    {' '.join(f'{b:02X}' for b in last_frame)}")

                    print()
                    print("  ---- 摇一摇云台, 四元数和角度应该跟着变 ----")

                print("=" * 60)

    except KeyboardInterrupt:
        print("\n\n已退出")
    finally:
        ser.close()

    el = time.time() - t0
    print()
    print("=" * 60)
    print("诊断报告")
    print("=" * 60)
    print(f"  监听时长      {el:.1f} 秒")
    print(f"  收到字节      {total_bytes}  ({total_bytes/max(el,0.001):.0f} B/s)")
    print(f"  解出完整帧    {total_frames}  ({total_frames/max(el,0.001):.1f} 帧/秒)")
    print(f"  丢弃杂散字节  {junk}")

    if total_bytes == 0:
        print("\n  结论: 没有收到任何数据")
        print("        → 电控没在发, 或接线/波特率有问题")
        return 1
    if total_frames == 0:
        print("\n  结论: 有数据但解不出帧, 波特率大概率不对")
        return 1

    d = decode(last_frame)
    print("\n  最后一帧:")
    print(f"    mode={d['mode']}({MODE_NAME.get(d['mode'],'?')})  color={d['color']}  "
          f"|q|={d['qnorm']:.4f}  弹速={d['bullet_speed']:.1f}  发弹数={d['bullet_count']}")

    print("\n  健康检查:")
    ok = True
    if abs(d["qnorm"] - 1.0) < 0.05:
        print("    [正常] 四元数模长接近 1, 陀螺仪姿态有效")
    else:
        print(f"    [异常] 四元数模长 {d['qnorm']:.4f}, 应接近 1.0")
        print("           → 电控没正确填 q[4], solver 会算出错误的世界坐标")
        ok = False
    if d["mode"] in MODE_NAME:
        print(f"    [正常] mode={d['mode']} 取值合法")
    else:
        print(f"    [异常] mode={d['mode']} 越界, 字节偏移可能没对齐")
        ok = False
    if 0 <= d["bullet_speed"] < 40:
        print(f"    [正常] 弹速 {d['bullet_speed']:.1f} m/s 在合理范围")
    else:
        print(f"    [异常] 弹速 {d['bullet_speed']:.1f} m/s 超出正常范围(0~30)")
        ok = False
    if junk > total_frames * FL * 0.1:
        print(f"    [注意] 杂散字节偏多({junk}), 帧长可能不是 {FL}, 用 --frame-len 试试别的值")

    if a.rerun_save:
        print(f"\n  波形已存: {a.rerun_save}")
        print(f"  查看:     rerun {a.rerun_save}")

    print()
    print("  结论: " + ("串口通信正常, 电控数据可用" if ok else "能收到数据, 但内容有问题, 见上面的[异常]项"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
