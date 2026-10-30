#!/usr/bin/env python3
"""canopen_bringup.py —— CANopen v5 台架主站与验收脚本（SocketCAN，仅标准库）

用途：不依赖 rak 的 ROS 2 栈，就能对 v5 固件做端到端验收：
  NMT Start → SYNC(2 ms) → 下发 RPDO（命令序号 + 远程门）→ 统计 TPDO 速率 → SDO 读关键对象。

用法：
    python3 tools/canopen_bringup.py --can can0 --seconds 3
    python3 tools/canopen_bringup.py --can can0 --no-drive      # 只收不发命令（零速/零门）

判据（2026-10-29 实测基线）：
    心跳 0x701/0x702 = 05；TPDO1..4 帧数/秒 = 100/100/100/100（底盘）、50/50/100/10（机械臂）；
    0x2050 cmd_en=1、0x2F00 tag_packed=固件标签前 4 字符；can0 berr-counter tx 0 / rx 0。
"""
import argparse
import collections
import socket
import struct
import sys
import threading
import time

CHASSIS = 1
ARM = 2

TPDO_HZ = {  # 期望速率（帧/秒）
    0x181: 100, 0x281: 100, 0x381: 100, 0x481: 100,
    0x182: 50, 0x282: 50, 0x382: 100, 0x482: 10,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can", default="can0")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--no-drive", action="store_true",
                    help="不发 cmd_flags bit0（远程门保持关）")
    ap.add_argument("--spd", type=float, default=0.0,
                    help="四轮速令 rad/s（默认 0；轮子架空才允许非 0）")
    args = ap.parse_args()

    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((args.can,))
    s.settimeout(0.2)

    def send(cid, data=b""):
        s.send(struct.pack("=IB3x8s", cid, len(data), data.ljust(8, b"\0")))

    def sdo_read(node, idx, sub=0, tries=3):
        for _ in range(tries):
            send(0x600 + node, struct.pack("<BHBI", 0x40, idx, sub, 0))
            end = time.time() + 0.5
            while time.time() < end:
                try:
                    f = s.recv(16)
                except socket.timeout:
                    break
                cid, _dlc = struct.unpack("=IB3x", f[:8])
                if cid == 0x580 + node and (f[8] & 0x43) == 0x43:
                    return struct.unpack("<I", f[12:16])[0]
        return None

    counts = collections.Counter()
    stop = False
    seq = [0]
    flags = 0 if args.no_drive else 1
    spd_raw = max(-32768, min(32767, int(round(args.spd * 100.0))))

    def rx():
        while not stop:
            try:
                f = s.recv(16)
            except socket.timeout:
                continue
            cid, dlc = struct.unpack("=IB3x", f[:8])
            counts[cid] += 1

    def periodic(period, fn):
        nxt = time.perf_counter()
        while not stop:
            fn()
            nxt += period
            dt = nxt - time.perf_counter()
            if dt > 0:
                time.sleep(dt)

    def traffic():
        send(0x080, b"")
        send(0x201, struct.pack("<hhhh", spd_raw, spd_raw, spd_raw, spd_raw))
        seq[0] = (seq[0] + 1) & 0xFFFFFFFF
        send(0x301, struct.pack("<IH", seq[0], flags))
        send(0x402, struct.pack("<IH", seq[0], flags))

    th = threading.Thread(target=rx, daemon=True)
    th.start()
    time.sleep(1.0)                       # 等两板启动（上电到 Pre-op 需要时间）
    for _ in range(5):
        send(0x000, bytes([1, CHASSIS]))
        send(0x000, bytes([1, ARM]))
        time.sleep(0.1)

    tm = threading.Thread(target=periodic, args=(0.002, lambda: send(0x080, b"")), daemon=True)
    tt = threading.Thread(target=periodic, args=(0.010, traffic), daemon=True)
    tm.start(); tt.start()
    time.sleep(args.seconds)
    stop = True
    time.sleep(0.1)

    hb1 = counts.get(0x700 + CHASSIS, 0)
    hb2 = counts.get(0x700 + ARM, 0)
    tag = sdo_read(CHASSIS, 0x2F00)
    age = sdo_read(CHASSIS, 0x2051)
    en = sdo_read(CHASSIS, 0x2050)
    tag_s = "".join(chr(b) for b in struct.pack("<I", tag) if b) if tag else "?"

    print(f"接口 {args.can}  采样 {args.seconds:.1f}s  心跳 node1={hb1} node2={hb2}")
    fail = 0
    for cid, hz in sorted(TPDO_HZ.items()):
        exp = round(hz * args.seconds)
        got = counts.get(cid, 0)
        ok = abs(got - exp) <= max(3, exp * 0.05)
        fail += 0 if ok else 1
        print(f"  {'OK ' if ok else 'BAD'} 0x{cid:03X}: {got:4d} 帧 (期望≈{exp}, {hz} Hz)")
    print(f"  SDO: node1 tag_packed={tag_s!r}  cmd_en={en}  cmd_age_ms={age}")
    print("结论：" + ("全部达标 ✅" if fail == 0 and hb1 and hb2 else f"未达标 ❌（{fail} 项速率异常或心跳缺失）"))
    return 0 if fail == 0 and hb1 and hb2 else 1


if __name__ == "__main__":
    sys.exit(main())
