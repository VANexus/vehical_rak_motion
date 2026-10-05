#!/usr/bin/env python3
# [AI] UART v1 协议镜像测试工具(Jetson ↔ F407 底盘)
# 规范: rak 仓 docs/protocol/jetson_f407_uart_v1.md;固件对齐响应: chassis 仓 docs/UART-v1-ALIGN.md
# 用法:
#   python3 uart_v1_test.py /dev/ttyUSB1          # 识别+握手+100Hz 遥测 3s(mode=0)
#   python3 uart_v1_test.py /dev/ttyUSB1 --arm --vx 200   # ARM+前进 200mm/s 3s(务必架空!)
#   python3 uart_v1_test.py --selftest            # 只跑 golden vector 自检
import argparse, struct, time

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0]); code_pos = 0; code = 1
    for b in data:
        if b == 0:
            out[code_pos] = code; code = 1; code_pos = len(out); out.append(0)
        else:
            out.append(b); code += 1
            if code == 0xFF:
                out[code_pos] = 0xFF; code = 1; code_pos = len(out); out.append(0)
    out[code_pos] = code
    return bytes(out) + b'\x00'

def cobs_decode(wire: bytes) -> bytes:
    """COBS 解码:不含定界符;定界符由 frame_decode 先剥掉。"""
    out = bytearray(); i = 0; pending = False
    while i < len(wire):
        c = wire[i]; i += 1
        if c == 0:
            pending = False       # 帧内不应有 0,容错跳过
            continue
        if pending:
            out.append(0)
        pending = (c != 0xFF)
        n = 254 if c == 0xFF else c - 1
        out += wire[i:i + n]; i += n
    return bytes(out)

def frame_encode(msg_type: int, seq: int, payload: bytes) -> bytes:
    body = struct.pack('<BBHH', 1, msg_type, seq, len(payload)) + payload
    return cobs_encode(body + struct.pack('<H', crc16(body)))

def frame_decode(wire: bytes):
    if wire.endswith(b'\x00'):    # 剥定界符再解码(否则尾 0 会被当 code 字节)
        wire = wire[:-1]
    d = cobs_decode(wire)
    if len(d) < 8 or d[0] != 1:
        return None
    _, mtype, seq, plen = struct.unpack_from('<BBHH', d, 0)
    if len(d) != 6 + plen + 2:
        return None
    body = d[:6 + plen]
    (crc,) = struct.unpack_from('<H', d, 6 + plen)
    if crc != crc16(body):
        return None
    return mtype, seq, d[6:6 + plen]

def selftest():
    assert crc16(b'123456789') == 0x29B1, hex(crc16(b'123456789'))
    w = frame_encode(0x01, 0x1234, bytes.fromhex('00 01 00 02 03 04 05 06'))
    golden = '06 01 01 34 12 08 01 02 01 08 02 03 04 05 06 44 58 00'
    assert w.hex(' ') == golden, w.hex(' ')
    m, s, p = frame_decode(w)
    assert (m, s) == (0x01, 0x1234) and p == bytes.fromhex('00 01 00 02 03 04 05 06')
    print('[自检] CRC16 + COBS + 组帧/解码 全部通过(golden vector 一致)')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port', nargs='?')
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--sec', type=float, default=3.0)
    ap.add_argument('--vx', type=int, default=0)
    ap.add_argument('--vy', type=int, default=0)
    ap.add_argument('--wz', type=int, default=0, help='mrad/s')
    ap.add_argument('--arm', action='store_true', help='mode=1(轮子会转!先架空)')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest or not a.port:
        selftest()
        if not a.port:
            print('用法: uart_v1_test.py <port> [--arm --vx 200] [--wz 100]')
        return
    import serial
    s = serial.Serial(a.port, a.baud, timeout=0.02)
    print(f'{a.port} @ {a.baud}  arm={a.arm} vx={a.vx} vy={a.vy} wz={a.wz}')

    # 1) 等识别横幅 ID:A(帧流是二进制,按子串找,别按行首)
    buf = bytearray(); t0 = time.perf_counter(); seen_id = None
    while time.perf_counter() - t0 < 2.0 and not seen_id:
        buf += s.read(max(1, s.in_waiting))
        i = buf.find(b'ID:')
        if i >= 0:
            j = buf.find(b'\x0a', i)
            seen_id = bytes(buf[i:j if j > i else i + 8]).decode(errors='replace').strip()
        if len(buf) > 4096:
            del buf[:2048]

    # 2) HELLO 握手
    seq_out = 1
    s.write(frame_encode(0x01, seq_out, struct.pack('<II', 0x01020304, 0x3)))
    rx = bytearray(); ack = None
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 1.0 and ack is None:
        rx += s.read(max(1, s.in_waiting))   # [AI] 非阻塞:别让读饿死发送循环
        while True:
            z = rx.find(b'\x00')
            if z < 0: break
            wire = bytes(rx[:z + 1]); del rx[:z + 1]
            fr = frame_decode(wire)
            if fr and fr[0] == 0x02:
                board, fw, boot, feat = struct.unpack('<HIII', fr[2])
                ack = (board, fw, boot, feat)
    if ack is None:
        print('[握手] 失败: 1s 内无 HELLO_ACK'); return
    print(f'[握手] HELLO_ACK board=0x{ack[0]:04X} fw=1.{(ack[1]>>8)&0xFF}.{ack[1]&0xFF} '
          f'boot_id=0x{ack[2]:08X} features=0x{ack[3]:X}')

    # 3) 100Hz SET_BASE_VELOCITY + BASE_STATE 统计
    mode = 1 if a.arm else 0
    payload = struct.pack('<BHiii', mode, 50, a.vx, a.vy, a.wz)
    n_state = 0; last_state = -1; last_echo = -1; faults = 0
    t0 = time.perf_counter(); t_next = t0; show = t0
    states = {}
    try:
        while time.perf_counter() - t0 < a.sec:
            now = time.perf_counter()
            if now >= t_next:
                seq_out = seq_out % 32767 + 1
                s.write(frame_encode(0x10, seq_out, payload))
                t_next += period if (period := 0.01) else period
                if t_next < now: t_next = now + period
            rx += s.read(max(1, s.in_waiting))   # [AI] 非阻塞:别让读饿死发送循环
            while True:
                z = rx.find(b'\x00')
                if z < 0: break
                wire = bytes(rx[:z + 1]); del rx[:z + 1]
                fr = frame_decode(wire)
                if not fr: continue
                if fr[0] == 0x11:
                    echo, st, fb, mvx, mvy, mwz, up = struct.unpack('<HBIiiiI', fr[2])
                    n_state += 1; states[st] = states.get(st, 0) + 1
                    last_state, last_echo = st, echo
                    if fb: faults += 1
                elif fr[0] == 0x12:
                    print(f'  [FAULT_EVENT] bitmap=0x{struct.unpack("<I", fr[2][:4])[0]:08X}')
            if now - show >= 0.5:
                print(f'  state={last_state} echo={last_echo} fb非零帧={faults} '
                      f'BASE_STATE={n_state}帧/{now-t0:.1f}s')
                show = now
    except KeyboardInterrupt:
        pass
    dt = time.perf_counter() - t0
    s.write(frame_encode(0x10, seq_out % 32767 + 1, struct.pack('<BHiii', 0, 50, 0, 0, 0)))
    s.close()
    print(f'---- BASE_STATE {n_state}帧/{dt:.2f}s = {n_state/dt:.1f}Hz '
          f'状态分布={states} 已发 DISARM 停车帧 ----')

if __name__ == '__main__':
    main()
