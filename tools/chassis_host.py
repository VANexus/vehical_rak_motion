#!/usr/bin/env python3
# [AI] 底盘 Python 控制类 —— UART v1 协议(与 ROS2 host 同款)，交互式开车用
#
# 用法一(交互式 REPL,推荐):
#   python -i chassis_host.py COM6
#   >>> car.forward(200)      # 前进 200mm/s
#   >>> car.back(150)         # 后退
#   >>> car.rotate(100)       # 原地左自转(100 mrad/s ≈ 5.7°/s)
#   >>> car.rotate(-100)      # 右自转
#   >>> car.turn(200, 50)     # 弧线:前进 200 + 左转
#   >>> car.strafe(150)       # 左横移(麦轮专属)
#   >>> car.stop()            # 速度清零(保持 ARM)
#   >>> car.status()          # 状态/实测速度/故障位
#   >>> car.close()           # 退出(自动 DISARM)
#
# 用法二(脚本):
#   from chassis_host import Chassis
#   with Chassis('COM6') as car:
#       car.forward(200); time.sleep(2); car.stop()
#
# 安全: 后台线程以 100Hz 持续发帧(lease 50ms);线程一停,板子 50ms 内自动
#       DISARM 刹车。close()/Ctrl+C 自动发 DISARM。速度限幅 vx/vy≤500, wz≤1000。
import struct, threading, time, sys

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

def frame_encode(msg_type: int, seq: int, payload: bytes) -> bytes:
    body = struct.pack('<BBHH', 1, msg_type, seq, len(payload)) + payload
    return cobs_encode(body + struct.pack('<H', crc16(body)))

class Chassis:
    VX_MAX, WZ_MAX = 500, 1000          # mm/s, mrad/s 软限幅
    LEASE_MS = 200   # [AI] R17 台架暂用 200(规范 50,待 EMI 整治后改回)

    def __init__(self, port, baud=921600):
        import atexit
        import serial
        self._s = serial.Serial(port, baud, timeout=0.02)
        self._seq = 1
        self._lock = threading.Lock()
        self._armed = False
        self._vx = self._vy = self._wz = 0
        self._rx = bytearray()
        self.state = dict(state='?', fault=0, mvx=0, mvy=0, mwz=0, echo=0, n_fb=0)
        self._run = threading.Event(); self._run.set()
        self._last_rehello = 0.0
        self._handshake(verbose=True)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        atexit.register(self.close)     # [AI] 任何退出路径(含 Ctrl+C)都停线程+DISARM

    # ---- 控制接口 ----
    def set_velocity(self, vx=0, vy=0, wz=0, arm=True):
        """vx/vy: mm/s(+X前 +Y左); wz: mrad/s(+逆时针)。arm=False 只清零并 DISARM。"""
        vx = max(-self.VX_MAX, min(self.VX_MAX, vx))
        vy = max(-self.VX_MAX, min(self.VX_MAX, vy))
        wz = max(-self.WZ_MAX, min(self.WZ_MAX, wz))
        with self._lock:
            self._vx, self._vy, self._wz = vx, vy, wz
            self._armed = bool(arm)

    def forward(self, speed=200):  self.set_velocity(vx=speed)
    def back(self, speed=150):     self.set_velocity(vx=-speed)
    def strafe(self, speed=150):   self.set_velocity(vy=speed)          # +左
    def rotate(self, wz=100):      self.set_velocity(wz=wz)             # +逆时针
    def turn(self, vx=200, wz=50): self.set_velocity(vx=vx, wz=wz)      # 弧线
    def stop(self):                self.set_velocity(0, 0, 0, arm=True)
    def disarm(self):              self.set_velocity(0, 0, 0, arm=False)

    def move_mm(self, dist_mm, speed=200):
        """按里程走:dist_mm>0 前进 / <0 后退,走完自动停(时间近似,先用速度闭环)。"""
        t = abs(dist_mm) / max(50, abs(speed))
        self.set_velocity(vx=speed if dist_mm >= 0 else -abs(speed))
        try:
            time.sleep(t)
        finally:
            self.stop()
        return round(t, 2)

    def status(self):
        return dict(self.state)

    # ---- 协议 ----
    def _send(self, mtype, payload):
        self._seq = self._seq % 32767 + 1
        self._s.write(frame_encode(mtype, self._seq, payload))

    def _handshake(self, verbose=False):
        self._s.reset_input_buffer()
        self._send(0x01, struct.pack('<II', 0x01020304, 0x3))
        deadline = time.time() + 1.0
        while time.time() < deadline:
            self._rx += self._s.read(max(1, self._s.in_waiting))
            while True:
                z = self._rx.find(b'\x00')
                if z < 0: break
                wire = bytes(self._rx[:z + 1]); del self._rx[:z + 1]
                fr = self._decode(wire)
                if fr and fr[0] == 0x02:
                    board, fw, boot, feat = struct.unpack('<HIII', fr[2])
                    if feat & 0x3 != 0x3:
                        raise RuntimeError('固件 features 缺 bit0/bit1')
                    self.board = dict(board=board, fw=f'1.{(fw>>8)&0xFF}.{fw&0xFF}', boot=boot)
                    if verbose:
                        print(f"[Chassis] 握手 OK: board=0x{board:04X} fw={self.board['fw']}")
                    return
        raise RuntimeError('1s 内无 HELLO_ACK')

    def _decode(self, wire):
        if wire.endswith(b'\x00'): wire = wire[:-1]
        out = bytearray(); i = 0; pending = False
        while i < len(wire):
            c = wire[i]; i += 1
            if c == 0: pending = False; continue
            if pending: out.append(0)
            pending = (c != 0xFF)
            n = 254 if c == 0xFF else c - 1
            out += wire[i:i + n]; i += n
        d = bytes(out)
        if len(d) < 8 or d[0] != 1: return None
        _, mtype, seq, plen = struct.unpack_from('<BBHH', d, 0)
        if len(d) != 6 + plen + 2: return None
        (crc,) = struct.unpack_from('<H', d, 6 + plen)
        if crc != crc16(d[:6 + plen]): return None
        return mtype, seq, d[6:6 + plen]

    def _loop(self):
        period = 0.01
        t_next = time.perf_counter()
        div = 0
        while self._run.is_set():
            now = time.perf_counter()
            if now >= t_next:
                with self._lock:
                    vx, vy, wz, armed = self._vx, self._vy, self._wz, self._armed
                if armed:
                    mode = 1
                else:
                    mode, vx, vy, wz = 0, 0, 0, 0
                self._send(0x10, struct.pack('<BHiii', mode, self.LEASE_MS, vx, vy, wz))
                t_next += period
                if t_next < now: t_next = now + period
            self._rx += self._s.read(max(1, self._s.in_waiting))
            while True:
                z = self._rx.find(b'\x00')
                if z < 0: break
                wire = bytes(self._rx[:z + 1]); del self._rx[:z + 1]
                fr = self._decode(wire)
                if not fr: continue
                if fr[0] == 0x11:
                    echo, st, fb, mvx, mvy, mwz, up = struct.unpack('<HBIiiiI', fr[2])
                    self.state = dict(state=['BOOT', 'DISARM', 'ARM', 'FAULT'][min(st, 3)],
                                      fault=fb, mvx=mvx, mvy=mvy, mwz=mwz,
                                      echo=echo, up_ms=up, n_fb=self.state['n_fb'] + 1)
                elif fr[0] == 0x12:
                    print(f'[FAULT_EVENT] bitmap=0x{struct.unpack("<I", fr[2][:4])[0]:08X}')
            div += 1
            st = self.state
            if (armed and st['state'] == 'FAULT' and (st['fault'] & 2) != 0
                    and now - self._last_rehello > 1.0):
                # [AI] 电机干扰打断 lease(>50ms)→板子锁存 bit1。失安全处理:
                # 先刹停+DISARM,重新 HELLO 清锁存;不自动重发速度,等用户重下指令。
                self._last_rehello = now
                with self._lock:
                    self._vx = self._vy = self._wz = 0
                    self._armed = False
                print('\n[Chassis] 电机干扰触发 lease 超时,已自动刹停;重新调用动作函数继续')
                try:
                    self._handshake(verbose=False)
                except Exception:
                    pass
            spare = t_next - time.perf_counter()
            if spare > 0: time.sleep(spare)

    def close(self):
        if not self._run.is_set(): return
        self._run.clear()
        try:
            for _ in range(10):                # 刹停+DISARM
                self._send(0x10, struct.pack('<BHiii', 0, self.LEASE_MS, 0, 0, 0))
                time.sleep(0.005)
        except Exception:
            pass
        self._thread.join(timeout=0.5)
        self._s.close()
        print('\n[Chassis] 已 DISARM 关闭')

    def __enter__(self): return self
    def __exit__(self, *a): self.close()

def main():
    global car                          # [AI] 交互模式:把 car 放进全局,REPL 才能访问
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM6'
    car = Chassis(port)
    print('--- 交互模式: car.forward(200)/car.move_mm(300)/car.rotate(100)/'
          'car.stop()/car.status()/car.close()   200=速度mm/s, 300=距离mm ---')
    banner = '''
def demo():
    car.forward(200); time.sleep(2); car.stop()   # 前进 2 秒
# 直接敲: car.forward(200)   停: car.stop()   退出: car.close()'''
    print(banner)

if __name__ == '__main__':
    main()
