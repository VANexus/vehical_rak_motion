# CAN 通信协议 v4（两端对接规范，规范性文档）

适用范围：Jetson（`rak`，Orin Nano + SN65HVD230）↔ 底盘 F407（`vehical_rak_chassis`）。
**这是两端的通信事实来源**：MCU 侧编解码、Jetson 侧编解码、以及本文的测试向量三者必须逐字节一致。任何一方改协议，必须先改 [`protocol-signals.yaml`](protocol-signals.yaml) 并重跑 `tools/can_check.py`。

> **v4 相对 v3 的定位变化（2026-10-09 已定，不再开洞）**
> 1. **CAN 是唯一主链路**：所有控制与遥测（安全门、4 轮速指令、7 轴舵机、全量遥测）全部走 CAN；**USB v2 串口降级为备用链路**，帧格式冻结不变（见 §10）。
> 2. **单节点拓扑**：机械臂板已烧毁退役，v3 的底盘↔机械臂互锁层（`XFER_*`）整体作废。
> 3. **控制契约沿用 ros2-control 架构**：4 轮速 + 7 轴（6 关节 + 1 夹爪）位置，麦轮 IK 在 Jetson 侧（见 `vehical_rak_chassis/docs/ROS2-CONTROL-ARCH.md`）。

术语：**必须**=强制；**应**=强烈建议，偏离需写明理由；**可**=可选。

---

## 1. 物理层与拓扑

- 单总线，**CAN 2.0A（11 bit ID）**，**1 Mbps**，经典帧 DLC=8。**禁用 CAN FD**（F407 bxCAN 不支持；Jetson mttcan 支持但必须保持关闭，`ip link` 不得带 `fd` 参数）。
- 两个节点，**总线两个物理端点各 120 Ω**（Waveshare SN65HVD230 模块板载一颗，确认焊点闭合；底盘板端应补一颗）。
- **两个节点 GND 必须互联**（第三根线）：差分接收器的共模参考。缺地时表现为"偶发收得到、大量 bus-off"的临界总线——2026-10-09 联调实测。
- 底盘侧：板载收发器，**PD0=CAN1_RX、PD1=CAN1_TX**（`docs/board-c30d.md` §2）。1 Mbps 位时序 = PCLK1 42 MHz / BRP2 / 21 tq（BS1=16, BS2=4）= 精确 1.000 Mbps，采样点 81.0 %。**时钟必须来自 HSE 8 MHz**（用 HSI 会出现偶发 bus-off 玄学故障）。
- Jetson 侧：mttcan（`c310000.mttcan`，50 MHz 时钟）+ 外置 **SN65HVD230**（3.3 V 器件，可直连 Orin）。
  推荐配置：`ip link set can0 type can bitrate 1000000 sample-point 0.810 restart-ms 100`（采样点对齐底盘的 81 %，restart-ms 自动从 bus-off 恢复）。
- ⚠️ **接线极性是历史事故点**：模块丝印与底盘端子标注可能不一致。上电先量隐性电平（H≈L≈2.5 V），通不上第一件事对调 H/L 试一次（2026-10-09 的根因就是对调后立即全通）。

## 2. 帧约定：raw8 与 e2e（v4 的关键简化）

v3 给**所有**帧加应用层 counter+CRC8。v4 按 CAN 最佳实践收敛为两种布局：

| 布局 | DLC | 内容 | 适用 |
| --- | --- | --- | --- |
| `raw8` | 8 | byte0..7 = 信号（小端、紧凑、总位宽 64 bit） | 除安全帧外的所有帧 |
| `e2e` | 8 | byte0..5 信号（48 bit）+ byte6 `[7:4]counter\|[3:0]flags` + byte7 CRC8 | **仅 SAFETY_CMD** |

理由：

1. CAN 2.0 帧本身带 **15 bit HW CRC + ACK + 自动重传**，对指令/遥测帧再加应用层 CRC 是伪保护（错误若穿透 HW CRC，大概率是系统性的，应用 CRC 也救不了），还把每帧载荷从 8 字节压缩到 6 字节。
2. `SAFETY_CMD` 是唯一"损坏会造成物理后果"的帧（急停/启动），保留 **AUTOSAR E2E Profile 1 同构**的 4 bit counter + CRC8 + Data ID 种子。
3. 指令流的"活性"不靠 counter，靠 **deadman 超时**（§6.2）；CAN 硬件保证帧不重复、不丢失（重传语义）。

**CRC8（仅 e2e 帧）**：多项式 0x07，无反射、无最终异或，**初值 = 本帧 Data ID**（不是 CAN ID！），覆盖 byte0..6：

```c
uint8_t crc8(const uint8_t *d, uint8_t len, uint8_t data_id) {
    uint8_t crc = data_id;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}
```

**字节序一律 Intel / 小端**。物理量↔整数换算一律 `round(物理值 / factor)`，饱和截断到范围端点，不取模、不回绕；两端各写"越界即拒绝发送并计数"的检查。

## 3. 标识符规划（优先级即功能）

11 bit ID 按"越小优先级越高"。**安全/管理帧 ID < 0x100 < 指令帧 < 反馈帧**，急停永远抢得到总线（`can_check.py` 强制校验此规则）。

| ID | 名称 | 方向 | 周期 | Data ID | 布局 |
| --- | --- | --- | --- | --- | --- |
| `0x010` | `SAFETY_CMD` | Jetson→广播 | 事件 | 0x01 | e2e |
| `0x020` | `EMCY` | 底盘→广播 | 事件（10 ms 内首发，重发 3 次） | 0x02 | raw8 |
| `0x030` | `NODE_INFO` | 底盘→广播 | 上电 1 次 + 1 Hz | 0x03 | raw8 |
| `0x100` | `DRIVE_CMD` | Jetson→底盘 | 100 Hz | 0x04 | raw8 |
| `0x110` | `ARM_CMD_A` (J1-J4) | Jetson→底盘 | 50 Hz | 0x05 | raw8 |
| `0x111` | `ARM_CMD_B` (J5,J6,夹爪) | Jetson→底盘 | 50 Hz | 0x06 | raw8 |
| `0x200` | `WHEEL_POS_A` (W1,W2) | 底盘→Jetson | 100 Hz | 0x07 | raw8 |
| `0x201` | `WHEEL_POS_B` (W3,W4) | 底盘→Jetson | 100 Hz | 0x08 | raw8 |
| `0x202` | `WHEEL_SPD` (W1-W4) | 底盘→Jetson | 100 Hz | 0x09 | raw8 |
| `0x210` | `SERVO_STATE_A` (J1-J4) | 底盘→Jetson | 50 Hz | 0x0A | raw8 |
| `0x211` | `SERVO_STATE_B` (J5,J6,夹爪) | 底盘→Jetson | 50 Hz | 0x0B | raw8 |
| `0x220` | `DIAG` | 底盘→Jetson | 10 Hz | 0x0C | raw8 |

### 带宽核算（1 Mbps，经典帧含填充约 110–130 µs/帧）

| 方向 | 帧率 | 来源 |
| --- | --- | --- |
| 下行 | 100 + 2×50 = 200 fps | DRIVE 100Hz + ARM 2×50Hz |
| 上行 | 3×100 + 2×50 + 10 + 1 = 361 fps | 轮位×2 + 轮速 + 舵机×2 + DIAG + NODE_INFO |
| 合计 | **~561 fps ≈ 总线负载 7 %** | 死区余量 >10 倍，未来扩展（IMU、电流估计）有空间 |

## 4. 帧信号表

| 帧 | byte0-1 | byte2-3 | byte4-5 | byte6-7 |
| --- | --- | --- | --- | --- |
| `0x100 DRIVE_CMD` | `W1SPD` | `W2SPD` | `W3SPD` | `W4SPD` |
| `0x110 ARM_CMD_A` | `J1ANG` | `J2ANG` | `J3ANG` | `J4ANG` |
| `0x111 ARM_CMD_B` | `J5ANG` | `J6ANG` | `GRIP` | 保留=0 |
| `0x200 WHEEL_POS_A` | `W1POS` (i32) | | `W2POS` (i32) | |
| `0x201 WHEEL_POS_B` | `W3POS` (i32) | | `W4POS` (i32) | |
| `0x202 WHEEL_SPD` | `W1SPD` | `W2SPD` | `W3SPD` | `W4SPD` |
| `0x210 SERVO_STATE_A` | `J1ANG` | `J2ANG` | `J3ANG` | `J4ANG` |
| `0x211 SERVO_STATE_B` | `J5ANG` | `J6ANG` | `GRIP` | 保留=0 |
| `0x220 DIAG` | `BAT_MV` | `FAULT` | `STATE` | `MODE` u8 + `BUSOFF_CNT` u8 |
| `0x030 NODE_INFO` | `PROTO` u8 + `HWREV` u8 | `BUILD` u16 | `FAULT` | `STATE` |
| `0x020 EMCY` | `FAULT` | `AUX` | `STATE` | 保留=0 |
| `0x010 SAFETY_CMD` | `CMD` u16（低 4 位有效） | 保留=0 | 保留=0 | counter/flags=0 + CRC |

## 5. 信号定义与量化

| 信号 | 位宽 | factor | 范围 | 单位 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `WxSPD` | 16 int16 | **0.01** | ±327.67 | **rad/s** | 轮角速度（前转正）。覆盖 ±327 rad/s，分辨率 10 mrad/s ≈ 0.33 mm/s 轮面速度（r=0.0325 m），远低于 25 kHz 闭环的死区 |
| `WxPOS` | 32 int32 | 1 | 全范围 | counts | 编码器累计位置，**上电=0**，里程计唯一来源。int32 在最大转速下回绕周期 > 年级，无需回绕处理 |
| `JxANG`/`GRIP` | 16 uint16 | **0.1** | 0…2700 (0…270°) | deg | 舵机角。0.1° 对齐舵机 PWM 物理分辨率（7.4 µs/° ≈ 0.07°），0.01° 是伪精度；全部无符号（0-270° 行程，Home 位见 §6.4） |
| `BAT_MV` | 16 uint16 | 1 | 0…65535 | mV | 电池电压。**欠压阈值 10500 mV**（实测定的硬约束） |
| `FAULT` | 16 uint16 | — | 见 §7 | — | 0 = 无故障 |
| `STATE` | 16 uint16 | — | 位图 | — | `bit0 estop_latched`、`bit1 remote_running`、`bit2 fault_active`、`bit3 cmd_stale`、`bit4 bat_uv`、`bit5 can_recovering`、bit6-15 保留 |
| `MODE` | 8 uint8 | — | mode 枚举 | — | 0 IDLE / 1 REMOTE / 2 ESTOP / 3 FAULT |
| `BUSOFF_CNT` | 8 uint8 | 1 | 0…255（饱和） | — | bus-off 累计计数，健康度指标 |
| `PROTO`/`HWREV`/`BUILD` | 8/8/16 | — | — | — | 协议版本（本文=4）/ 硬件版本 / git short hash 折 16 位 |

轮序 `W1..W4` 与编码器通道 `TIM2/3/4/5` 一一对应（`app_car.c` 单点定义）；关节序 `J1..J6` 对应舵机通道 `ch0..ch5`（固件轴序 J1..J6+夹爪 → ch5..ch0，`app_car.h` 单点定义）。

## 6. 网络管理、模式机与安全动作

### 6.1 `SAFETY_CMD`（ID 0x010，e2e 帧广播）

`CMD` 低 4 位有效：

| 值 | 命令 | 效果 |
| --- | --- | --- |
| 0x1 | `START` | IDLE/FAULT(已复位)→REMOTE（前提 estop 已清） |
| 0x2 | `STOP` | 受控减速到 IDLE（可 START 恢复） |
| 0x4 | `EMERGENCY_STOP` | **立即锁存急停**：25 kHz ISR 下一周期（≤40 µs）执行四轮动态刹车 |
| 0x5 | `CLEAR_ESTOP` | **规范保留**。⚠️ 固件（R43 起生产语义）**故意不实现 CAN 清除**——清急停必须**物理短按按钮**（确认人在车边）。收到即丢弃 |
| 0x6 | `CLEAR_FAULT` | 故障复位（仅当故障条件已消失） |
| 0x7 | `SOFT_RESET` | `NVIC_SystemReset`；急停锁存**跨复位保持** |

- **急停路径不经过任务调度**：`0x010` 在 CAN 接收中断里校验 CRC（DID 种子）→ 置标志 → 25 kHz ISR ≤40 µs 执行刹车。全链路响应预算 **≤1 ms**。
- **CRC 校验失败 / CMD 非法**：丢弃并计数，绝不部分执行。

### 6.2 deadman 与超时

| 监视方 | 被监视 | 时限 | 超时动作 |
| --- | --- | --- | --- |
| 底盘 | `DRIVE_CMD` 流 | **100 ms** | 轮速目标清零，受控减速停 → IDLE，`STATE.bit3 cmd_stale=1` |
| 底盘 | `ARM_CMD_A/B` 流 | **100 ms** | **舵机保持当前位置**（不回 Home、不断使能） |
| Jetson | `DIAG`/`NODE_INFO` 流 | **200 ms** | 判节点失联：冻结 ros2_control 输出、发 `STOP`、UI 报警 |

上电行为（强制）：**IDLE、电机不驱动、舵机直接输出 Home 安全位**（R44 语义：500/2166/2500/500/500/500 µs = 指令 0/225/270/0/0/0°），发一帧 `NODE_INFO`。电机启动只有显式 `START` + `DRIVE_CMD` 路径。

### 6.3 模式机

```
IDLE --START--> REMOTE --STOP/断流--> IDLE
  |                |
  +--ESTOP(锁存)---+--ESTOP(锁存)      ESTOP --按钮短按--> IDLE
  |                |
  +--FAULT--+ <----+                    FAULT --CLEAR_FAULT(条件消失)--> IDLE
```

### 6.4 与 USB v2 的语义对照（迁移表）

| USB v2 能力 | CAN v4 对应 | 差异说明 |
| --- | --- | --- |
| HELLO/feature 协商 | 无需（协议版本由 `NODE_INFO.PROTO` 静态声明） | CAN 无会话概念 |
| 序号窗口/重传 | 无需（CAN HW 重传） | |
| lease 看门狗 200 ms | `DRIVE_CMD` deadman 100 ms | CAN 周期流更密，超时更严 |
| `DRIVE_COMMAND` 4×int32 urad/s | `DRIVE_CMD` 4×int16 ×0.01 rad/s | 精度 10 mrad/s 足够；Jetson 侧负责换算 |
| `ARM_COMMAND` 7×int32 µrad | `ARM_CMD_A/B` 7×uint16 ×0.1° | 对齐舵机物理分辨率，Jetson 侧换算 |
| 遥测 DRIVE_STATE 100 Hz | `WHEEL_POS_A/B` + `WHEEL_SPD` @100 Hz | 位置 int32 counts，语义等价 |
| 遥测 SERVO/诊断 | `SERVO_STATE_A/B` @50 Hz + `DIAG` @10 Hz | |
| 安全三命令 | `SAFETY_CMD` | CAN 版 e2e 保护 |

## 7. 故障码（`FAULT_CODE`，uint16）

| 码 | 名称 | 触发条件 | 动作 |
| --- | --- | --- | --- |
| 0x0000 | NONE | — | — |
| 0x0010 | ENC_STALL | 占空比 >50 % 持续 500 ms 而编码器增量 <阈值 | FAULT，停该通道 |
| 0x0011 | ENC_JUMP | 单 1 ms 窗速度突变量 >2× 上限 | FAULT |
| 0x0012 | ENC_NO_SIGNAL | 上电自检四路编码器计数全零 | FAULT，禁止 REMOTE |
| 0x0020 | CAN_BUS_OFF | bxCAN 进入 bus-off | 自动重入网（restart-ms），累计计数入 `BUSOFF_CNT` |
| 0x0030 | BAT_UV | 电池 <**10500 mV**（实测硬约束） | 禁止 ARMED 轮命令（安全层拒绝），`STATE.bit4=1` |
| 0x0031 | BAT_OV | >16 V | FAULT |
| 0x0040 | LOOP_OVERRUN | 25 kHz ISR 占用 >70 % 或丢拍 | FAULT（实时性红线） |
| 0x0060 | ESTOP_LATCHED | 急停锁存中 | 见 §6.1 |

## 8. 一致性测试向量（两端必须复现）

单一来源：[`protocol-signals.yaml`](protocol-signals.yaml)。下表由 [`../tools/can_check.py`](../tools/can_check.py) 参考实现**实际编码得出**（`--update` 已回写，幂等）：

| # | 帧 | ID | 输入 | 8 字节（hex） |
| --- | --- | --- | --- | --- |
| V1 | `SAFETY_CMD` | `0x010` | CMD=EMERGENCY_STOP(4)、counter=5 | `04 00 00 00 00 00 50 1D` |
| V2 | `EMCY` | `0x020` | fault=0x30、aux=1、state=0x0005 | `30 00 01 00 05 00 00 00` |
| V3 | `NODE_INFO` | `0x030` | proto=4、hw=1、build=0xABCD、state=0x0001 | `04 01 CD AB 00 00 01 00` |
| V4 | `DRIVE_CMD` | `0x100` | W1=+5.00、W2=−2.00、W3=+5.00、W4=−2.00 rad/s | `F4 01 38 FF F4 01 38 FF` |
| V5 | `ARM_CMD_A` | `0x110` | J1=0°、J2=225°、J3=270°、J4=0°（Home 位） | `00 00 CA 08 8C 0A 00 00` |
| V6 | `ARM_CMD_B` | `0x111` | J5=0°、J6=0°、GRIP=90° | `00 00 00 00 84 03 00 00` |
| V7 | `WHEEL_POS_A` | `0x200` | W1=100000、W2=−50000 counts | `A0 86 01 00 B0 3C FF FF` |
| V8 | `WHEEL_POS_B` | `0x201` | W3=123456789、W4=−1 counts | `15 CD 5B 07 FF FF FF FF` |
| V9 | `WHEEL_SPD` | `0x202` | W1=W2=1.20、W3=W4=−0.80 rad/s | `78 00 78 00 B0 FF B0 FF` |
| VA | `SERVO_STATE_A` | `0x210` | J1=0°、J2=225°、J3=270°、J4=0° | `00 00 CA 08 8C 0A 00 00` |
| VB | `SERVO_STATE_B` | `0x211` | J5=0°、J6=0°、GRIP=90° | `00 00 00 00 84 03 00 00` |
| VC | `DIAG` | `0x220` | bat=11100 mV、mode=REMOTE、state=0x0003 | `5C 2B 00 00 03 00 01 00` |

向量里的数值是**原始整数**（raw），物理值 = raw × factor。

### 校验器就是门禁

```bash
python3 tools/can_check.py            # 结构校验 + 向量比对 + e2e CRC 负例，失败退出码 1
python3 tools/can_check.py --update   # 协议改动后回写 expect（幂等）
python3 tools/can_check.py --c-header # 生成两端共用的 docs/gen/can_protocol.h
```

必测边界用例（两端各自跑）：负数饱和、int32 回绕前后值、轮速 0.01 rad/s 分辨率舍入、保留位非零拒收、`SAFETY_CMD` CRC 篡改拒收（**已有实测**：2026-10-09 联调中 CRC 用错 DID 种子的急停帧被固件正确丢弃）。

## 9. 实测基线（2026-10-09 首次联调，v3 帧格式）

- Jetson can0（mttcan + SN65HVD230）↔ 底盘 R44，1 Mbps：上行 `0x030`@1kHz + `0x200`@100Hz + `0x230` 连续 3 秒无错帧，counter 连续、CRC 全对。
- 下行急停帧（CRC 合法）经 SWD 读 RAM 验证：`estop_latched=1`、`mode=ESTOP`，落入 25 kHz ISR 快速路径。
- 根因记录：CANH/CANL 某一端反接 → 全哑 + 临界抖动；对调后全通。
- **v4 帧格式尚未有固件实现**（见 §10 待办），上表 v4 向量是规范值，待 R45 上板后按 §8 边界用例复测。

## 10. 已定决定与待办

### 10.1 已定（2026-10-09，用户拍板）

| # | 决定 |
| --- | --- |
| 1 | **CAN 全面接管**：安全、驱动、舵机、遥测全走 CAN；USB v2 串口冻结为备用（不删代码，不再演进） |
| 2 | 指令编码按 CAN 最佳实践重设计（int16 量化 + raw8 无应用层 CRC），**不照抄** USB v2 的 int32 语义 |
| 3 | v3 全量作废（互锁层随臂板退役），v4 重写 ID 表 |
| 4 | 指令频率：DRIVE 100 Hz、ARM 50 Hz；遥测 100/50/10 Hz——总线负载 ~7 % |
| 5 | 电池欠压 10500 mV 沿用为硬约束（安全层拒绝 ARMED 轮命令） |

### 10.2 固件侧待办（R45，底盘仓）

1. `app_can.c` 按 v4 重写编解码（参考 `docs/gen/can_protocol.h`），删除 XFER/EMCY_ARM；`0x010` 快速路径保留（CRC 种子改 DID 0x01 不变，向量 V1 与 v3 兼容）。
2. cantx 任务改发 `WHEEL_POS_A/B` + `WHEEL_SPD`（100 Hz）+ `SERVO_STATE_A/B`（50 Hz）+ `DIAG`（10 Hz）+ `NODE_INFO`（1 Hz）。
3. canrx 任务解析 `DRIVE_CMD`（4×int16 0.01 rad/s → 轮速目标，接原 `car_can_velocity` 路径）与 `ARM_CMD_A/B`（0.1° → `servo_set_pulse`）。
4. deadman：`DRIVE_CMD` 100 ms / `ARM_CMD` 100 ms；`DIAG` 增加 `BUSOFF_CNT`。
5. **Build tag 升 R45**；上板按 §8 边界用例复测并记录实测数据（AGENTS.md §5 规矩）。

### 10.3 Jetson 侧待办（`rak` 仓）

1. 新增 CAN 主机节点（`python-can` + SocketCAN），编解码以 `protocol-signals.yaml` 为准。
2. `rak_hardware` SystemInterface 切 CAN 传输层（4 轮速 + 7 关节位置契约不变）。
3. TUI（`rak_drive_tui` / `rak_arm_tui`）增加 CAN 后端；串口后端保留为备用。

### 10.4 版本共存约定

- `NODE_INFO.PROTO=4` 是 v4 节点的身份声明；Jetson 收到 `PROTO=3` 的 `NODE_INFO`（旧固件）应提示版本不匹配并拒绝发指令。
- v3 文档（git 历史）仅作互锁设计参考，不再维护。
