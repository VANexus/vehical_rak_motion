# CAN 通信协议 v4（两端对接规范，规范性文档）

适用范围：Jetson（`rak`，Orin Nano + SN65HVD230）↔ 底盘板 F407（`vehical_rak_motion`）↔ 机械臂板 F407（`vehical_rak_arm`）。
**这是两端的通信事实来源**：MCU 侧编解码、Jetson 侧编解码、以及本文的测试向量三者必须逐字节一致。任何一方改协议，必须先改 [`protocol-signals.yaml`](protocol-signals.yaml) 并重跑 `tools/can_check.py`。

> **★ 协议已冻结（FROZEN，2026-10-09 用户拍板）★**
> 自本版起**不再改帧/信号/ID/量化**。此后只做上位机侧功能开发——所有已规划功能
> （ros2_control 臂 7 轴、夹爪、TUI、导航上层）均已可在本协议帧集内实现。
> 确需修订走提案流程：YAML → 两端同步实现 → `can_check.py` → 本文同步 → 两 MCU 仓同批提交。
> 已实现状态：**R49-R61 固件全帧集已上板，Jetson `CanSession` 全帧收发对齐**（见 §9）。

> **v4 相对 v3 的定位变化（2026-10-09 已定，不再开洞）**
> 1. **CAN 是唯一主链路**：所有控制与遥测（安全门、4 轮速指令、7 轴舵机、全量遥测）全部走 CAN；**USB v2 串口降级为备用链路**，帧格式冻结不变（见 §10）。
> 2. **双板拓扑（2026-10-28 恢复）**：机械臂板重新启用，架构回到「底盘板（4 轮闭环 + IMU）」+「机械臂板（6 路舵机）」，两板同挂一条 CAN 总线。v3 的底盘↔机械臂互锁层（`XFER_*`）仍未启用。
> 3. **ARM 帧归属变更（2026-10-28）**：`ARM_CMD_A/B`、`SERVO_STATE_A/B` **帧 ID 与字节布局一字未动**，只是**处理板**从底盘板改为机械臂板 —— 同一条总线上 ID 全局唯一 → Jetson 侧零改动。机械臂板新增自己的两帧上行 `0x031 ARM_NODE_INFO` / `0x221 ARM_DIAG`，避免与底盘板的 `0x030` / `0x220` 同 ID 撞车。协议版本号仍是 4（`NODE_INFO.PROTO` 不变），新帧属加法，Jetson 侧可先不订阅。
> 4. **控制契约沿用 ros2-control 架构**：4 轮速 + 7 轴（6 关节 + 1 夹爪）位置，麦轮 IK 在 Jetson 侧（见 `vehical_rak_motion/docs/ROS2-CONTROL-ARCH.md`）。

术语：**必须**=强制；**应**=强烈建议，偏离需写明理由；**可**=可选。

---

## 1. 物理层与拓扑

- 单总线，**CAN 2.0A（11 bit ID）**，**1 Mbps**，经典帧 DLC=8。**禁用 CAN FD**（F407 bxCAN 不支持；Jetson mttcan 支持但必须保持关闭，`ip link` 不得带 `fd` 参数）。
- 三个节点（Jetson + 底盘板 + 机械臂板），**总线两个物理端点各 120 Ω**（Waveshare SN65HVD230 模块板载一颗，确认焊点闭合；底盘板端应补一颗）。
- **三个节点 GND 必须互联**（第三根线）：差分接收器的共模参考。缺地时表现为"偶发收得到、大量 bus-off"的临界总线——2026-10-09 联调实测。
- 两块 MCU 板侧：板载收发器，**PD0=CAN1_RX、PD1=CAN1_TX**（`docs/board-c30d.md` §2）—— 底盘板与机械臂板并联到同一对 CANH/CANL。1 Mbps 位时序 = PCLK1 42 MHz / BRP2 / 21 tq（BS1=16, BS2=4）= 精确 1.000 Mbps，采样点 81.0 %。**时钟必须来自 HSE 8 MHz**（用 HSI 会出现偶发 bus-off 玄学故障）。
- Jetson 侧：mttcan（`c310000.mttcan`，50 MHz 时钟）+ 外置 **SN65HVD230**（3.3 V 器件，可直连 Orin）。
  实配：`ip link set can0 type can bitrate 1000000 sample-point 0.800 restart-ms 100`（`can0-up.service` 持久化）。
  ⚠️ **采样点勘误（实测）**：mttcan @50 MHz 在 1 Mbps 下每 bit 恰 50 tq，采样点粒度 2 %——规范曾写 0.810，但驱动就近取整后**实际生效 0.800**。与底盘 81.0 % 差 1.0 pp，在 CAN 容限内，整场联调（97 万+ 帧）零错误帧。排查位时序问题时以 `ip -details link show can0` 实际读数为准。
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
| `0x031` | `ARM_NODE_INFO` | 机械臂板→广播 | 上电 1 次 + 1 Hz | 0x0D | raw8 |
| `0x100` | `DRIVE_CMD` | Jetson→底盘 | 100 Hz | 0x04 | raw8 |
| `0x110` | `ARM_CMD_A` (J1-J4) | Jetson→**机械臂板** | 50 Hz | 0x05 | raw8 |
| `0x111` | `ARM_CMD_B` (J5,J6,夹爪) | Jetson→**机械臂板** | 50 Hz | 0x06 | raw8 |
| `0x200` | `WHEEL_POS_A` (W1,W2) | 底盘→Jetson | 100 Hz | 0x07 | raw8 |
| `0x201` | `WHEEL_POS_B` (W3,W4) | 底盘→Jetson | 100 Hz | 0x08 | raw8 |
| `0x202` | `WHEEL_SPD` (W1-W4) | 底盘→Jetson | 100 Hz | 0x09 | raw8 |
| `0x210` | `SERVO_STATE_A` (J1-J4) | **机械臂板**→Jetson | 50 Hz | 0x0A | raw8 |
| `0x211` | `SERVO_STATE_B` (J5,J6,夹爪) | **机械臂板**→Jetson | 50 Hz | 0x0B | raw8 |
| `0x220` | `DIAG` | 底盘→Jetson | 10 Hz | 0x0C | raw8 |
| `0x221` | `ARM_DIAG` | **机械臂板**→Jetson | 10 Hz | 0x0E | raw8 |
| `0x032` | `CHASSIS_IMU` | 底盘→Jetson | **100 Hz**（2026-10-29 新增） | 0x0F | raw8 |
| `0x222` | `ARM_IMU` | **机械臂板**→Jetson | **100 Hz**（2026-10-29 新增） | 0x10 | raw8 |

### 带宽核算（1 Mbps，经典帧含填充约 110–130 µs/帧）

| 方向 | 帧率 | 来源 |
| --- | --- | --- |
| 下行 | 100 + 2×50 = 200 fps | DRIVE 100Hz + ARM 2×50Hz |
| 上行 | 3×100 + 2×50 + 10 + 1 + 10 + 1 + 100 + 100 = 572 fps | 轮位×2 + 轮速 + 舵机×2 + DIAG + NODE_INFO + ARM_DIAG + ARM_NODE_INFO + **CHASSIS_IMU 100Hz + ARM_IMU 100Hz** |
| 合计 | **~772 fps ≈ 总线负载 9–10 %** | 仍有 >9 倍余量；软 I2C 限制底盘 IMU 暂为 100 Hz |

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
| `0x031 ARM_NODE_INFO` | `PROTO` u8 + `HWREV` u8 | `BUILD` u16 | `FAULT` | `STATE` |
| `0x221 ARM_DIAG` | `FAULT` | `STATE` | `MODE` u8 + `SERVO_ERRCNT` u8 | 保留=0 |
| `0x020 EMCY` | `FAULT` | `AUX` | `STATE` | 保留=0 |
| `0x010 SAFETY_CMD` | `CMD` u16（低 4 位有效） | 保留=0 | 保留=0 | counter/flags=0 + CRC |
| `0x032 CHASSIS_IMU` | `DYAW` i16 (mrad) | `PITCH` i16 (0.01°) | `ROLL` i16 (0.01°) | `EVT` u8 + `AGE` u8 |
| `0x222 ARM_IMU` | `GZ` i16 (0.01°/s) | `AMAG` u16 (mg) | 保留=0 | `EVT` u8 + `AGE` u8 |

### 3.1 IMU 帧的事件位（`EVT_u8`，两帧同义）

| 位 | 名称 | 含义 |
| --- | --- | --- |
| bit0 | `impact` | 冲击（\|a\| 突变超阈）；机械臂板同时表示“碰撞确认” |
| bit1 | `slip` | 打滑/空转（仅底盘板：\|陀螺偏航率 − 轮速偏航率\| 超阈） |
| bit2 | `still` | 静止（零偏自标定窗口成立） |
| bit3 | `saturated` | 接近满量程，可信度下降 |
| bit4 | `stale` | 样本陈旧（>20 ms） |
| bit5-7 | — | 保留，必须为 0 |

> **前提（2026-10-29 用户确认）**：三颗 IMU 全部装在**车体**上（机械臂板不在机械臂上）。
> 刚体上任意点角速度相同 → `0x032.DYAW` 与 `0x222.GZ` 是**同源冗余**，可逐样本互校；
> 加速度受安装位置影响（α×r + ω²r），只用于模值/冲击。
> 机械臂碰撞确认 = 车静止 + 冲击尖峰 + 轮速为零；车在动时只报 bump，不参与控制判决。
>
> 两帧都只传**增量/事件**，不传姿态本体：权威姿态仍是 Jetson 的 LSM6DSR（`/dev/i2c-7`，1000 Hz）。
> MCU 侧不做跨总线融合（CAN 无同步时钟），只做预积分与本地事件判断。

## 5. 信号定义与量化

| 信号 | 位宽 | factor | 范围 | 单位 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `WxSPD` | 16 int16 | **0.01** | ±327.67 | **rad/s** | 轮角速度（前转正）。覆盖 ±327 rad/s，分辨率 10 mrad/s ≈ 0.33 mm/s 轮面速度（r=0.0325 m），远低于 25 kHz 闭环的死区 |
| `WxPOS` | 32 int32 | 1 | 全范围 | counts | 编码器累计位置，**上电=0**，里程计唯一来源。int32 在最大转速下回绕周期 > 年级，无需回绕处理 |
| `JxANG`/`GRIP` | 16 uint16 | **0.1** | 0…2700 (0…270°) | deg | 舵机角。0.1° 对齐舵机 PWM 物理分辨率（7.4 µs/° ≈ 0.07°），0.01° 是伪精度；全部无符号（0-270° 行程，Home 位见 §6.4） |
| `BAT_MV` | 16 uint16 | 1 | 0…65535 | mV | 电池电压。**欠压阈值 10500 mV**（实测定的硬约束） |
| `FAULT` | 16 uint16 | — | 见 §7 | — | 0 = 无故障 |
| `STATE` | 16 uint16 | — | 位图 | — | `bit0 estop_latched`、`bit1 remote_running`、`bit2 fault_active`、`bit3 cmd_stale`、`bit4 bat_uv`、`bit5 can_recovering`、**`bit6-10 驱动门自诊断`**（`bit6 dg_remote` 远程门、`bit7 dg_cal` 轮标定有效、`bit8 dg_imu` IMU 样本新鲜、`bit9 dg_snapshot` 编码器快照 4/4 新鲜、`bit10 dg_stab` 稳定补偿+换算完成——最近一次轮速命令入口的放行链快照）、bit11-15 保留 |
| `MODE` | 8 uint8 | — | mode 枚举 | — | 0 IDLE / 1 REMOTE / 2 ESTOP / 3 FAULT |
| `BUSOFF_CNT` | 8 uint8 | 1 | 0…255（饱和） | — | bus-off 累计计数，健康度指标 |
| `SERVO_ERRCNT` | 8 uint8 | 1 | 0…255（饱和） | — | 机械臂板：舵机指令异常计数（饱和 255），健康度指标 |
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
| 0x5 | `CLEAR_ESTOP` | **已实现（R50/R52）**：ISR 置请求，1 kHz 任务经安全门落地。门 = 电池 ≥10500 mV + 非 FAULT + 无快速停请求；PD3 使能开关门跳过（开关藏车内难达，远程清除语义） |
| 0x6 | `CLEAR_FAULT` | 故障复位（仅当故障条件已消失） |
| 0x7 | `SOFT_RESET` | `NVIC_SystemReset`；急停锁存**跨复位保持** |

- **急停路径不经过任务调度**：`0x010` 在 CAN 接收中断里校验 CRC（DID 种子）→ 置标志 → 25 kHz ISR ≤40 µs 执行刹车。全链路响应预算 **≤1 ms**。
- **CRC 校验失败 / CMD 非法**：丢弃并计数，绝不部分执行。
- **急停语义（v4 冻结版）**：急停的置位与清除**全权走 CAN `SAFETY_CMD`**。物理按钮（PE0）因实测硬件故障（恒低卡死）**禁用**，按钮固件路径保留、硬件修复后可恢复；舵机不受急停影响（保持当前位置），急停只作用于轮驱动。

### 6.2 deadman 与超时

| 监视方 | 被监视 | 时限 | 超时动作 |
| --- | --- | --- | --- |
| 底盘 | `DRIVE_CMD` 流 | **100 ms** | 轮速目标清零，受控减速停 → IDLE，`STATE.bit3 cmd_stale=1` |
| 机械臂板 | `ARM_CMD_A/B` 流 | **100 ms** | **舵机保持当前位置**（不回 Home、不断使能） |
| Jetson | `DIAG`/`NODE_INFO` 流 | **200 ms** | 判节点失联：冻结 ros2_control 输出、发 `STOP`、UI 报警 |

上电行为（强制）：**IDLE、电机不驱动、舵机直接输出 Home 安全位**（R44 语义：500/2166/2500/500/500/500 µs = 指令 0/225/270/0/0/0°；2026-10-28 起由**机械臂板**执行），各自发一帧 `NODE_INFO` / `ARM_NODE_INFO`。电机启动只有显式 `START` + `DRIVE_CMD` 路径。

**机械臂板的安全语义（2026-10-28 新增）**：臂板是独立 MCU，必须有本地安全兜底——
- 收到 `0x010 SAFETY_CMD` 的 `EMERGENCY_STOP`（cmd=4）→ 立即**停止落地新的 ARM 指令**（舵机保持当前位置），并置 `STATE.bit0 estop_latched=1`。
- 急停锁存期间收到的 `ARM_CMD_A/B` **一律不落地**（只更新遥测计数）。
- `CLEAR_ESTOP`（cmd=5）→ 清锁存（臂板无本地按钮，清除权在 Jetson）。
- 与底盘板不同：**臂板不再「舵机不受门控」**（旧契约见 §10 备注 7，那是单板时代的分工）。

### 6.3 模式机

```
IDLE --START--> REMOTE --STOP/断流--> IDLE
  |                |
  +--ESTOP(锁存)---+--ESTOP(锁存)      ESTOP --CAN CLEAR_ESTOP--> IDLE
  |                |                   （物理按钮禁用：PE0 硬件故障，v4 冻结语义）
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
| VD | `ARM_NODE_INFO` | `0x031` | proto=4、hw=1、build=0x1234 | `04 01 34 12 00 00 00 00` |
| VE | `ARM_DIAG` | `0x221` | fault=0、state=0x0003、mode=REMOTE | `00 00 03 00 01 00 00 00` |
| VF | `CHASSIS_IMU` | `0x032` | dYaw=+0.123 rad、pitch=−1.50°、roll=+2.50°、evt=impact\|slip、age=4 ms | `7B 00 6A FF FA 00 03 04` |
| V10 | `ARM_IMU` | `0x222` | gz=−12.34°/s、\|a\|=1450 mg、evt=impact、age=9 ms | 见门禁输出 |

向量里的数值是**原始整数**（raw），物理值 = raw × factor。

### 校验器就是门禁

```bash
python3 tools/can_check.py            # 结构校验 + 向量比对 + e2e CRC 负例，失败退出码 1
python3 tools/can_check.py --update   # 协议改动后回写 expect（幂等）
python3 tools/can_check.py --c-header # 生成两端共用的 docs/gen/can_protocol.h
```

必测边界用例（两端各自跑）：负数饱和、int32 回绕前后值、轮速 0.01 rad/s 分辨率舍入、保留位非零拒收、`SAFETY_CMD` CRC 篡改拒收（**已有实测**：2026-10-09 联调中 CRC 用错 DID 种子的急停帧被固件正确丢弃）。

## 9. 实测基线

- **2026-10-09 首次联调（v3 帧格式）**：Jetson can0（mttcan + SN65HVD230）↔ 底盘 1 Mbps，上行连续 3 秒无错帧；急停帧（CRC 合法）经 SWD 读 RAM 验证落入 25 kHz ISR 快速路径（`estop_latched=1`、`mode=ESTOP`）。根因记录：CANH/CANL 某一端反接 → 全哑 + 临界抖动；对调后全通。两节点 GND 未互联会造成"偶发收得到、大量 bus-off"。
- **R49-R63（v4 帧格式）**：固件全帧集实现并上板；Jetson `CanSession` 按 v4 实现下行 `DRIVE_CMD@100Hz`+`SAFETY_CMD`（e2e CRC/counter）与上行 `WHEEL_POS/SPD/DIAG/NODE_INFO` 接收；`ARM_CMD_A/B` 发送与 `SERVO_STATE_A/B` 接收在冻结版同步补齐（Session API 层）。实测数据按 §8 边界用例随台架复测记录（AGENTS.md §5 规矩）。
  - **2026-10-09 全链路实测**：安全冒烟 10/10（START→REMOTE、急停 ISR 快速路径、坏 CRC 拒收、CLEAR_ESTOP、STOP）；驱动门自诊断位图 bit6-10 实时点亮验证（零目标 DRIVE 全链放行、deadman 断流回收）；生产路径 e2e（`rak/scripts/can_session_e2e.cpp`，经 CanSession 与 ros2_control 同一代码路径）ARM 命令 Home 位与 SERVO_STATE 读回逐轴一致。
  - **2026-10-09 全动作域实测**（四轮架空、臂周清空前提）：臂 J1-J6 全行程逐轴扫描 0-270° 回读一致（±2°）+ 夹爪命令域契约（回读恒 0）+ 回 Home（`rak/scripts/can_full_motion_test.cpp`）；麦轮四类轮速组合（前进/横移/旋转/斜航/±2 rad/s）符号与通路全部正确落地，稳态跟踪 ±8 %（3 rad/s）——低速（1 rad/s）存在 ~2 s 静摩擦突破期，属轮 PI/判向**调参项，非协议**（`rak/scripts/wheel_track_probe.cpp` 归档）。MoveIt 冻结契约见 rak 仓 `docs/arm_moveit_contract.md`。
  - **R62/R63 ARM 量化勘误**：R49 起 ARM_CMD 0.1°→µrad 因子写成 ×174.533（少 10 倍，命令角缩至 1/10）；R62 修正因子后 int32 乘法在 raw>1230（>123°）溢出回绕；R63 改 int64 中间量终修。协议/YAML 语义本身无误，纯固件实现勘误——CAN ARM 路径在此次 e2e 前从未做非零实测。
- **链路层基线**：Jetson `can0` = mttcan @50 MHz，`bitrate 1000000 sample-point 0.800 restart-ms 100`（mttcan 采样点粒度 2 %，0.810 不可达、实配 0.800——见 §1 勘误；`can0-up.service` 持久化）；底盘 = HSE 8 MHz → PCLK1 42 MHz / BRP2 / 21 tq，精确 1.000 Mbps、采样点 81.0 %。

## 10. 已定决定与待办

### 10.1 已定（2026-10-09，用户拍板）

| # | 决定 |
| --- | --- |
| 1 | **CAN 全面接管**：安全、驱动、舵机、遥测全走 CAN；USB v2 串口冻结为备用（不删代码，不再演进） |
| 2 | 指令编码按 CAN 最佳实践重设计（int16 量化 + raw8 无应用层 CRC），**不照抄** USB v2 的 int32 语义 |
| 3 | v3 全量作废（互锁层随臂板退役），v4 重写 ID 表 |
| 4 | 指令频率：DRIVE 100 Hz、ARM 50 Hz；遥测 100/50/10 Hz——总线负载 ~7 % |
| 5 | 电池欠压 10500 mV 沿用为硬约束（安全层拒绝 ARMED 轮命令） |
| 6 | **协议内容冻结（FROZEN）**：驱动门自诊断位图（STATE bit6-10）为最后一项内容修订；急停置位/清除全权走 CAN（按钮 PE0 硬件故障禁用，语义见 §6.1） |
| 7 | ~~舵机命令不受固件远程门/急停门控~~（单板时代契约，2026-10-28 作废）——舵机迁到独立机械臂板后，臂板自己锁存急停并拒绝落地指令（见 §6.2）；Jetson 侧仍负责「何时该动」的许可 |

### 10.2 固件侧（R45→R61，✅ 已完成）

1. ✅ `app_can.c` 按 v4 重写编解码，XFER/EMCY_ARM 已删除；`0x010` 快速路径保留（CRC 种子 DID 0x01，向量 V1 兼容）。
2. ✅ cantx 任务发 `WHEEL_POS_A/B` + `WHEEL_SPD`（100 Hz）+ `SERVO_STATE_A/B`（50 Hz）+ `DIAG`（10 Hz）+ `NODE_INFO`（1 Hz），相位错开防邮箱挤爆。
3. ✅ canrx 任务解析 `DRIVE_CMD`（接 `car_wheel_command_urad` 路径）与 `ARM_CMD_A/B`（凑齐 A/B 组落地 `servo_set_pulse`）。
4. ✅ deadman：`DRIVE_CMD` 100 ms / `ARM_CMD` 100 ms；`DIAG` 含 `BUSOFF_CNT` 与 STATE bit6-10 驱动门自诊断。
5. ✅ R50/R52：CAN `CLEAR_ESTOP` 启用（§6.1 门）；R51/R56：过滤器/RX 通知运行时自愈 @1 Hz；R57：过滤器寄存器偏移修正。
6. ✅ R59：按钮急停禁用（PE0 硬件故障），CAN 全权语义（§6.1）。

### 10.3 Jetson 侧（`rak` 仓，冻结版状态）

1. ✅ CAN 传输后端：`src/rak_chassis` `CanSession`（SocketCAN），与 `SerialSession` 同 `Session` API，`transport:=usb|can` 工厂切换。
2. ✅ 冻结版补齐：`set_arm_command`（7 轴 rad → 0x110/111，0.1° 量化）+ `SERVO_STATE_A/B` 订阅解析进 `SessionSnapshot`——**Session 通信能力与协议帧集完全对齐**。
3. ✅ `rak_system`（ros2_control SystemInterface）4 轮 velocity/pos-vel 接口已接 CAN 后端。
4. ⬜ 上位机**功能级**待办（均不需要动协议）：ros2_control 臂 7 轴 position 接口 + `joint_trajectory_controller` 接线；TUI（`rak_commissioning`）CAN 后端；`EMCY(0x020)` 事件帧两端按需启用（DIAG 已覆盖故障上报，帧定义保留）。

### 10.4 版本共存约定

- `NODE_INFO.PROTO=4` 是 v4 节点的身份声明；Jetson 收到 `PROTO=3` 的 `NODE_INFO`（旧固件）应提示版本不匹配并拒绝发指令。
- v3 文档（git 历史）仅作互锁设计参考，不再维护。
