> **两份副本约定**：本文件在 `vehical_rak_chassis` 与 `vehical_rak_arm` 各存一份，内容必须保持一致——改动时**同一批提交两个仓**（手工 cp 即可，没有同步工具）。

# CAN 通信协议 v3（三端对接规范，规范性文档）

适用范围：Jetson（`rak`）↔ 底盘 F407（`vehical_rak_chassis`）↔ 机械臂 F407（`vehical_rak_arm`）。
**这是三端唯一的通信事实来源**：MCU 侧的 `app_can.h`、Jetson 侧的编解码实现、以及本文的测试向量三者必须逐字节一致。任何一方改了协议，必须先改本文并重发测试向量。

术语：**必须**=强制；**应**=强烈建议，偏离需写明理由；**可**=可选。

---

## 1. 物理层与拓扑

- 单条总线，**CAN 2.0A（11 bit ID）**，**1 Mbps**，经典帧 8 字节。**不使用 CAN FD**——F407 的 bxCAN 不支持 FD，一帧 FD 会让两个 MCU 节点判为错误帧（`rak` 文档里"CAN FD"的表述待修正，见 §10）。
- 三个节点全部挂同一条总线，**只有总线两个物理端点各接 120 Ω**（中间节点不接）。Jetson、两块 F407 的 **GND 必须互联**。
- **两块 F407 用板载 CAN 收发器**，MCU 侧引脚固定 **PD0 = CAN1_RX、PD1 = CAN1_TX**（`docs/board-c30d.md` §2，出自板级资源 PDF）。⚠️ 设计草案里的 `CAN1_RX=PB8/CAN1_TX=PB9` **作废**——PB8/PB9 是电机 A 的两个驱动输入。
- **Jetson 侧用外接模块**（Orin 板载 mttcan 控制器 `can0`，50 MHz 时钟，硬件支持 FD 但本项目**禁止启用 FD**）。
  ⚠️ 用户计划的 **TJA1050 是 5 V 器件**，其 RXD 输出高电平约 VCC−0.5 V ≈ 4.25 V，而 Orin 的 GPIO/CAN 引脚是 3.3 V 域。落地前必须二选一：① 改用 **3.3 V 收发器**（SN65HVD230 / MCP2551+3.3V 逻辑侧 / TJA1050T 的 3.3 V 版本）；② 保留 TJA1050 但在 RXD→Orin 之间加电平移位。**不要直连赌它扛得住。**
- 接线极性：CAN_H / CAN_L 不得反接；总线隐性电平 ≈2.5 V，显性 ≈ CAN_H 3.5 V / CAN_L 1.5 V。上电第一件事是用万用表量这两个电平，接错会表现为"偶发收不到帧"。

## 2. 位时序与时钟（最容易翻车的一处）

**两个 MCU 节点的 CAN 位时钟必须来自 HSE 8 MHz 晶振，不能是 HSI。**

骨架当前是 HSI+PLL→168 MHz（`.ioc` 里 PH0/PH1 已配成外部晶振模式但时钟树没用）。HSI 的精度是百分比分级、随温度漂移，而 1 Mbps 下 CAN 对两端晶振总偏差的要求在几百 ppm 量级——**用 HSI 跑 1 Mbps 会出现"有时能通、偶发 bus-off"的玄学故障**，且难以定位。工程化第一步就是把时钟树切到 HSE（上一轮 H5–H7 也是用 HSE 的）。

推荐的 CAN1 位时序（PCLK1 = 42 MHz，采样点 85.7 %，符合 CiA 对 1 Mbps ≤87.5 % 的建议）：

| 参数 | 值 | 说明 |
| --- | --- | --- |
| 分频 BRP | 3 | 42 MHz / 3 = 14 MHz 位时钟 |
| 同步段 | 1 tq | 固定 |
| 段 1 (BS1) | 11 tq | |
| 段 2 (BS2) | 2 tq | 采样点 = (1+11)/14 = 85.7 % |
| SJW | 1 tq | |
| 合计 | 14 tq/bit → **1.000 Mbps** | |

验收方法：① 三端各自 `ip -details link show can0` / OLED 显示 bus-off 计数为 0；② 连续跑 10 min 压力测试，`berr-counter` 不增长；③ 有条件时用示波器量 CANH-CANL 显性脉宽。

## 3. 标识符规划（优先级即功能）

11 bit ID 按"越小优先级越高"分配。**安全类帧的 ID 一定小于所有指令与反馈帧**，保证急停永远抢得到总线——这与草案"0x102 急停、0x300 节点间"的编号顺序相反，是 v3 最重要的一处修正。

| 优先级 | ID | 名称 | 方向 | 周期 | Data ID |
| --- | --- | --- | --- | --- | --- |
| 1 安全 | `0x010` | `SAFETY_CMD` | Jetson→广播；节点也需监听 | 事件 | 0x01 |
| 2 故障 | `0x020` | `EMCY_CHASSIS` | 底盘→广播 | 事件（10 ms 内首发，重发 3 次） | 0x02 |
| 2 故障 | `0x021` | `EMCY_ARM` | 机械臂→广播 | 同上 | 0x03 |
| 3 互锁 | `0x030` | `XFER_CH_TO_ARM` | 底盘→机械臂 | **1 kHz** | 0x04 |
| 3 互锁 | `0x031` | `XFER_ARM_TO_CH` | 机械臂→底盘 | **1 kHz** | 0x05 |
| 4 指令 | `0x100` | `CHASSIS_VEL_CMD` | Jetson→底盘 | 100 Hz | 0x06 |
| 4 指令 | `0x101` | `ARM_JOINT_CMD_A` (J1-J3) | Jetson→机械臂 | 100 Hz | 0x07 |
| 4 指令 | `0x102` | `ARM_JOINT_CMD_B` (J4+BASE) | Jetson→机械臂 | 100 Hz | 0x08 |
| 5 反馈 | `0x200` | `CHASSIS_VEL_FB` | 底盘→Jetson | 100 Hz | 0x09 |
| 5 反馈 | `0x201` | `CHASSIS_DIAG` | 底盘→Jetson | 10 Hz | 0x0A |
| 5 反馈 | `0x210` | `ARM_JOINT_FB_A` (J1-J3) | 机械臂→Jetson | 100 Hz | 0x0B |
| 5 反馈 | `0x211` | `ARM_JOINT_FB_B` (J4+BASE) | 机械臂→Jetson | 100 Hz | 0x0C |
| 5 反馈 | `0x212` | `ARM_DIAG` | 机械臂→Jetson | 10 Hz | 0x0D |
| 6 管理 | `0x230` | `NODE_INFO_CHASSIS` | 底盘→广播 | 上电 1 次 + 1 Hz | 0x0E |
| 6 管理 | `0x231` | `NODE_INFO_ARM` | 机械臂→广播 | 上电 1 次 + 1 Hz | 0x0F |

**与草案编号的映射**（便于对照旧文档，不代表兼容）：`0x100→0x100`、`0x101→0x101+0x102`（拆两帧）、`0x102→0x010`、`0x200→0x200+0x201`、`0x201→0x210+0x211+0x212`、`0x300→0x030`、`0x301→0x031`、`0x302→0x010` 的 sync 位。

> 为什么关节指令拆成两帧：草案的 `0x101` 用 4×int16 占满 8 字节，既没有 CRC 位也没有计数器位，与草案自己"每帧必须 CRC8"的要求矛盾。v3 统一为"5 字节数据 + 1 字节 counter/flags + 1 字节 CRC"，所以每帧最多装 2 个 int16 + 1 个 uint16 或 3 个 int16（见 §5 打包规则）。**关节角因此 3+1 拆帧。**

## 4. 通用帧结构（E2E 保护）

所有帧的 DLC = 8，布局统一：

| 字节 | 内容 |
| --- | --- |
| 0 … 5 | 信号数据（按各帧信号表，**小端**） |
| 6 | **高 4 位 = Alive Counter（0-15 滚动）**，**低 4 位 = Flags**（按帧定义） |
| 7 | **CRC8**，覆盖字节 0-6（含 counter/flags），**初值 = 本帧 Data ID**，多项式 0x07，无反射、无输出异或 |

```c
/* 参考实现（三端逐字等价；Jetson 侧 Python 同算法） */
uint8_t crc8(const uint8_t *p, uint8_t n, uint8_t init) {
    while (n--) { uint8_t c = *p++; c ^= init; init = c & 0x80 ? (c<<1)^0x07 : c<<1; }
    /* 注意：上面这种写法错，用下面的标准逐位式 */
    return init;
}
/* 正确版本 */
uint8_t crc8(const uint8_t *d, uint8_t len, uint8_t data_id) {
    uint8_t crc = data_id;                     /* 初值 = Data ID */
    for (uint8_t i = 0; i < len; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;                                /* 无最终异或 */
}
```

**字节序：一律 Intel / 小端**（低字节放在低地址字节）。理由：STM32 是小端机，可 `memcpy` 直接映射；ROS 侧手写编码不易错。⚠️ 这与上一轮"多字节字段大端"的口径**相反**——两端都还没有实现，现在统一成本最低，但任何从旧工程搬来的代码都要改。

**接收端校验顺序（强制）**：CRC 正误 → counter 是否 +1（允许 +0 视作重复帧丢弃）→ 是否跳变（跳变计入丢帧统计）→ 才更新应用数据。任一失败：丢弃该帧并累加错误计数，**绝不部分更新**。

**Flags 低 4 位定义**

| 帧族 | bit0 | bit1 | bit2 | bit3 |
| --- | --- | --- | --- | --- |
| 指令帧 | enable（1=允许执行） | 限速档（0=高限/1=低限） | 保留 | 保留 |
| 状态帧 | mode bit0 | mode bit1 | estop 锁存 | fault 有效 |
| `SAFETY_CMD` | 命令码 bit0-3（见 §7） | | | |

mode（2 bit，状态帧）：`0=IDLE`、`1=REMOTE`、`2=ESTOP`、`3=FAULT`。

## 5. 信号定义与量化

| 信号 | 位宽 | 类型 | factor | 范围 | 单位 | 说明 |
| --- | --- | --- | --- | --- | --- | --- |
| `VX`, `VY` | 16 | int16 | 1 | −32768…32767 | **mm/s** | 车体系速度；X 前 Y 左 |
| `WZ` | 16 | int16 | **0.001** | ±32.768 | **rad/s** | 逆时针为正。⚠️ 草案是 0.01（分辨率 0.57 °/s，太粗），v3 改 0.001 |
| `J1..J4_POS` | 16 | int16 | **0.01** | ±327.67 | **度** | 关节角，单圈关节够用；比 0.1° 精细，仍不溢出 |
| `BASE_POS` | 16 | **uint16** | **0.01** | 0…655.35 | **度** | 底座 360° 连续旋转，无符号才能覆盖 |
| `BAT_MV` | 16 | uint16 | 1 | 0…65535 | mV | 电池电压（PB0 分压换算） |
| `FAULT_CODE` | 16 | uint16 | — | 见 §8 | — | 0 = 无故障 |
| `WHEEL_RPM` | 8 ×4 | uint8 | 10 | 0…2550 | rpm | 每路一轮，低分辨率足够遥测 |
| `STATE_u16` | 16 | uint16 | — | 位图 | — | `bit0 homed`、`bit1 homing_active`、`bit2-3 mode`、`bit4 estop_latched`、`bit5 holding`（臂在主动锁位）、`bit6 stepper_mismatch`、`bit7 cmd_stale`（见 `protocol-signals.yaml` 的 `state_bits`） |
| `PROTO_VER` | 8 | uint8 | — | — | — | 本协议版本，本文档 = **3** |

**打包规则**：同一帧内信号按表顺序从字节 0 起紧凑排列，小端；不得跨帧共享信号；不得在帧内留空洞（未用位填 0）。

**编码约定**：物理量与整数的换算一律 `round(物理值 / factor)`，饱和截断到范围端点，**不取模、不回绕**。三端各写一个"越界即拒绝发送并计数"的检查，不要靠下游发现。

## 6. 帧信号表

| 帧 | 字节 0-1 | 2-3 | 4-5 | 字节 6 | 字节 7 |
| --- | --- | --- | --- | --- | --- |
| `0x100 CHASSIS_VEL_CMD` | `VX` | `VY` | `WZ` | counter/flags(enable, 限速) | CRC |
| `0x101 ARM_JOINT_CMD_A` | `J1_POS` | `J2_POS` | `J3_POS` | counter/flags(enable) | CRC |
| `0x102 ARM_JOINT_CMD_B` | `J4_POS` | `BASE_POS` | 保留=0 | counter/flags(enable) | CRC |
| `0x200 CHASSIS_VEL_FB` | `VX`实测 | `VY`实测 | `WZ`实测 | counter/flags(mode/estop/fault) | CRC |
| `0x201 CHASSIS_DIAG` | `BAT_MV` | `FAULT_CODE` | **`STATE_u16`**（位图） | counter/flags | CRC |
| `0x210 ARM_JOINT_FB_A` | `J1_POS`实测 | `J2` | `J3` | counter/flags | CRC |
| `0x211 ARM_JOINT_FB_B` | `J4`实测 | `BASE_POS`实测 | 保留=0 | counter/flags | CRC |
| `0x212 ARM_DIAG` | `BAT_MV` | `FAULT_CODE` | **`STATE_u16`** | counter/flags | CRC |
| `0x030 XFER_CH_TO_ARM` | `VX`实测 | `VY`实测 | `WZ`实测 | counter/flags | CRC |
| `0x031 XFER_ARM_TO_CH` | `FAULT_CODE` | `J4_POS`实测 | `BASE_POS`实测 | counter/flags | CRC |
| `0x010 SAFETY_CMD` | `CMD_CODE`（见 §7） | 保留=0 | 保留=0 | counter/flags=0 | CRC |
| `0x020/0x021 EMCY_*` | `FAULT_CODE` | `FAULT_AUX`（首个出错通道号等） | 保留=0 | counter/flags | CRC |
| `0x230/0x231 NODE_INFO` | `PROTO_VER`+`HW_REV`（打包 16 bit） | `BUILD_HASH`（固件 tag 的 CRC16 或 git short hash 转 uint16） | 保留=0 | counter/flags | CRC |

**`0x031` 里为什么放 J4/BASE**：底盘需要的是"臂是否伸出、伸出多少"用于稳定性互锁，不需要 1 kHz 的全 4 关节；`XFER_ARM_TO_CH` 用 1 kHz 传故障 + 最影响质心的两个量。这是我对草案"1 kHz 全量互发"做的裁剪，**理由与代价见 §9 待确认 #4**。

## 7. 网络管理、模式机与安全动作

### 7.1 `SAFETY_CMD`（ID 0x010，广播）

`CMD_CODE`（uint16，低 4 位有效）：

| 值 | 命令 | 效果 |
| --- | --- | --- |
| 0x1 | `START` | IDLE/FAULT→REMOTE（前提是 estop 已清、使能开关有效） |
| 0x2 | `STOP` | 受控减速到 IDLE（可被 START 恢复） |
| 0x4 | `EMERGENCY_STOP` | **立即锁存急停**：底盘动态刹车；**机械臂进入 `ESTOP_HELD` —— 持续闭环主动锁住当前关节角，不断使能**（已确认：臂断电会回落）。只停止接受新的运动目标 |
| 0x5 | `CLEAR_ESTOP` | 解除急停锁存 → IDLE（不自动进 REMOTE） |
| 0x6 | `CLEAR_FAULT` | 故障复位（仅当故障条件已消失；否则保持 FAULT） |
| 0x7 | `SOFT_RESET` | 节点自复位（`NVIC_SystemReset`），复位后仍保持急停锁存与未 homed 状态 |
| 0x8 | `HOME_REQUEST` | **请求零位标定**（低速单向堵转找死点）。这是唯一允许节点自主运动的初始化途径；必须同时满足 `enable=1` + 使能开关有效，且建议人工在旁（见 §7.5） |
| 0x9 | `RELEASE_HOLD` | 解除锁位、断使能。**仅在臂处于已 parked 姿态或人工支撑时发**；否则会造成回落砸物（`STATE.bit5 holding` 会如实回报） |

- **急停路径不经过 100 Hz 任务**：`0x010` 在 CAN 接收中断里置标志，25 kHz ISR 下一个周期（≤40 µs）就执行刹车/锁位。全链路响应预算 **≤1 ms**。
- **急停是锁存的**，只有 `CLEAR_ESTOP` 能解除；看门狗复位、软复位都不得清除它（放到 RTC 备份寄存器或复位后备 RAM，并在 `NODE_INFO`/`DIAG` 里回报锁存状态）。
- ⚠️ **锁位不是免费的**：`ESTOP_HELD` 下堵转电流会长期流过电机与 AT8236（本板**无电流采样**，只能靠占空比/温度模型估算），因此锁位状态**应限时**（建议 ≤30 s，超时转 `FAULT` 并上报，同时提示人工介入），而不是无限期硬抱。

### 7.2 生命期（deadman）与超时——**两个节点动作不同**，这是 v3 与草案的另一处修正

| 监视方 | 被监视 | 时限 | 超时动作 |
| --- | --- | --- | --- |
| 底盘 | `0x100` 指令流 | **100 ms** | 目标速度置零，按 ≤`DECEL_LIMIT`（建议 3000 mm/s²）减速停 → IDLE |
| 机械臂 | `0x101`/`0x102` 指令流 | **100 ms** | **保持当前关节位置**（不回落、不回零），持续闭环 |
| 机械臂 | `0x030`（底盘互锁帧） | **10 ms** | 底盘失联 → 进入 FAULT 并锁存（臂不依赖底盘姿态，但互锁要求可见） |
| 底盘 | `0x031`（臂互锁帧） | **10 ms** | 记录臂故障，若臂报 FAULT 则限制自身速度档 |
| Jetson | `0x200`/`0x210` 反馈流 | **200 ms** | 判节点失联：冻结 Nav2/MoveIt 输出，发 `STOP`，UI 报警 |

理由：**用户已确认臂断电/停机会回落**（MG513P30 + 30:1、无抱闸）。所以"停"对臂而言等于"掉下来"——草案给两者都写"自动停止"是错的。臂的所有降级动作都必须是**保持闭环锁位**，真正的释放只能由 `RELEASE_HOLD` 显式触发。

### 7.5 零位标定（homing）——机械臂唯一的位基建立途径

硬件现实（用户确认）：**执行器只有 WHEELTEC MG513P30_12V，没有限位开关、没有绝对值编码器、没有电流采样**。所以零点只能靠**低速堵转撞到机械死点**建立：初始时那个死点位置是卡死的，能确定它就是 0；但一旦初始化完再次撞同一点，不能默认它仍是零位（必须单向趋近）。

流程（强制，写进固件状态机）：

1. **方向性**：零位标定只能从**同一侧**趋近死点（消除 30:1 减速箱齿隙引入的迟滞误差）；回程重新趋近时必须先退回安全角度再多走一段补偿量（建议 ≥ 实测齿隙迟滞带宽，需标定）。
2. **低速低压**：堵转阶段速度上限建议 ≤ 50 计数/ms、占空比上限 ≤ 40 %（**具体值必须实测**：目标是能撞死但不顶坏结构件）。
3. **堵转判定**（无电流采样，只能用间接判据）：同一轴占空比已达上限、但 1 ms 窗速度增量连续低于阈值，持续 ≥ `HOMING_STALL_MS`（建议 300 ms）→ 判定到位；随即停止该轴并将当前位置记为 0。
4. **超时即失败**：单轴在 `HOMING_TIMEOUT_MS`（建议 5 s）内未堵转 → `FAULT` + `EMCY(0x0052)`，**不得自行重试**，也不得把当前点当零位。
5. **未 homed 就禁止 REMOTE**：`STATE.bit0 homed = 0` 时，任何 `START` 都应被拒绝并报 `0x0053`；`SOFT_RESET`/看门狗复位后 `homed` 必须置回 0（零位不能存在 Flash 里当永久真值——齿隙与机械变动会让它骗人）。
6. **底盘不做 homing**：底盘只需上电把当前轮位当里程计原点（相对零位），没有绝对位置需求。

### 7.3 上电与使能顺序（强制）

1. 上电后节点一律进 `IDLE`，**输出为零、电机不驱动**，发一帧 `NODE_INFO`；机械臂额外要求 `homed=0`；
2. 只有在板载使能开关（PD3）有效 + 收到 `START` + 指令帧 `enable=1` 三者同时成立，才允许进入 `REMOTE`；**机械臂额外多一个前置条件：`homed=1`**（先 `HOME_REQUEST` 再 `START`）；
3. `REMOTE` 下若 `enable` 位从 1→0，按 `STOP` 处理（底盘受控减速；臂保持锁位）；
4. 任何自检失败（编码器无变化、CAN bus-off、电压欠压）→ `FAULT` + `EMCY`。

### 7.4 时间同步

不做硬时基同步（1 Mbps + FreeRTOS 抖动无法保证同拍）。`0x010` 的 sync 语义仅用于"同时开始/同时停止"这类粗对齐；两板 1 kHz 互发本身即互相可见时标。**Jetson 侧对每帧记录内核接收时间戳**，用于端到端延迟与丢帧核算（目标：单向 <1 ms）。

## 8. 故障码（`FAULT_CODE`，uint16）

| 码 | 名称 | 触发条件 | 动作 |
| --- | --- | --- | --- |
| 0x0000 | NONE | — | — |
| 0x0010 | ENC_STALL | 占空比 >50 % 持续 500 ms 而编码器增量 <阈值 | FAULT，停该通道 |
| 0x0011 | ENC_JUMP | 单 1 ms 窗速度突变量 >2× 上限 | FAULT |
| 0x0012 | ENC_NO_SIGNAL | 上电自检时四路编码器计数全零 | FAULT，禁止 REMOTE |
| 0x0020 | CAN_BUS_OFF | bxCAN 进入 bus-off | 自动重入网（`restart-ms`），累计计数入 DIAG |
| 0x0021 | CAN_CRC | 连续 20 帧 CRC 错 | 降速档 + 报 DIAG |
| 0x0030 | BAT_UV | 电池 <10.5 V（12 V 系统，**待按实际电池曲线定**） | 禁止峰值输出，限速档 |
| 0x0031 | BAT_OV | >16 V | FAULT |
| 0x0040 | LOOP_OVERRUN | 25 kHz ISR 占用 >70 % 或丢拍 | FAULT（这是实时性红线） |
| 0x0050 | JOINT_LIMIT | 关节超软件限位（机械臂） | 该轴停止，全臂 FAULT |
| 0x0051 | STEP_STALL | 步进已发脉冲但位置无变化（机械臂底座） | 该轴 FAULT |
| 0x0052 | HOMING_FAIL | 零位标定超时未堵转（§7.5 第 4 条） | FAULT，`homed` 保持 0，禁止 REMOTE |
| 0x0053 | NOT_HOMED | 未做零位标定就 `START`（机械臂） | 拒绝，回 `IDLE` |
| 0x0054 | HOLD_TIMEOUT | `ESTOP_HELD` 锁位超过限时（建议 30 s） | 转 FAULT 并提示人工介入（不断使能） |
| 0x0060 | ESTOP_LATCHED | 急停锁存中 | 见 §7.1 |

⚠️ 草案里的"电流保护 / 过流检测"**没有硬件依据**：本板没有任何通道电流采样通道（`docs/board-c30d.md` §3）。v3 用 `ENC_STALL`（占空比高 + 编码器不动）做等效判据。真需要电流保护，得在预留 ADC（PC4/PA4/PA5）上加检流电阻并标定，那是硬件改动。

## 9. 一致性测试向量（三端必须复现）

单一来源：[`protocol-signals.yaml`](protocol-signals.yaml)（机器可读信号库）。下表由 ``../tools/can_check.py``（路径在工作区根，本仓不包含） 用参考实现**实际编码得出**（已运行验证，非手推）：

| # | 帧 | ID | 输入 | 8 字节（hex） |
| --- | --- | --- | --- | --- |
| V1 | `CHASSIS_VEL_CMD` | `0x100` | Vx=500、Vy=−200、ωz=150（=0.150 rad/s）、enable=1、counter=0 | `F4 01 38 FF 96 00 01 60` |
| V2 | `ARM_JOINT_CMD_A` | `0x101` | J1=10.00°、J2=−5.00°、J3=2.50°、enable=1、counter=1 | `E8 03 0C FE FA 00 11 21` |
| V3 | `CHASSIS_VEL_FB` | `0x200` | 实测 Vx=498、Vy=−203、ωz=149、mode=REMOTE、counter=12 | `F2 01 35 FF 95 00 C1 52` |
| V4 | `SAFETY_CMD` | `0x010` | CMD=EMERGENCY_STOP(4)、counter=5 | `04 00 00 00 00 00 50 1D` |
| V5 | `XFER_ARM_TO_CH` | `0x031` | fault=0x0051、J4=−12.34°、BASE=360.00°、counter=15、flags=0x9 | `51 00 2E FB A0 8C F9 BF` |
| V6 | `NODE_INFO_CHASSIS` | `0x230` | proto=3/hw=1、build=0xABCD、counter=0 | `01 03 CD AB 00 00 00 5C` |

向量里的数值是**原始整数**（raw），物理值 = raw × factor（例：V1 的 ωz raw=150、factor=0.001 → 0.150 rad/s）。

### 校验器就是门禁

```bash
python3 tools/can_check.py            # 结构校验 + 向量比对 + CRC 篡改负例，失败退出码 1
python3 tools/can_check.py --update   # 协议改动后回写 expect（已验证幂等）
python3 tools/can_check.py --c-header # 生成两个 MCU 工程共用的 docs/gen/can_protocol.h
```

已实测的行为：改乱任意一条 `expect` → 退出码 1 并指 出差异；逐字节篡改帧内容 → CRC 必失败（负例检查）；协议字段改后 `--c-header` 重生成，C 端的 ID/Data ID/周期宏自动跟着变。**任何一方单独“顺手改协议”都会在 CI 上抖出来，而不是上线后才发现对不上。**

必测的边界用例（三端各自跑）：负数饱和（Vx=−40000 → 截断到 −32768 并计一次越界）、counter 回绕（15→0）、CRC 错 1 位、重复帧（同 counter）、跳帧（+2）、`BASE_POS=65535`（655.35°）、`WZ` 符号、保留位非零（应拒收）。

## 10. 已定决定与待办

### 10.1 用户 2026-10-02 已定（不再开洞）

| # | 决定 | 写进哪里的哪一节 |
| --- | --- | --- |
| 1 | **底盘全向运动**：前后左移右移 + 自转，**且允许三者同时非零**（斜走、边走边转） | `0x100` 的 `VY` 有效（§5）；MCU 侧必须做**矢量饱和等比缩放**（§10.2-1） |
| 2 | **航向权威源 = Jetson 的 LSM6DSRTR（SPI）**；两块 F407 不读板载 ICM20948（硬件存在，PB10/11/12，但不用） | MCU 不做姿态/航向（architecture.md §1）；帧里**不加 yaw 字段** |
| 3 | **臂断电会回落** → 所有降级动作 = 主动锁位，释放只能 `RELEASE_HOLD` 显式触发 | §7.1/§7.2/§7.5、故障码 `0x0054` |
| 4 | **零位只能靠低速堵转撞死点建立**（无开关、无绝对编码器） | 新增 `HOME_REQUEST` + §7.5 流程 + `STATE.bit0 homed` + 故障码 `0x0052/0x0053` |
| 5 | **Jetson 侧收发器 = TJA1050**（外置模块） | §10.3 带电平警告 |
| 6 | 夹爪驱动、步进驱动器型号与反馈：**推迟**（不阻塞 4 个关节的 Phase B） | 记在 ``vehical_rak_arm/README.md``（路径在工作区根，本仓不包含） §4 |
| 7 | 电源系统（欠压门限等）：**不用管** | `0x0030/0x0031` 保留但门限暂不启用（先只报 DIAG 不触发 FAULT） |

### 10.2 因此派生的实现约束（不是新问题，是上面的推论）

1. **矢量饱和必须等比**：三轮同时给 `Vx`、`Vy`、`ωz` 时，某轮的合成轮速可能超限。处理方式：先把四轮目标算出来，若 `max(|v_i|) > V_CMD_LIMIT` 则**四个同比例缩小**，**不得逐轴截断**（逐轴截断会改变运动方向，斜走会变直走）。这条写进 `app_mecanum` 的验收用例。
2. **Jetson 侧控制器选型**：现有 `motor_stub/cmd_vel_stub.py` 是原样透传 `Twist`（它自己注释写了"unicycle contract is the only contract"）。要产生 `linear.y`，**Nav2 必须换 `nav2_mecanum_controller`**（输出的 Twist 带 y）；`cmd_vel_stub` 作为透传层可以不改，但它的注释与文档得同步改成"全向合同"。MoveIt 下发的是关节目标，走 `0x101/0x102`，与路径规划不冲突。
3. **`homed` 不入 Flash**：复位/看门狗后必须重做零位标定。这是硬要求，不是建议。

### 10.3 Jetson（`rak`）侧待办（用户指示：先 push 到最新再改）

**仓状态（先对齐再动工）**：本地 `rak` HEAD = `3588521`（把 Orin 运行树 import 成 snapshot，领先 `origin/main` 1 个提交）且 `brain/` 有未提交改动；Orin 上 `~/rak` HEAD 仍是更早的 `1b2869e` 却有 **34 个未提交文件**；两边都没 push。**改 `rak` 前必须先决定以哪边为准**，否则会互相覆盖。

实测事实（直接查 Orin 得到）：机器 = **Jetson Orin Nano Developer Kit**；`can0` 存在但 DOWN，驱动 = **mttcan**（`c310000.mttcan`，clock **50 MHz**，带 dtseg/dsjw 即**硬件支持 FD，但本项目禁用**）；另有第二个控制器 `c320000` 未注册；`/opt/nvidia/jetson-io` 存在但**里面没有任何 CAN 功能项**（grep 无命中）→ **CAN 引脚不在 jetson-io 可配范围**，具体物理针脚得查载板手册/设备树后才能断言。另：全仓**一行 CAN 代码都没有**（`nerve` 不存在、`CanFrame.msg` 已删）。

要改的：

1. `rak/CLAUDE.md` 与 `README.md` 里的 **3 处 "CAN FD"** → 本v3（CAN 2.0A @1 Mbps）。
2. `rak/CLAUDE.md` 写的叶子节点 **STM32G474 + HRTIM** / **STM32H743**（`Thecnfor/rak-arm`、`rak-chassis`）→ 实际是**两块相同的 C30D F407**，在本工作区 `vehical_rak_arm`/`vehical_rak_chassis`。
3. 新增 CAN 节点（按 §3-§6 + `protocol-signals.yaml`）。
4. 启用命令**不得带 fd**：`ip link set can0 type can bitrate 1000000` → `ip link set can0 up`。上总线前先用 **`loopback` 自环**验证控制器与驱动栈（`cansend`/`candump` 自己发自己收），再接真总线——这样能把"引脚没引出来 / 收发器电平不对 / 位时序错"三类问题分开。需 sudo（本机 `sudo -n` 被拒，要口令）。
5. **TJA1050 电平警告（必须处理）**：TJA1050 是 **5 V 供电**器件，RXD（送进 Orin 的那根）输出高电平接近其 IO 电源，而 Orin 的 GPIO 是 3.3 V 域。三种可行做法：① 用带 **VIO** 的版本并把 VIO 接 3.3 V；② RXD→Orin 串 1 kΩ + 3.3 V 钳位二极管；③ 直接换 **3.3 V 收发器**（SN65HVD230，与 F407 板载同类）。**上电前先用万用表量 RXD 对地电压，确认 ≤ 3.6 V 再插 Orin 针脚。**

### 10.4 MCU 侧要改的

1. 时钟树切 HSE（§2）——**上总线前第一项**。
2. CAN 引脚按 **PD0/PD1**，收发器用板载；`.ioc` 里别选 PB8/PB9。
3. `app_can` 只写纯函数 pack/unpack（可 host 编译 + 跑 §9 向量），ISR 里只置标志与拷贝，不解析。
