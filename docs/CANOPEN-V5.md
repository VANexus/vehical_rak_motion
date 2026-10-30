# CAN 通信规范 v5 —— CANopen（规范性文档）

> **本文件是 v5 的规范件**，与 `docs/protocol.md`（v4）互斥：同一总线上只能跑一套。
> 三个 MCU/上位机仓各存一份副本（伞形 `docs/`、`vehical_rak_motion/docs/`、
> `vehical_rak_edge_ai/docs/`），内容必须逐字一致（`tools/canopen_gate.sh` 校验）。
> 设计依据与落地进度：`docs/superpowers/specs/2026-10-29-can-v5-canopen-design.md`。

## 0. 现状（2026-10-29）

| 侧 | 状态 |
| --- | --- |
| 底盘板 `vehical_rak_motion`（R73）、机械臂板 `vehical_rak_edge_ai`（A14） | **v5 已落地**（P0 栈接入 + P1 PDO 全链路，台架实测通过） |
| Jetson `rak` | ⚠️ **仍是 v4 `CanSession`**，与 v5 板子**不兼容**；台架用 `tools/canopen_bringup.py`（原始 SocketCAN 主站）。生产主站 = P3（lely-core） |
| 回滚 | 两 MCU 仓 `cmake -DAPP_USE_CANOPEN=OFF` 即编回 v4 固件（`Core/Src/app_can.c` 保留） |

## 1. 物理层与拓扑

- 单总线，**CAN 2.0A / 11 bit / 1 Mbps / 不开 FD**（F407 bxCAN 限制）。
- 节点：**Jetson = NMT master，node 127**；**底盘 MCU = node 1**；**机械臂 MCU = node 2**。
- 两块 C30D 用**板载收发器**（PD0=CAN1_RX / PD1=CAN1_TX），两个物理端点各 120 Ω，
  **三节点必须共地**（缺地表现为"偶发收到、大量 bus-off"）。
- 位时序：PCLK1 42 MHz / BRP2 / BS1=16 / BS2=4 → 精确 1.000 Mbps，采样点 81 %；
  **时钟必须来自 HSE 8 MHz**（HSI 会偶发 bus-off）。

## 2. 协议栈与单一来源

| 层 | 由谁定义 | 不许手写 |
| --- | --- | --- |
| 协议语义 | CiA 301（EN 50325-4）+ CiA 304（EN 50325-5，安全） | 自造帧表 |
| 对象清单（SSOT） | `docs/canopen/<node>.json` | 直接改 `OD.c/h` |
| EDS | `docs/canopen/<node>.eds`（由 `tools/canopen/gen_eds.py` 从清单生成） | 手改 |
| 固件 OD | `Core/Inc/OD.h` + `Core/Src/OD.c`，由 **CANopenEditor 官方 `CanOpenNodeV4` 导出器**从 EDS 生成 | 手改 |
| 协议栈 | `third_party/canopen/CANopenNode`（v4，含 304）+ `CANopenNode_STM32`，两仓同哈希 | 改栈需记入 `PATCHES.md` |
| 桥接 | `Core/Src/app_canopen.c`（PDO 变量 ↔ app_car/app_arm/app_joint） | — |

重新生成：
```bash
python3 tools/canopen/gen_eds.py docs/canopen/<node>.json tools/canopen/template.eds docs/canopen/<node>.eds
dotnet EDSSharp.dll --infile docs/canopen/<node>.eds --outfile OD.c --type CanOpenNodeV4
cp OD.c Core/Src/OD.c; cp OD.h Core/Inc/OD.h
```

## 3. 通信对象与周期

| 对象 | 周期 | 说明 |
| --- | --- | --- |
| NMT（0x000） | 事件 | 上电停在 **Pre-operational**，Jetson `Start` 后才允许 PDO |
| Heartbeat（0x700+ID） | 1000 ms | 生产者；消费者超时（Jetson 127）P2 配置 |
| SYNC（0x080） | **2 ms** | Jetson 产生，PDO 同步基准 |
| EMCY（0x080+ID） | 事件 | 故障即发，恢复发消 |
| SDO（0x580+ID / 0x600+ID） | 事件 | 参数、诊断、身份；COB-ID 由栈按 node ID 自动偏移 |
| PDO | 见 §4 | 同步发送类型 = 每 N 个 SYNC |

带宽：SYNC 5.5 % + 上/下行 PDO ≈ 9 % + SRDO（P2）≈ 2 % → **总计 ≈ 18 %**，目标 ≤ 25 %。

## 4. PDO 映射（实测速率）

**底盘 node 1**

| PDO | COB-ID | 映射（对象） | 周期 | 实测/3 s |
| --- | --- | --- | --- | --- |
| RPDO1 | 0x201 | `0x2000..0x2003` W1..W4 速令（INT16，0.01 rad/s） | 事件（主站 100 Hz） | — |
| RPDO2 | 0x301 | `0x2040 cmd_seq`(U32) + `0x2041 cmd_flags`(U16) | 事件（100 Hz） | — |
| TPDO1 | 0x181 | `0x2010..0x2013` 四轮实测速 | 每 5 SYNC = 100 Hz | 300 ✔ |
| TPDO2 | 0x281 | `0x2020 W1pos` `0x2021 W2pos`（I32 counts） | 100 Hz | 300 ✔ |
| TPDO3 | 0x381 | `0x2022 W3pos` `0x2023 W4pos` | 100 Hz | 300 ✔ |
| TPDO4 | 0x481 | `0x2030..0x2034` IMU（dYaw/pitch/roll/evt/age） | 100 Hz | 300 ✔ |

**机械臂 node 2**

| PDO | COB-ID | 映射 | 周期 | 实测/3 s |
| --- | --- | --- | --- | --- |
| RPDO1 | 0x202 | `0x2100..0x2103` J1..J4 令（U16，0.1°） | 事件（50 Hz） | — |
| RPDO2 | 0x202→0x302 | `0x2104..0x2107` J5/J6/夹爪/保留 | 事件（50 Hz） | — |
| RPDO3 | 0x402 | `0x2140 cmd_seq` + `0x2141 cmd_flags` | 事件（50 Hz） | — |
| TPDO1 | 0x182 | `0x2110..0x2113` J1..J4 回读 | 50 Hz | 150 ✔ |
| TPDO2 | 0x282 | `0x2114..0x2117` J5/J6/夹爪/保留 | 50 Hz | 150 ✔ |
| TPDO3 | 0x382 | `0x2120..0x2124` IMU | 100 Hz | 300 ✔ |
| TPDO4 | 0x482 | `0x2130 state` `0x2131 fault` `0x2132 mode` `0x2133 errcnt` | 10 Hz | 30 ✔ |

未使用的 PDO 槽位（底盘 RPDO3/4、机械臂 RPDO4）**必须**把 COB-ID 置 bit31=1（禁用），
见 §7。

## 5. 命令活性与安全语义

- **deadman = 命令序号 + 100 ms**：`cmd_seq` 在 100 ms 内必须变化，且 `cmd_flags.bit0=1`，
  且节点处于 Operational，否则底盘 `enable=0`（松链接、零目标）、机械臂 `arm_set_remote(0)`。
  不依赖 SYNC 是否在场。
- **CAN 不是急停主路径**：急停必须走硬线双通道 → 安全继电器/接触器断动力，MCU 本地 ≤1 ms 关 PWM。
  Jetson 在安全链外，其指令按非安全处理。SRDO（CiA 304）在 P2 接两 MCU 之间的安全状态协同。
- 桥接可观测量（SDO）：`0x2050 cmd_en`、`0x2051 cmd_age_ms`、`0x2052 car_state`、
  `0x2053 car_mode`；机械臂 `0x2150 cmd_en`、`0x2151 cmd_age_ms`、`0x2154 joint_fault`、
  `0x2155 joint_zeroed`。
- 身份：`0x2F00 tag_packed`（build_tag 前 4 字符小端打包，实测 `52 37 33`="R73"）、
  `0x2F01 reset_cause`（'S'/'I'/'P'/'B'）。

## 6. 台架验收（不依赖 Jetson 的 rak 栈）

```bash
python3 tools/canopen_bringup.py --can can0 --seconds 3      # NMT Start + SYNC + RPDO + 速率统计 + SDO 读数
```
判据：心跳 `701/702=05`；§4 的实测列速率；`cmd_en=1`（`cmd_flags=1` 时）；
`ip -details link show can0` 的 berr-counter tx 0 / rx 0。

## 7. 上游坑（必须知道，否则板子"看起来死了"）

1. **未使用的 RPDO/TPDO 不置 bit31 → 节点永远回不到 Operational**：零映射 + 合法 COB-ID
   被判定 `CO_EM_PDO_WRONG_MAPPING` → Error register → `CO_NMT_ERR_ON_ERR_REG` 把节点
   从 Operational 踢回 Pre-operational，TPDO 全灭。`gen_eds.py` 已强制生成。
2. **CanOpenSTM32 的 RX 软件过滤器倒序遍历** → 空的 0x1016 心跳消费槽位（`ident=0`）
   会截走 NMT `0x000`，NMT Start 永不生效。本仓改为正向遍历（`third_party/canopen/PATCHES.md` §4）。
3. **OD 必须按索引升序**（CANopenNode 用二分查找）。EDS 段落顺序决定生成顺序，
   `gen_eds.py` 已整体重排；门禁 `canopen_gate.sh` 会校验 `ODList` 升序与 `OD_ENTRY_H*` 索引一致。
4. **VISIBLE_STRING（0x1008/0x1009/0x100A）当前不可用**：经官方导出器生成的字符串对象
   会让两板在 `canopen_app_init` 阶段 HardFault（实测 PC 停在 `HardFault_Handler`）。
   身份改用 `0x2F00/0x2F01` 整型对象；字符串对象留待上游修复或改用 DOMAIN 方案。

## 8. v4 与 v5 的关系

- v4 规范件保留在 `protocol.md` / `PROTOCOL.md`（各仓），**只作为回滚分支的文档**，不再演进。
- `docs/protocol-signals.yaml`、`docs/gen/can_protocol.h`、`tools/can_check.py` 只服务 v4；
  编 v5 时不需要过 `can_check`，改走 §6 的 CANopen 门禁。
- **禁止同总线混跑**：v5 的 0x100（TIME 段）/0x201/0x210/0x220… 与 v4 的 DRIVE_CMD、
  WHEEL_POS、DIAG 等 ID 直接冲突。切换 = 两板 + 上位机一起换；回滚同理。
