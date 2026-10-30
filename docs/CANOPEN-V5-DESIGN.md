# CAN v5 — CANopen 精简栈 + CiA 304 安全通信（设计规格）

- 状态：方向已定（2026-10-29，用户拍板 A 方案：CANopen 精简栈），进入落地
- 适用：Jetson（非安全域）↔ 底盘 MCU（node 1）↔ 机械臂 MCU（node 2），单条经典 CAN 1 Mbps
- 取代：`docs/protocol.md` v4（v4 保留在 git 历史与回滚标签，不与 v5 同总线混跑）
- 单一来源：本规格 + 每节点 EDS/OD；`docs/protocol-signals.yaml` 与 `tools/can_check.py` 在 v5 切换完成后退役

## 1. 目标与非目标

**目标**
1. 协议层使用工业标准 CANopen（CiA 301）子集：NMT、Heartbeat、EMCY、SDO、SYNC、LSS；不再手写帧表与编解码。
2. 安全通信使用 CiA 304 SRDO（EN 50325-5，IEC 61784-3 黑通道 profile），目标 SIL 2 能力。
3. 工程流程企业化：EDS 单一来源、OD 代码生成/校验、CI、单元测试、vcan/HIL、MISRA C:2012 静态检查、需求→对象→测试追溯。
4. 两端（两块 F407 + Jetson）使用同一套对象语义与同一份 EDS 校验；Jetson 侧用现成 CANopen 主站库（lely-core / python-canopen 先做台架）。
5. 为功能安全认证准备证据链：安全计划、SRDO 验证、故障注入、安全反应时间实测、安全手册。

**非目标**
- CAN FD / CANopen FD（F407 bxCAN 不支持；换 MCU 才能做）。
- CiA 402 每轴驱动子协议（本系统是“一个 MCU 管多轴”，不是每电机一个 CAN 节点）。
- Jetson 进入安全链（Linux/非实时，明确列为非安全相关）。
- 用软件替代安全接触器/STO（硬件缺口见 §8）。

## 2. 拓扑与角色

```
                    CAN 1 Mbps / 11-bit / 经典帧
   Jetson (node 127) ────────┬──────────────────────┐   非安全域
   NMT Master / SYNC / SDO   │                      │
                     底盘 MCU (node 1)        机械臂 MCU (node 2)
                     4 轮 25 kHz 闭环          6 舵机 + J1/J2 电机
                           │                      │
                     硬线急停 A              硬线急停 B
                           └──── 双通道急停回路 ────┤
                                                  │
                                    安全继电器/接触器（切动力）
```

- Jetson：NMT 主站、SYNC 产生者、SDO 客户端、PDO 生产者/消费者；不参与任何安全功能。
- 底盘 MCU：4 轮速度闭环 + 底盘 IMU + 本地安全状态 + 硬线急停输入 + SRDO 生产/消费。
- 机械臂 MCU：J1/J2 闭环 + J3..J6 舵机 + 臂板 IMU + 本地安全状态 + 硬线急停输入 + SRDO 生产/消费。
- 上电默认：两块 MCU 进入 Pre-operational，安全状态 = 停止；Jetson 发 NMT Start 后才进入 Operational。

## 3. 协议基线

| 项 | 规定 |
| --- | --- |
| 物理层 | ISO 11898-2，1 Mbps，双绞线，两端 120 Ω，三节点共地 |
| 帧 | CAN 2.0A，11-bit，DLC≤8，不开 FD |
| 应用层 | CANopen CiA 301 子集（EN 50325-4） |
| 安全层 | CiA 304 SRDO（EN 50325-5），IEC 61784-3 黑通道 |
| 节点 ID | 底盘 1，机械臂 2，Jetson 127（NMT 主站） |
| 心跳 | 生产者 1000 ms；消费者（Jetson）超时 500 ms；MCU 互相消费超时 1500 ms |
| SYNC | 周期 2 ms（500 Hz），Jetson 产生；PDO 同步发送 |
| SDO | 标准 SDO server（expedited + segmented）；Jetson 侧 SDO client |
| EMCY | 标准紧急帧；节点故障立即发，恢复后发 EMCY 清除 |
| LSS | 支持 LSS slave（节点 ID/波特率现场配置），生产中可关 |
| 标识 | 0x1008 设备名、0x1009 硬件版本、0x100A 软件版本（build tag）、0x1018 身份 |

### COB-ID 规划（CiA 301 预定义连接集）

| 功能 | 底盘（1） | 机械臂（2） | Jetson（127） |
| --- | --- | --- | --- |
| NMT | 0x000（收） | 0x000（收） | 0x000（发） |
| SYNC | 0x080（收） | 0x080（收） | 0x080（发） |
| EMCY | 0x081（发） | 0x082（发） | 0x08F（收） |
| TIME | 0x100（收，可选） | 0x100（收，可选） | 0x100（发，可选） |
| RPDO1 | 0x201 | 0x202 | — |
| RPDO2 | 0x301（预留） | 0x302 | — |
| TPDO1 | 0x181 | 0x182 | — |
| TPDO2 | 0x281 | 0x282 | — |
| TPDO3 | 0x381 | 0x382 | — |
| TPDO4 | 0x481 | 0x482 | — |
| SDO tx | 0x581 | 0x582 | — |
| SDO rx | 0x601 | 0x602 | — |
| Heartbeat | 0x701 | 0x702 | 0x77F |
| SRDO（安全） | 0x500（发）/0x510（收） | 0x510（发）/0x500（收） | 不参与 |

> SRDO 的 COB-ID 与参数由 OD 0x1300/0x1380 配置（CiA 304），上表是工程约定；需在 EDS 中固化并做冲突检查。
> 0x500/0x510 不与节点 1/2 的预定义 RPDO/TPDO 冲突（RPDO4 为 0x501/0x502）。

## 4. 对象字典与 PDO 映射

### 4.1 标准对象（每节点必备）

0x1000 Device type、0x1001 Error register、0x1003 Predefined error field、0x1005 SYNC COB-ID、0x1006 Communication cycle、0x1007 Synchronous window、0x1008 Manufacturer device name、0x1009 Hardware version、0x100A Software version、0x1010 Store parameters、0x1011 Restore default、0x1017 Heartbeat producer、0x1018 Identity、0x1019 SYNC counter overflow、0x1400/0x1600 RPDO1、0x1800-0x1803/0x1A00-0x1A03 TPDO1-4、0x1016 Heartbeat consumer、0x1200/0x1280 SRDO 参数。

### 4.2 厂商对象

**底盘（node 1）**

| 索引 | 名称 | 类型 | 访问 | PDO |
| --- | --- | --- | --- | --- |
| 0x2000 | WheelSpeedCmd | ARRAY[4] INT16，0.01 rad/s | RW | RPDO1 |
| 0x2001 | WheelSpeedFb | ARRAY[4] INT16，0.01 rad/s | RO | TPDO1 |
| 0x2002 | WheelPosA | ARRAY[2] INT32，counts | RO | TPDO2 |
| 0x2003 | WheelPosB | ARRAY[2] INT32，counts | RO | TPDO3 |
| 0x2004 | ChassisImu | RECORD：dYaw INT16 mrad、pitch INT16 0.01°、roll INT16 0.01°、evt UINT8、age UINT8 ms | RO | TPDO4 |
| 0x2005 | ChassisStatus | RECORD：state UINT16、fault UINT16、mode UINT8、gate UINT8 | RO | SDO/EMCY |
| 0x2006 | ChassisDiag | RECORD：bat_mv UINT16、busoff UINT8、tec UINT8、rec UINT8、lec UINT8、uptime UINT32 | RO | SDO |
| 0x2007 | WheelConfig | RECORD：kp/ki/kd、限幅、极性 | RW | SDO |

**机械臂（node 2）**

| 索引 | 名称 | 类型 | 访问 | PDO |
| --- | --- | --- | --- | --- |
| 0x2100 | ArmCmdA | ARRAY[4] UINT16，0.1°（J1-J4） | RW | RPDO1 |
| 0x2101 | ArmCmdB | ARRAY[4] UINT16，0.1°（J5,J6,GRIP,保留） | RW | RPDO2 |
| 0x2102 | ServoStateA | ARRAY[4] UINT16，0.1° | RO | TPDO1 |
| 0x2103 | ServoStateB | ARRAY[4] UINT16，0.1° | RO | TPDO2 |
| 0x2104 | ArmImu | RECORD：gz INT16 0.01°/s、amag UINT16 mg、evt UINT8、age UINT8、reserved UINT16 | RO | TPDO3 |
| 0x2105 | ArmStatus | RECORD：state UINT16、fault UINT16、mode UINT8、servo_errcnt UINT8 | RO | SDO/EMCY |
| 0x2106 | ArmDiag | RECORD：can_tec/rec/lec、impact_cnt、uptime | RO | SDO |
| 0x2107 | JointConfig | RECORD：PI 增益、软限位、归零偏移 | RW | SDO |

**公共安全对象**

| 索引 | 名称 | 类型 | 访问 | 说明 |
| --- | --- | --- | --- | --- |
| 0x2200 | SafetyState | UINT16：bit0 estop_latched、bit1 safe_state、bit2 peer_ok、bit3 local_estop、bit4 fault | RO | SRDO |
| 0x2201 | SafetyConfig | UINT16：SRDO 周期、超时、反应时间预算 | RW | SDO |
| 0x2202 | SafetyCounter | UINT8：SRDO 滚动计数 | RO | SRDO |
| 0x2F00 | BuildInfo | VISIBLE_STRING（如 R70-canopen-v5） | RO | SDO |

### 4.3 PDO 映射与周期

| 节点 | PDO | 映射 | 传输类型 | 实际周期 |
| --- | --- | --- | --- | --- |
| 底盘 | RPDO1 | 0x2000[0..3] | 255（事件） | Jetson 100 Hz |
| 底盘 | TPDO1 | 0x2001[0..3] | 5（每 5 个 SYNC） | 100 Hz |
| 底盘 | TPDO2 | 0x2002[0..1] | 5 | 100 Hz |
| 底盘 | TPDO3 | 0x2003[0..1] | 5 | 100 Hz |
| 底盘 | TPDO4 | 0x2004[0..4] | 5 | 100 Hz |
| 机械臂 | RPDO1 | 0x2100[0..3] | 255 | Jetson 50 Hz |
| 机械臂 | RPDO2 | 0x2101[0..3] | 255 | Jetson 50 Hz |
| 机械臂 | TPDO1 | 0x2102[0..3] | 10 | 50 Hz |
| 机械臂 | TPDO2 | 0x2103[0..3] | 10 | 50 Hz |
| 机械臂 | TPDO3 | 0x2104[0..4] | 5 | 100 Hz |
| 机械臂 | TPDO4 | 0x2105[0..3] + 0x2106 摘要 | 50 | 10 Hz |

状态/诊断的完整细节走 SDO 轮询（Jetson 1 Hz）与 EMCY；PDO 只保留实时必需的 8 字节。

### 4.4 带宽核算（1 Mbps，经典帧约 110–130 µs）

- SYNC 500 fps ≈ 5.5%
- 底盘 TPDO 4×100 = 400 fps ≈ 4.4%
- 机械臂 TPDO 2×50 + 100 + 10 = 210 fps ≈ 2.3%
- 下行 RPDO 100 + 2×50 = 200 fps ≈ 2.2%
- SRDO 2 对 × 50 Hz（20 ms）≈ 2.2%
- Heartbeat/EMCY/SDO ≈ <1%
- **合计 ≈ 18%，目标 ≤25%，保留 ≥4 倍余量。**

## 5. 安全架构（认证边界）

- **急停主路径不走 CAN**：双通道硬线急停 → 安全继电器/接触器断电机动力；两块 MCU 各自 ≤1 ms 关 PWM + 刹车。CAN SRDO 只做状态协同、跨节点安全停止提示与诊断。
- **安全状态定义**：接触器断开 + PWM 关闭 + 刹车使能 + 舵机保持（按安全评估最终定义）。
- **SRDO**：两 MCU 互发 SafetyState（20 ms 周期，正反双帧 + CRC + 时间监控）；超时/CRC/反转不一致 → 本地安全状态 + EMCY。
- **Jetson 非安全**：Jetson 的 START/STOP/运动指令一律按非安全处理；急停权限不依赖 Jetson。
- **安全反应时间预算**：硬线回路 ≤20 ms；MCU 本机关断 ≤1 ms；SRDO 超时判故障 ≤40 ms（20 ms 周期 + 20 ms 超时）。
- **认证口径**：CAN 通道按 IEC 61784-3 黑通道 + CiA 304，目标 SIL 2 能力；整机 PL d/SIL 2 需补齐 §8 硬件后再评估。

## 6. 硬件缺口（认证前置条件，软件不可替代）

| 缺口 | 现状 | 要求 |
| --- | --- | --- |
| 安全转矩关断/接触器 | AT8236 无 STO，无动力接触器 | 增加安全继电器/接触器（带辅助反馈），由双通道急停回路控制 |
| 急停输入 | 无独立双通道急停输入（PE0 是用户键/已故障） | 每板 2 路独立急停输入 + 不一致检测 |
| 外部看门狗 | 仅 F407 内部 IWDG | 增加外部窗口看门狗（或安全继电器带看门狗） |
| 功率级诊断 | 无电流采样、无输出回读 | 增加母线/相电流或输出回读用于安全诊断 |
| 编码器冗余 | 每轮单编码器 | 若安全功能含“安全限速/位置”，需第二通道或安全评估论证 |

> 在上述硬件补齐前，可交付的是「通信通道 SIL 2 能力 + 安全状态机 + 证据框架」，不是整机证书。

## 7. 工具链与工程流程

**现成栈（不手写协议）**
- MCU：CANopenNode v4（Apache-2.0，含 CiA 304 SRDO，MISRA C:2012，CiA 一致性测试通过）+ CANopenNode_STM32 官方 HAL 移植层。
- Jetson：lely-core（Apache-2.0，CANopen master/client）；台架先用 python-canopen + SocketCAN。
- OD/EDS：EDS 为单一来源；用 CANopenEditor 生成 OD.c/h 与文档；CI 用 `canopen` Python 库解析 EDS 并比对 OD 映射与测试向量。
- 测试：pytest + python-can + vcan（协议/超时/故障注入）；台架 HIL（Jetson can0 + 两块 C30D）。
- 静态检查：`cppcheck --addon=misra` + 现有 `-Wall -Wextra -Werror`；第三方栈按其自带 MISRA 声明单独管理。
- CI：两仓 Debug/Release 零警告 + 主机侧协议测试 + EDS 校验；产物 SHA-256 存档。

**仓库结构（每块 MCU）**
```
third_party/canopen/CANopenNode/        # 上游源码，固定版本，Apache-2.0
third_party/canopen/CANopenNode_STM32/  # 官方 HAL 移植层
Core/Inc/OD.h  Core/Src/OD.c            # 本节点 OD（由 EDS 生成/校验）
Core/Src/app_canopen.c                  # 与 app_* 的桥接（PDO 变量 ↔ 控制环）
docs/canopen/<node>.eds                 # EDS 单一来源
tools/canopen_*.py                      # 主站/验收脚本
```

**版本固定**
- CANopenNode 固定到上游 commit（记录在 `third_party/canopen/VERSION`）。
- 两块 MCU 使用同一份 CANopenNode/CANopenNode_STM32 副本哈希（CI 校验）。

## 8. 迁移与切换

1. v4 与 v5 的 CAN ID 冲突（0x100 TIME vs v4 DRIVE_CMD、0x201/0x202 vs CANopen RPDO/TPDO），**禁止同总线混跑**。
2. 切换窗口：同时烧录两块 MCU + 更新 Jetson 的 `CanopenSession`；v4 固件保留 tag/分支用于回滚。
3. 上电顺序：Jetson 先起 NMT/SYNC → 两块 MCU 进 Pre-operational → Jetson NMT Start → 校验心跳/SDO/PDO。
4. v4 的 `docs/protocol.md`、`protocol-signals.yaml`、`can_check.py` 在 v5 验收后归档，不再演进。

## 9. 阶段与验收

**P0 基础（本阶段）**
- 栈入库（两仓同哈希）+ 编译链接通过 + 两板能启动到 Pre-operational + 心跳可见 + SDO 读写正常。
- 验收：Jetson/python-canopen 能 NMT Start、读 0x1000/0x1008/0x100A、读写一个测试对象；总线上只有 CANopen 帧。

**P1 实时数据**
- 全部 PDO 映射上线；100/50/10 Hz 周期与带宽达标；驱动/遥测端到端一致。
- 验收：Jetson 侧 PDO 收发计数、周期抖动、CAN 错误计数（ESR/TEC/REC）实测。

**P2 安全**
- SRDO 上线；安全状态机 + 硬线急停输入（硬件到位后）；故障注入（丢帧/重复/延迟/位翻转/超时）。
- 验收：安全反应时间实测 ≤ 预算；SRDO 超时/不一致触发安全状态；EMCY 正确。

**P3 Jetson 集成与切换**
- lely-core 集成到 `rak_chassis`（`CanopenSession` 与现有 `Session` API 对齐）；ROS 2 接口不变。
- 验收：ros2_control 通过 CANopen 跑通四轮 + 七轴；v4 帧为零；回滚演练通过。

**P4 认证证据**
- 安全计划、需求追溯、FMEA、SRDO 验证报告、安全手册、工具资格；生产切换认证商业栈（ISIT COSAFE / SYS TEC / MicroControl）并完成栈资格。

## 10. 待决/采购项

1. **认证商业栈**：生产安全节点建议 ISIT COSAFE（CiA 301 V4.2 + CiA 304，ST 合作伙伴，有认证版本）或 SYS TEC/MicroControl；需询价与 license。
2. **CiA 厂商 ID**：0x1018 需要注册的 Vendor ID；未注册前用 0x00000000 并在 EDS 标注。
3. **CANopenEditor**：生成 OD.c/h 需 .NET/Windows 环境；本机 Linux 先以 EDS + CI 校验，生成环境待定（用户 Windows 侧或 CI runner）。
4. **硬件安全包**：接触器/安全继电器、双通道急停、外部看门狗、功率级诊断（见 §6）。
5. **Jetson 主站选型**：生产用 lely-core；台架先用 python-canopen。`ros2_canopen` 以 CiA 402 为主，不直接适用自定义 OD。

## 11. 参考

- CiA 301 / EN 50325-4 CANopen application layer
- CiA 304 / EN 50325-5 CANopen Safety（SRDO）
- IEC 61784-3 Functional safety fieldbuses（black channel）
- IEC 61508-2 §7.4.11 数据通信要求
- CANopenNode v4: https://github.com/CANopenNode/CANopenNode
- CanOpenSTM32: https://github.com/CANopenNode/CanOpenSTM32
- CANopenEditor: https://github.com/CANopenNode/CANopenEditor
- lely-core: https://github.com/lely-industries/lely-core

## 12. 落地进度（2026-10-29 实测，非计划）

**P0 栈接入 ✅**：两板 CANopenNode v4（node 1 / node 2，1 Mbps）上电进 Pre-operational，
1 Hz 心跳 `0x701/0x702 = 7F`；NMT Start 后变 `05`；SDO 双向实测
（`601#40001000…` → `581#43 00 10 00 …`，node 2 同理 `602/582`）。

**P1 PDO 全链路 ✅**（底盘 23 对象 / 机械臂 31 对象，Jetson 原始 SocketCAN 主站实测 3 s）：

| COB-ID | 实测帧数/3s | 设计值 |
| --- | --- | --- |
| `0x181 0x281 0x381 0x481` | 300 / 300 / 300 / 300 | 100 Hz ✔ |
| `0x182 0x282` | 150 / 150 | 50 Hz ✔ |
| `0x382` | 300 | 100 Hz ✔ |
| `0x482` | 30 | 10 Hz ✔ |

RPDO → OD → 控制层实测（`cmd_flags=1` + 命令序号滚动）：`0x2050 cmd_en=1`、
`0x2051 cmd_age_ms=90`、`0x2052 car_state=1`（底盘确实进入遥控态）；机械臂
`cmd_flags=0` 时 `0x2150 cmd_en=0`（远程门按语义关断）。`can0` 错误计数 tx 0 / rx 0。

**两个必须记住的坑**（已写入 `third_party/canopen/PATCHES.md` 并由 `gen_eds.py` 强制）：
1. 未使用的 RPDO/TPDO 必须置 COB-ID bit31（disabled），否则零映射被判定
   `PDO_WRONG_MAPPING` → Error register → 节点被踢回 Pre-operational、TPDO 全灭。
2. CanOpenSTM32 移植层 RX 软件过滤器**倒序**匹配，8 个空的 0x1016 槽位
   （`ident=0/mask=0x87FF`）会先截走 NMT `0x000`，NMT Start 永不生效；改正向遍历修复。

**下一步**：① 0x1008/0x1009/0x100A 设备字符串对象未加（SDO 读 0x100A 目前 abort）；
② 机械臂 TPDO3 的 IMU 数值静止态不合理（gz≈69 dps、amag≈62 g），属 A7/A8 应用层，
与 v5 无关，单独排查；③ P2 SRDO + HB consumer 配 Jetson(127) + 故障注入；
④ P3 Jetson lely-core 生产主站；⑤ CI（gen_eds + 官方导出器 + EDS/OD 一致性 + vcan 回归）。
