# vehical_rak_motion — 底盘板固件（STM32F407VET6 / C30D）

**四轮麦轮底盘**：四路 AT8236 电机驱动 + 四路正交编码器 + SSD1306 OLED + 板载 IMU，
四轮 **25 kHz 硬实时速度闭环**，由 Jetson 上位机经 **CAN v4 @1 Mbps** 控制。

> **2026-10-28 恢复双板架构**：机械臂从本板**迁出**，改由独立板 `vehical_rak_arm`
> （git 仓名历史遗留为 `vehical_rak_edge_ai`）承担 6 路舵机。本板**只做底盘**：
> 4 轮闭环 + IMU + CAN。协议 **ID 与字节布局一字未动**（Jetson 侧零改动）。
> 姊妹仓 = `vehical_rak_edge_ai`（**机械臂板**，同硬件、不同角色）；上位机 = `VANexus/rak`（Jetson）。

条款与操作流程看本仓 [`AGENTS.md`](AGENTS.md) 与 [`CLAUDE.md`](CLAUDE.md)；
协议看 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)（规范件）+
[`docs/protocol-signals.yaml`](docs/protocol-signals.yaml)（机器可读单一来源）；
板级真相看 [`docs/BOARD-C30D.md`](docs/BOARD-C30D.md)。**新成员先读 [`docs/QUICKSTART.md`](docs/QUICKSTART.md)**。

---

## 0. 双板架构（2026-10-28 恢复）

一条 CAN 总线（CAN 2.0A，1 Mbps）挂三个节点，帧 ID 全局唯一：

```
              CAN 总线 1 Mbps
   Jetson（rak）┬── 0x100 DRIVE_CMD / 0x010 SAFETY_CMD ── 底盘板 vehical_rak_motion（本仓）
                ├── 0x110/0x111 ARM_CMD_A/B / 0x010 ──── 机械臂板 vehical_rak_arm
                ├── 0x200/0x201/0x202 轮遥测 ←────────── 本仓
                └── 0x210/0x211/0x221/0x031 臂遥测 ←──── 机械臂板
```

- 本仓 = **底盘板**：4 轮 25 kHz 闭环 + IMU；上行 `0x200/0x201 WHEEL_POS`、`0x202 WHEEL_SPD`、
  `0x220 DIAG`、`0x030 NODE_INFO`；下行消费 `0x100 DRIVE_CMD` + `0x010 SAFETY_CMD`。
- 舵机帧 `0x110/0x111` 由机械臂板落地，**本板不再消费**（同总线 ID 唯一 → Jetson 零改动）。
- 架构分层见工作区 [`docs/architecture.md`](../docs/architecture.md)。

---

## 1. 当前状态：R64（底盘板，双板架构）

| 项 | 状态 |
| --- | --- |
| CubeMX 生成 | ✅ FW_F4 V1.28.3，TargetToolchain=CMake；HSE 8 MHz 时钟树 + TIM7 时基；外设均在 `MX_*_Init()`（R20 起不再手写寄存器） |
| Debug / Release 构建 | ✅ 双预设零警告 |
| 应用代码 | ✅ R64：4 轮 25 kHz 闭环 + 四路编码器 + IMU IIR + CAN v4 主链路 + OLED + ADC 电池 + IWDG；**R64 删舵机**（`app_servo`）、删 `0x110/111` 消费与 `0x210/211` 上行 |
| 上板运行 | ✅ 历史版本已上板（R5/R49-R57 烟测记录见附录）；**R64 待上板复测** |
| CAN 物理联调 | ⏳ 与机械臂板 / Jetson 三节点联调待做（2026-10-29 起） |
| ⚠️ 已知问题 | **左前轮 `allow_motion=true` 后自转不停**（GitHub issue [#8](https://github.com/VANexus/rak/issues/8)）——见 [`docs/KNOWN-ISSUE-runaway-wheel.md`](docs/KNOWN-ISSUE-runaway-wheel.md) |

`build_tag` 单点定义在 [`Core/Src/app_car.c`](Core/Src/app_car.c) 顶部，OLED 显示。

---

## 2. 外设与模块

| 资源 | 配置 |
| --- | --- |
| 电机 PWM ×4 | A: PB8/PB9(TIM10/11)　B: PE5/PE6(TIM9)　C: PE9/PE11(TIM1_CH1/2)　D: PE13/PE14(TIM1_CH3/4)；**板载 AT8236 双输入，无 STBY**（`IN1=IN2=1` 刹车、`=0` 滑行） |
| 编码器 ×4 | A: PA15/PB3(TIM2)　B: PB4/PB5(TIM3)　C: PB6/PB7(TIM4)　D: PA0/PA1(TIM5)，硬件正交 ×4 |
| 控制 ISR | **TIM6 @25 kHz**（APB1 定时器 84 MHz，ARR=3359）；四路测速 + 定点 PI，ISR 单一写者 |
| CAN1 | **PD0=RX / PD1=TX，板载收发器**，1 Mbps（PCLK1 42 MHz / BRP2 / BS1=16 / BS2=4，采样点 81.0 %） |
| OLED / 按键 / LED | OLED 软 SPI(PD11-14)、USER 键 PE0、LED PE8 |
| ADC1 / IWDG | PB0=IN8 电池电压（100k:10k 分压）；IWDG ~2 s |
| 时钟 | **HSE 8 MHz** → 168 MHz；HAL 时基 TIM7，SysTick 归 FreeRTOS |
| 调试口 | USART1 PA9/PA10（TYPE-C），USB v2 会话为**备用链路**（帧格式冻结） |

模块（`Core/Src/`）：`app_car.c`（电机/编码器/25 kHz ISR/ADC/IWDG/OLED/IMU 集成）、
`app_wheel_control.c`（轮级 PD 稳定）、`app_can.c`（CAN v4 编解码/任务）、
`app_imu.c`、`app_oled.c`、`app_serial*.c` + `app_protocol_v2.c`（USB v2 备用）、
`app_sign_store.c`（encoder_sign 持久化）、`app_can_control_policy.c`（安全门）。

> 编码器常数：**61 600 计数/输出轴圈**（A 路手转实测 61 602；理论 512×4×30 = 61 440；
> B/C/D 未分别标定）。电机型号 **MG513P30_12V**（30:1，带编码器）。

---

## 2.1 IMU 6 轴扩展（R65，2026-10-29）

- 原来只读陀螺 Z（稳定器用）；R65 起同一个 14 字节块读里带上**加速度 3 轴**：
  新增倾角（pitch/roll）、冲击事件、帧间偏航增量。**陀螺 Z 路径与稳定器/航向逻辑一字未动。**
- 新增 CAN `0x032 CHASSIS_IMU`（100 Hz，纯加法）：`DYAW(mrad)/PITCH(0.01°)/ROLL(0.01°)/
  EVT/AGE`；`EVT.bit1 = slip` 沿用 R58 的 `dbg_slip` 判据（|陀螺偏航率 − 轮速偏航率| 超阈）。
- 采样率上限来自**软 I2C 100 Hz 轮询**；要升到 200–400 Hz 必须换硬件 I2C2 + DMA
  （PB10/PB11 = AF4，需 CubeMX 重生成；当前 `~/STM32Cube/Repository` 缺 FW_F4 包，阻塞）。
- 三颗 IMU 全部在车体上 → 底盘这颗与机械臂板那颗是**同源冗余**（角速度与安装位置无关），
  Jetson 侧可用两帧互校判“谁坏了”。

## 3. 构建与烧录（Windows / Linux 同一套命令）

```bash
. ./env/activate.sh          # Windows: . .\env\activate.ps1（只需 STM32CubeCLT 1.22.0）
cmake --preset Debug   && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
# → build/<预设>/vehical_rak_motion.{elf,hex,bin} + .map

STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_motion.hex -v -rst
```

提交前本仓必须过：`python3 tools/can_check.py`（动了协议）+ `cmake --build --preset Debug/Release`（零警告）
+ `bash tools/cubemx_lint.sh`（动了外设）。**跨端比对用 `.bin`/`.hex` 的 SHA-256**，不要比 `.elf`。

---

## 4. 通信协议

**CAN v4 是唯一主链路**（规范 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)，内容 FROZEN）。
`0x010 SAFETY_CMD` 是广播 e2e 帧：急停（cmd=4）在 ISR 快速路径锁存；清除（cmd=5）经安全门落地。
`0x100 DRIVE_CMD` 断流 100 ms → 目标清零 + `STATE.bit3`。

**不要自己手写帧表**：`python3 tools/can_check.py --c-header` 生成 `docs/gen/can_protocol.h`；
提代码前 `can_check.py` 必须退出码 0。

---

## 5. 本工程硬规矩（摘要，权威见 [`AGENTS.md`](AGENTS.md)）

1. **25 kHz 硬实时闭环**：ISR 内禁浮点/除法/`printf`/阻塞；占用率必须实测（≤ 35 %）。
2. **上电不得自动驱动电机**；电机启动只能来自显式 `START` + `DRIVE_CMD`；测试先架空车轮。
3. **1 Mbps CAN 必须 HSE 8 MHz**——别被 CubeMX 重生成抹回 HSI。
4. 上板必须**真断电冷启动**并以 OLED `build_tag` 判别版本；每次有效改动升一档。
5. 外设配置一律走 CubeMX（[`docs/CUBEMX-SOP.md`](docs/CUBEMX-SOP.md)）；应用层只写逻辑。
6. 板级/协议共享常量**单点定义**；AI 注释以 `[AI]` 前缀标记。
7. 25 kHz 能否出扭矩有一条**未裁决的冲突记录**（台架说只有 10 kHz 能出扭矩）——
   实测不通过就停下上报，**不许擅自改频率**（见附录「待裁决风险项」）。

---

## 6. 参考资料（只读）

**唯一来源**：`vehical_rak_edge_ai/docs/WHEELTEC L150Pro 智能小车资料_V2.1_2026.04.20.zip`（656 MB，未入 git）。

- `9.原理图/C30D 原理图和资源分配表/`：**引脚与硬件事实的最终出处**。
- `8.芯片手册/`：`VP230.pdf`（板载 CAN 收发器）、`D版_AT8236.pdf`、`D版_STM32F407VET6.PDF`、IMU 手册。
- `6.STM32源码/`、`5.C30D V2.0基础实验例程/`：轮趣官方 CubeMX 代码与例程。

> ⚠️ `car2/` 系列（台架归档）**已于 2026-10-14 弃用移除**。由它得出的台架期结论
> （静摩擦 75%、滑行 0.10–0.11 圈、25 kHz 出扭矩疑问等）一律降级为「**转述、待重新实测**」。

---

# 附录：历史与未决问题

## 待裁决风险项

- **PWM 频率 25 kHz vs 10 kHz**：设计规定 25 kHz；台架期有一条「只有 10 kHz 能出扭矩」的
  记录（原出处 `car2/` 已移除 → 转述、无法复核）。→ 首次上板用示波器 + 带载实测裁决；
  不通过就上报决策，**不许擅自降频**。
- **静摩擦起步**：约 75 % 占空比才能挣脱（转述、待重测）→ H7 方案：四路同拍 KICK 75 % 强踢后交 PI。
- **刹车滑行**：约 0.10–0.11 圈（转述、待重测）→ `brake_lead` 定心。

## R64 / R62-R63（2026-10-28 双板架构）

- **R64**：舵机迁到机械臂板；删 `app_servo`（6 路舵机）、删 CAN `0x110/111 ARM_CMD` 解析、
  删 `0x210/211 SERVO_STATE` 上行；本板只留 4 轮闭环 + IMU + CAN。协议 ID/布局未变。
- **R63**：`ARM_CMD` 换算乘法改 `int64` 中间量（R62 因子放大后 int32 在 raw>1230° 溢出回绕）。
- **R62**：修 CAN `ARM_CMD` 量化因子 10× 错（R49 起 0.1°→urad 写成 ×174.533）。

## R58-R61：CAN v4 协议冻结

- **R61**：协议内容冻结（驱动门自诊断位图并入 STATE bit6-10，`docs/PROTOCOL.md` 标 FROZEN）；
  急停全权走 CAN（R59 按钮 PE0 硬件故障禁用）。
- **R58/R59**：驱动门自诊断位图；PE0 按钮实测卡死恒低 → 禁用，急停置位/清除全走 CAN。

## R49-R57：CAN v4 主链路与修复史

CAN 全面接管控制与遥测（协议 v4），USB v2 冻结为备用。
烟测已验证：遥测 7 帧全速率、急停 CAN 锁存/ISR 快速路径、CAN 清急停（R50/R52）、
驱动链路（轮位增量与 SWD 直读 TIM CNT 逐位一致）、deadman、过滤器自愈（R51）。

- **R51 过滤器自愈 @1Hz**：RX 静默死亡根因 = 运行时 FA1R.FACT0 被清 + FFA1R.FFA0 被置。
- **R52 CAN 清急停跳过 PD3 门**：实测断电后 PD3 读高，把按钮+CAN 清除全拒；远程清除语义要求跳过。
- **R54 回退 R53 栈扩容**：4096×4 打爆 FreeRTOS 堆 → 任务创建失败 → 800 ms 复位循环。
- **R55**：FreeRTOS 堆 15360→32768（R53 失败真因 = 堆耗尽）。
- **R57 重大更正**：此前「过滤器寄存器被篡改」是**误判**——过滤器偏移用错（FFA1R 实为 0x214、
  FA1R 实为 0x21C）。按正确偏移复核：**过滤器与 IER 从未被篡改，一直配置正确**（R51 自愈每秒空跑，无害）。

## 仍未解（历史）

- **START/CLEAR（cmd=1/5）在新 boot 上不被执行**（cmd=4 急停正常，同一 ISR 同一 CRC 门）。
  仍未有定论；进一步用 gdb 断点（需 ELF 符号）+ 逻辑分析仪抓 `0x010` 线上真实字节。
- **estop 幽灵再锁存**：CAN 流量期间 estop 数分钟级被重新锁存（已排除串口/残帧/按钮）。
  规避：出现锁存后用 CAN `CLEAR_ESTOP`（`010#050000000000001C`）1 秒内可恢复。

---

*最后更新：2026-10-28（R64：恢复双板架构，本板收敛为底盘）。*
