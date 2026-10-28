# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 先读

本仓是 **C30D 板（STM32F407VET6 + 板载 AT8236）四轮麦轮底盘固件**。硬性约束、安全红线、署名约定都在 [`AGENTS.md`](AGENTS.md)——**那份是权威，本文件不重复其条款，只讲构建与架构**。开工前按顺序读：

1. [`AGENTS.md`](AGENTS.md) — 硬约束（25 kHz 实时性、上电不驱动、HSE、冷启动、注释署名）
2. [`docs/QUICKSTART.md`](docs/QUICKSTART.md) — 装环境 → 构建 → 烧录 → 首次上板清单
3. [`docs/BOARD-C30D.md`](docs/BOARD-C30D.md) — **引脚与硬件真相**（板载 CAN 在 PD0/PD1；AT8236 无 STBY；无电流采样）
4. [`docs/PROTOCOL.md`](docs/PROTOCOL.md) + [`docs/protocol-signals.yaml`](docs/protocol-signals.yaml) — CAN 协议 v3
5. [`docs/ROS2-CONTROL-ARCH.md`](docs/ROS2-CONTROL-ARCH.md) — **进行中的架构变更**（见下）
6. [`docs/CUBEMX-SOP.md`](docs/CUBEMX-SOP.md) — **改任何外设/配置前必读**：外设配置一律走 CubeMX（含无头命令参考、生成后必查清单、反模式）

姊妹仓 `vehical_rak_edge_ai`（git 仓名历史遗留）是同一块板的**机械臂板 `vehical_rak_arm`** 角色（6 路舵机）；上位机是 `VANexus/rak`（Jetson）。**2026-10-28 恢复双板架构**：舵机已从本板迁到机械臂板（R64），本板收敛为 pure 底盘。

## 常用命令

```bash
. ./env/activate.sh                                  # 每次开工（Windows: . .\env\activate.ps1）
cmake --preset Debug   && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 \
    -w build/Debug/vehical_rak_motion.hex -v -rst

python3 tools/can_check.py            # 协议门禁：结构校验 + 向量比对 + CRC 负例，退出码 0 才算过
python3 tools/can_check.py --update   # 协议改动后回写 YAML 的 expect 字段
python3 tools/can_check.py --c-header # 生成 docs/gen/can_protocol.h（勿手改）

bash tools/cubemx_lint.sh             # 外设配置门禁：应用层出现寄存器级外设初始化就报错（见 docs/CUBEMX-SOP.md）
```

- 只依赖 **STM32CubeCLT 1.22.0**（含 GCC 14.3 / CMake / Ninja / Programmer），`env/activate.sh` 负责探测并前置 PATH。
- **没有单元测试框架**；唯一的自动化门禁是 `can_check.py`（需要 PyYAML）。动了协议就必须跑它。
- 提交前：Debug + Release **零警告** + `can_check.py` 通过。
- 上板后**必须真断电冷启动**（电池关 → 拔 ST-LINK → 等 OLED 全黑 → 再上电），看屏上 `build_tag` 判别实际在跑哪版。
- 主机侧联调工具：`tools/uart_v1_test.py`（协议镜像 + golden vector 自检）、`tools/chassis_host.py`（100 Hz 控制类）。

## 架构

### 启动链（`Core/Src/main.c`）

`HAL_Init` → `SystemClock_Config`（HSE 8 MHz + PLL → 168 MHz，APB1 /4、APB2 /2）→ `MX_GPIO_Init` → 应用初始化三连 `car_init` / `can_init` / `serial_init` → `osKernelInitialize` / `MX_FREERTOS_Init` / `osKernelStart`。

HAL 时基是 **TIM7 @1 kHz**，SysTick 归 FreeRTOS。

**外设配置一律由 CubeMX 生成（R20 起）**：`.ioc` 里启用 TIM1/2/3/4/5/6/8/9/10/11/12、CAN1、USART1/3(+DMA)、ADC1、IWDG、DMA，`stm32f4xx_hal_conf.h` 对应模块已打开，初始化落在生成的 `tim.c` / `can.c` / `usart.c` / `adc.c` / `iwdg.c` / `dma.c` / `gpio.c` 里，`main.c` 依次调用 `MX_*_Init()`。**应用层只写逻辑，不再手写寄存器。**

唯一例外是 **25 kHz ISR 的函数体**（`app_car.c` 的 `car_control_isr()`，AGENTS.md §1 的时序预算吃不下 HAL 回调链）——但它也只被生成的 `TIM6_DAC_IRQHandler` 通过 `USER CODE` 块调用，**外设本身仍由 CubeMX 配**。

> 改外设、改配置、加外设 → **必读 [`docs/CUBEMX-SOP.md`](docs/CUBEMX-SOP.md)**（含无头 CubeMX 命令参考）。
> 门禁：`bash tools/cubemx_lint.sh`（应用层出现寄存器级外设初始化就报错）。

### 任务与实时环（`Core/Src/freertos.c`）

| 线程 | 优先级 | 频率 | 职责 |
| --- | --- | --- | --- |
| `defaultTask` | Normal | 循环 | 空转（R41 起舵机测试代码已移除） |
| `ctrl` | High | 100 Hz | `car_control_step()`：监督器 |
| `canrx` | AboveNormal | 阻塞于队列 | CAN 收帧解析 |
| `cantx` | Normal | 1 kHz | CAN 发送 + `serial_poll_tx` |
| `stat` | BelowNormal | 10 Hz | 喂 IWDG + OLED 刷新 |

**25 kHz 闭环不在调度器里**，在 `app_car.c` 的 `TIM6_DAC_IRQHandler`：编码器计数 → 每 25 拍（1 ms）M 法测速 + 定点 PI → 刷 PWM。ISR 内禁止浮点/除法链/`printf`/阻塞调用；**转速、位置、占空比由 ISR 单一写者持有，任务只读**。ISR 占用率预算 ≤ 周期 35 %（改动后要实测）。

### 模块划分（`Core/Src`）

- `app_car.c`（最大，核心）— 电机/PWM/编码器/ADC/IWDG 的**运行时控制**（初始化已在生成的 `tim.c` / `adc.c` / `iwdg.c` 里，本文件只做 `HAL_TIM_PWM_Start` / `HAL_TIM_Encoder_Start` / `HAL_ADC_*` / `HAL_IWDG_Refresh` 与 `__HAL_TIM_SET_COMPARE`）；TIM6 **25 kHz 闭环 ISR（`car_control_isr`，函数体寄存器级）**；100 Hz 监督器状态机（IDLE/REMOTE/ESTOP/FAULT/CALIBRATING + 台架自检）；麦轮 IK（`ik_wheel_target`）与前向运动学；IMU 自稳定（`stabilizer_apply`）；按键生产语义（R43：短按=清急停锁存、长按=本地急停；**R59/R61 起 PE0 按钮硬件故障禁用，急停全权走 CAN——v4 冻结语义**）；`build_tag`。顶层常量（PWM_HZ、`ENCODER_COUNTS_PER_REV`、`IK_PERMM`、`forward_dir`、限幅、PI 增益）都在本文件头部。
- `app_can.c` — **CAN v4 帧编解码（主链路，协议已冻结）** + CRC8(e2e) + RX 队列；下行 `0x010 SAFETY`(e2e，急停 ISR ≤40 µs 快速路径 + START/STOP/CLEAR_ESTOP)、`0x100 DRIVE_CMD`、`0x110/111 ARM_CMD`；上行 `0x200/201 WHEEL_POS`+`0x202 WHEEL_SPD`+`0x210/211 SERVO_STATE`+`0x220 DIAG`+`0x030 NODE_INFO`；过滤器/RX 通知 1 Hz 运行时自愈、邮箱 ABRQ 自愈；驱动门自诊断位图在 STATE bit6-10。协议真源 `docs/protocol-signals.yaml`（v4 FROZEN）。
- `app_serial.c` — 团队 UART v1：**COBS + CRC16/CCITT-FALSE**，921600 8N1，USART1/USART3；解码在 USART IRQ 逐字节跑，TX 走 TXE 中断续发；lease 看门狗 + 故障位图 + `ID:A` 识别横幅。同文件承载 **USB v2** 会话（R23 起）：HELLO/feature 协商、序号窗口、lease 看门狗、安全三命令、`DRIVE_COMMAND`（4 轮 urad/s）/`ARM_COMMAND`（7 轴 µrad）/遥测/诊断（**R49 起 USB v2 冻结为备用链路，CAN v4 为主**；会话细节见 `app_serial_session.c`、编解码 `app_protocol_v2.c`，对端实现见 rak 仓 `src/rak_chassis`）。
- `app_imu.c` — 软 I2C 陀螺仪，ICM20948 / MPU6050 双芯片自动探测，未找到则静默降级；只服务**轮级**自稳定（航向权威在上位机）。
- ~~`app_servo.c` — 6 路舵机~~：**R64 已删除并从本板迁到机械臂板 `vehical_rak_arm`**（`app_servo` / `app_arm` / `app_build` / `app_can` / `app_oled`）。本板不再驱动舵机、不再消费 CAN `0x110/0x111`、不再发 `0x210/0x211`。
- `app_oled.c` — SSD1306 软 SPI 调试显示，惰性初始化。

数据流：下行 `CAN 0x100 / UART SET_BASE_VELOCITY` → `car_can_velocity` → 稳定器 → IK → 四路 `v_cmd` → 25 kHz ISR 闭环；上行 `wheel_speed` → 前向运动学 → `0x200` / `BASE_STATE`。

### 共享文件的单点定义与双仓同步

- **协议三件套**（`docs/PROTOCOL.md`、`docs/protocol-signals.yaml`、`docs/gen/can_protocol.h`）与 `docs/BOARD-C30D.md`、`tools/can_check.py`、`env/activate.*` 在**两个 MCU 仓各存一份**。改任一份时**同一批提交两个仓**（手工 `cp`，无同步工具），并在提交说明写明。`docs/gen/can_protocol.h` 是生成物，不手改。
- 一个物理量只允许一个常数（引脚/定时器号/编码器计数），定义在板级头文件或 `app_car.c` 顶部；`docs/BOARD-C30D.md` 是引脚真值。参考课设与 AI 草案里的 TB6612 / STBY / PB8-PB9 当 CAN / 电流保护**都是错的**。

### CubeMX 边界（重生成会覆盖）

- 只往 `/* USER CODE BEGIN xxx */ … /* USER CODE END xxx */` 之间写。
- 新 `.c/.h` 加进**顶层** `CMakeLists.txt` 的 `target_sources` / `target_include_directories`；`cmake/stm32cubemx/CMakeLists.txt` 会被整体覆盖。
- 重生成后必查：`stm32f4xx_it.c` 不得恢复 SVC/PendSV/SysTick 三个 handler；`SystemClock_Config` 的 HSE 是否被抹回 HSI。

## 注释与版本约定

- AI/工具产出的注释一律加 `[AI]` 前缀（`grep -rn "\[AI\]" Core/` 可列出全部）；**用户原有注释永不删除或改写**，需要调整先征询。
- `build_tag`（`app_car.c`）每有有效改动升一档；板子在跑哪版以屏上标签为准，不以你烧了什么为准。

## 未裁决 / 进行中的事项

- **25 kHz vs 10 kHz 冲突**：台架有一条记录说只有 10 kHz 能出扭矩，25 kHz 试过不出。这是**未裁决风险项**——实测不通过就停下上报，**不许擅自改频率**。
- **ros2_control 架构变更（CAN 主链路已落地，2026-10-09 冻结）**：控制契约改为 **4 轮速 + 7 舵机位置（6 关节 + 1 夹爪）**，**麦轮 IK 移到上位机**；板载 IMU 收窄为轮级。**固件侧已完成**：USB v2 会话/安全门、轮速闭环与轮级等比饱和、**CAN v4 主链路（R49-R61，协议已冻结——急停全权走 CAN，按钮 PE0 硬件故障禁用；驱动门自诊断在 STATE bit6-10）**；**R64 起舵机迁到机械臂板 `vehical_rak_arm`**（本板只留 4 轮 + IMU）。**剩余在上位机（均不需要动协议）**：ros2_control 臂 7 轴 position 接口 + `controller_manager`/spawner + TUI CAN 后端。协议真源 `docs/PROTOCOL.md` + `docs/protocol-signals.yaml`（**FROZEN，修订须走提案流程**）。详见 [`docs/ROS2-CONTROL-ARCH.md`](docs/ROS2-CONTROL-ARCH.md)。
- **UART lease 200 ms 偏离规范 [10,50]**：电机 PWM 运行时 USART ORE 丢字节导致 50 ms 必超时（R18 把串口 IRQ 优先级 7→4 后 ORE 降 84 %）。当前实发 200 ms，**待团队签字**。详见 [`docs/UART-v1-ALIGN.md`](docs/UART-v1-ALIGN.md)。
