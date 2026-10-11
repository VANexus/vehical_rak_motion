# vehical_rak_motion — 底盘 + 机械臂运动控制固件（STM32F407VET6 / C30D）

> 架构依据：[`docs/ROS2-CONTROL-ARCH.md`](docs/ROS2-CONTROL-ARCH.md)。
> 臂板烧毁后只剩本板，目标由上位机经 `ros2_control` 控制 4 轮、6 轴机械臂
> 和独立舵机夹爪；CAN v3 仅留作双板时期的备用协议。
> 本 README 后续板级/构建资料仍有历史状态，协议现状以
> [`docs/UART-v2-ALIGN.md`](docs/UART-v2-ALIGN.md) 为准。

四轮麦轮底盘板：四路 AT8236 电机驱动 + 四路正交编码器 + SSD1306 OLED。
当前固件仍实现 UART v1 体速度控制；目标控制架构和 UART v2 规范见
[`docs/ROS2-CONTROL-ARCH.md`](docs/ROS2-CONTROL-ARCH.md) 与
[`docs/UART-v2-ALIGN.md`](docs/UART-v2-ALIGN.md)。CAN v3 是双板时期的备用协议，
不属于当前 Jetson 链路。v2 目前只有独立 codec，尚未接入控制 endpoint。

行为约束看本仓 [`AGENTS.md`](AGENTS.md)，架构与当前 UART 目标协议分别见
[`docs/ROS2-CONTROL-ARCH.md`](docs/ROS2-CONTROL-ARCH.md) 与
[`docs/UART-v2-ALIGN.md`](docs/UART-v2-ALIGN.md)。CAN v3 规范仅适用于历史双板总线。

**新成员直接从 [`docs/QUICKSTART.md`](docs/QUICKSTART.md) 开始**（装环境 → 构建 → 烧录 → 首次上板清单 → 常见坑）。

### 仓库结构与文档布局

本目录是**独立 git 仓**（可单独 clone、单独构建，`Drivers/` 与 `Middlewares/` 已纳入版本库）：

```
vehical_rak_motion/
├── README.md              本文件：现状、外设待配、模块划分
├── docs/QUICKSTART.md     快速上手（先读这个）
├── docs/QUICKSTART.md      快速上手（跨端）
├── docs/PROTOCOL.md        CAN 协议 v3（规范）
├── docs/protocol-signals.yaml  协议的机器可读定义（信号库）
├── docs/BOARD-C30D.md      板级引脚与外设资源（硬件真相）
├── docs/gen/can_protocol.h 由 can_check.py --c-header 生成的帧宏
├── tools/can_check.py      协议校验器 / 参考编解码 / 测试向量
└── AGENTS.md               本仓工作约束（agent 与人都读）
└── Core/ Drivers/ Middlewares/ cmake/ CMakeLists.txt …
```

本仓**自包含**：单独 clone 就能编译、也能跑协议校验，不依赖上层目录。

约定很简单：`docs/PROTOCOL.md`、`docs/BOARD-C30D.md`、`docs/protocol-signals.yaml`、`tools/can_check.py` 在**两个 MCU 仓各存一份**，改动时同一批把另一仓也改掉（`cp` 一下就行，不引同步工具）；`docs/gen/can_protocol.h` 是 `can_check.py --c-header` 的产物，**不手改**。

---

**远端**：`https://github.com/VANexus/vehical_rak_motion`（private，默认分支 `main`，团队 `RAK-嵌入式开发组` 有 maintain 权限）。
协作三步：**开工先 `git pull`** → 改动 → **收工 `git add -A && git commit && git push`**；提交前本仓必须 `python3 tools/can_check.py` 与 `cmake --build --preset Debug` 双双零问题。

---

## 1. 当前状态：R5（板上运行中）—— 全栈移植 + HSE/TIM7 + boot 解耦 + 按键修复 + 精简

> 移植与版本细节见交接文档（`../car/README4.md`）。业务代码在 `Core/Src/app_car.c`（电机/编码器/OLED/ADC/IWDG/TIM6 25kHz ISR/舵机/串口）与 `Core/Src/app_can.c`（CAN v3）。

| 项 | 状态 |
| --- | --- |
| CubeMX 生成 | ✅ FW_F4 V1.28.3，TargetToolchain=CMake；R2/R3 验证通过（时基 TIM7 + HSE 8MHz 时钟树；R3 修复 boot 对 HAL tick 的依赖） |
| Debug / Release 构建 | ✅ 双配置零警告（Windows 实测，Debug FLASH 36.4KB / RAM 45.3KB） |
| 应用代码 | ✅ R5：25kHz 闭环 + CAN v3 + 串口遥控 + 台架自检（OLED/舵机已移除；寄存器级，LL 裁决见交接文档 §六） |
| 上板运行 | ✅ R5 已上板运行（2026-10-03：uwTick 单调无复位；台架确认进行中） |
| CAN 软件栈 | ✅ 端到端验证通过（SWD 回环注入 0x100 → 应用层收到；TX/引脚/1Mbps 正常） |
| CAN 物理联调 | ❌ 当前总线无活跃对端（ESR ACK 错误稳态）——待对方板/接线（清单见交接文档 §二C） |
| 外设配置 | MX 侧仅 FREERTOS/NVIC/RCC/SYS（时基 TIM7）；**电机/编码器/CAN/OLED/ADC/IWDG 由应用代码寄存器级实现**（不走 MX，防重生成覆盖） |

---

## 2. 构建与烧录（Windows / Linux 完全同一套命令）

```bash
# ① 加载共享工具链（STM32CubeCLT 1.22.0）
. ./env/activate.sh          # Windows: . .\env\activate.ps1

# ② 构建
cmake --preset Debug && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
# 产物：build/<预设>/vehical_rak_motion.{elf,hex,bin} + .map
# hex/bin 由顶层 CMakeLists.txt 的 POST_BUILD 生成（CubeMX 不生成这两样）

# ③ 烧录（两端同一个 CLI）
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_motion.hex -v -rst
```

`build/`、`cmake-build-*/` 属生成物，已在根 `.gitignore` 里排除。
Linux 上 udev 规则（`49-stlinkv2*.rules`）已存在，`dialout` 组已加入，插拔 ST-LINK 后无需额外配置。

---

## 3. 骨架里已经有什么

| 内容 | 值 / 位置 |
| --- | --- |
| IP | `FREERTOS`、`NVIC`、`RCC`、`SYS`（仅 4 个，见 `.ioc` 的 `Mcu.IPNb`） |
| FreeRTOS | CMSIS-OS **v2**，tick 1 kHz，`configTOTAL_HEAP_SIZE=15360`（heap_4），`configMAX_PRIORITIES=56`，只有一个 `defaultTask`（栈 128 字，normal 优先级）→ `Core/Src/freertos.c` |
| 时钟 | **HSE 8MHz 晶振 + PLL**（8/4×168/2）→ SYSCLK 168 MHz，APB1 /4（定时器 84 MHz）、APB2 /2（定时器 168 MHz），`FLASH_LATENCY_5` → `main.c` 的 `SystemClock_Config()`。R2 起时钟树已用 HSE（AGENTS 硬约束，CAN 上总线前提）；`car_init()` 保留 HSE→168MHz 运行时兜底 |
| HAL 时基 | **TIM7 @1kHz**（R2 起，`stm32f4xx_hal_timebase_tim.c`），SysTick 归 FreeRTOS；TICK_INT_PRIORITY=15（CubeMX 默认） |
| 调试 | SWD 已开（PA13/PA14） |
| HAL 模块 | 仅基集：CORE/RCC/GPIO/DMA/EXTI/PWR/FLASH。`stm32f4xx_hal_conf.h` 里 TIM/CAN/ADC/IWDG/UART **都是注释状态** |
| newlib 线程安全 | `STM32_THREAD_SAFE_STRATEGY=4` + `newlib_lock_glue.c` + `stm32_lock.h`（CubeMX 生成，勿手删） |
| 链接脚本 | `STM32F407xx_FLASH.ld`：FLASH 512 K / RAM 128 K / CCMRAM 64 K，`_estack` = RAM 顶 |
| GPIO 初始化 | `MX_GPIO_Init()` 只使能 GPIOH/GPIOA 时钟，**按键、LED、OLED 引脚一个都没配** |

⚠️ 本工程与 `vehical_rak_edge_ai`（姊妹仓，VANexus 下）同属 C30D 板但角色不同：本仓跑**底盘 + 机械臂运动控制**（CAN 主链路），edge_ai 仓跑**边缘 BNN/SNN 推理**（CAN + SPI 屏 + 单路电机/编码器）。两仓共享板级/协议文档，但 `.ioc` 与外设集不同。

---

## 4. 待补清单（按依赖顺序）

### Phase A：外设（CubeMX 里配，配完重新生成 + 编译）

> **R1/R2 实施路线变更**：电机/编码器/CAN/OLED/ADC/IWDG/舵机/串口已由 `app_car.c`/`app_can.c` **寄存器级实现**（不在 MX 配置，重生成不影响）；MX 侧只配了时基与 RCC。下表保留原 MX 路线方案作参考；**RCC 与 TIM7 两行已完成**。

| 外设 | 目标 | 备注 |
| --- | --- | --- |
| TIM1 CH1/2/3/4 | 电机 C(PE9/PE11) D(PE13/PE14) PWM | **25 kHz** → APB2 定时器 168 MHz 下 `ARR = 6719`；AT8236 双输入：`IN1=IN2=1` 刹车、`=0` 滑行 |
| TIM9 CH1/2 | 电机 B(PE5/PE6) PWM | 25 kHz（APB2 定时器同 168 MHz） |
| TIM10 CH1 / TIM11 CH1 | 电机 A(PB8/PB9) PWM | 25 kHz |
| TIM2/3/4/5 编码器模式 | A(PA15/PB3) B(PB4/PB5) C(PB6/PB7) D(PA0/PA1) | 硬件正交 ×4，61 600 计数/输出圈（A 路实测，B/C/D 未分别标定） |
| 控制 ISR 定时器（TIM6） | 25 kHz，与 PWM 同频 | **TIM6 在 APB1→定时器 84 MHz，所以 `ARR = 3359`**（与 PWM 定时器的 6719 不同值）；四路测速 + 定点 PI + 刷新，**占用率必须实测**（预算 ≤ 6 720 周期的 35%）；ISR 内不调任何 FreeRTOS API |
| CAN1 | **PD0=RX / PD1=TX，板载收发器**（资源 PDF 确认）；1 Mbps | ⚠️ 草案写的 `PB8/PB9` **作废**（那是电机 A 的驱动输入）。另：骨架时钟源当前是 **HSI**，**必须先改成 HSE 8 MHz**，否则 1 Mbps 会偶发错误帧（见 [`docs/PROTOCOL.md`](docs/PROTOCOL.md) §2） |
| GPIO | PE0 USER、PE8 LED、PD11-14 OLED 软 SPI、PD3 使能开关 | |
| ADC1 | PB0 = IN8 电池电压，100k:10k 分压 | |
| IWDG | 约 2 s，由状态任务喂 | |
| USART1 | PA9/PA10（TYPE-C，CH9102F） | 仅调试输出，不参与实时控制（与草案一致，此处草案没错） |
| TIM7 | HAL 时基（1 kHz）；SysTick 已让给 FreeRTOS | ✅ **R2 已配**（SYS Timebase=TIM7，`stm32f4xx_hal_timebase_tim.c`） |
| **RCC** | **HSI → HSE 8 MHz**（PH0/PH1 已有晶振） | ✅ **R2 已配**（HSE+PLL→168MHz；位时序按 42 MHz PCLK1 / BRP=3 / BS1=11 / BS2=2 得到 85.7 % 采样点） |

**PWM / 控制频率 = 25 kHz，这是设计规定**（见 `docs/PROTOCOL.md`（见 docs/PROTOCOL.md 与 AGENTS.md） §5）。但台架期有一条记录写"10 kHz 是唯一验证过能让桥臂出扭矩的频率，25 kHz 试过发不出扭矩"（`../car2/four-motor-sync/main.c:5-8`）。把它当**风险项**，不当改频率的理由：配置完就带载 + 示波器实测；不通过就**停下上报并决策**（驱动能力问题），**不许擅自改成 10 kHz 交差**。

### Phase B：应用模块（按 `docs/PROTOCOL.md`（见 docs/PROTOCOL.md 与 AGENTS.md） §3 分层）

| 层 | 模块 | 放哪里 |
| --- | --- | --- |
| L1 驱动 | `app_motor`（方向/刹车/滑行）、`app_encoder`、`app_oled`、`app_batt`、`app_watchdog` | **`../platform/`**（两端共用，Phase 3 建立） |
| L2 实时 | `app_loop`：25 kHz ISR → 1 ms 窗 M 法测速 → 定点 PI（限幅 85%）+ 起步同拍强踢 | **`../platform/`** |
| L3 解算 | `app_mecanum`：`(Vx,Vy,ωz)` → 四轮速度目标（含符号表与 `IK_PERMM` 常数） | 本工程 `Core/Src/`（底盘专属） |
| L4 安全 | `app_supervise`：命令解析、100 ms 生命期、急停锁存、模式机、遥测、台架测试 | **`../platform/`** |
| L5 通信 | `app_can`：帧编解码 + CRC8 + RX 队列 + 1 kHz/100 Hz 节拍 | **`../platform/`** |
| 板级差异 | `board_c30d.h`：引脚/定时器号/通道/计数常数（**一个量只允许一个常数**） | 本工程 `Core/Inc/`（各工程一份） |

⚠️ `platform/` 里的代码不得 include 本工程专属头文件；先写在本工程，Phase 3 再抽（但**第 1 步就按可抽的形状写**，别写成依赖 `main.h` 宏的硬编码）。

任务表：`Task_Control`(100 Hz) / `Task_CAN_Rx` / `Task_CAN_Tx`(1 kHz) / `Task_Status`(10 Hz，喂狗+OLED) / `Task_Debug`，控制环放 TIM ISR 内不进调度。

### Phase C：联调

按 [`docs/PROTOCOL.md`](docs/PROTOCOL.md) v3 跑：物理层（**两端各 120 Ω + 共地 + CANH/L 极性**）→ `0x100` 速度流 + `0x200` 回读 + CRC/counter 校验 → 100 ms deadman（底盘减速停）→ `0x010` 急停锁存/解除 → `0x030/0x031` 1 kHz 互锁与 10 ms 超时 → IK 三参数标定（轮径、`lx+ly`、每轮符号）→ 增益整定。

**不要自己手写帧编解码**：跑 `python3 ../tools/can_check.py --c-header` 生成 `../docs/gen/can_protocol.h`（含 ID / Data ID / 周期宏、CRC8 参考实现、16 bit 小端读写），接入顶层 `CMakeLists.txt` 的 include 路径；提代码前 `can_check.py` 必须退出码 0。

---

## 5. 本工程的四条硬规矩

1. **只在 `USER CODE BEGIN xxx` … `USER CODE END xxx` 标记对之间写**，标记外是 CubeMX 的地盘，重生成会被抹掉。
2. 重生成后必查三处：`stm32f4xx_it.c` 里 SVC/PendSV/SysTick 三个 handler 不能被恢复（由 FreeRTOS 提供）；`cmake/stm32cubemx/CMakeLists.txt` 会被整体覆盖（自定义内容一律加在**顶层** `CMakeLists.txt`）；`SystemClock_Config` 的 HSE/HSI 与 `.ioc` 是否一致（历史上出过 `RCC_HSE_OFF` 导致黑屏）。
3. 烧完必须**真断电冷启动**（电池关 → 拔 ST-LINK → 等 OLED 全黑 → 上电），并以屏上 `build_tag` 判别实际运行版本；软复位可能仍在跑旧指令。
4. 上电**不得自动驱动电机**；测试先架空车轮。

## 6. 参考资料（只读）

- `README3.md`（car2 台架归档，未纳入本仓）：H7 交接文档，§5 调参速查、§6 硬件接口速查、§8 踩坑记录是密度最高的部分。
- `docs/hardware.md`（car2 台架归档，未纳入本仓）：引脚映射与其出处（`bsp_c30d.c` + 原理图 PDF）。
- `four-motor-25k/main.c`（car2 台架归档，未纳入本仓）：裸机版电机/编码器/OLED/ADC 寄存器操作参考（**金标准台架代码**，移植时改成 HAL，别直接抄进本工程）。
- `../car2/tools/references/smart-transport-robot/`：同板完整课设（底盘 `chassis/mecanum.c`、`motor_ctrl.c`、`calib.c`，IMU 打法）。

*最后更新：2026-10-03（R5：上板运行 + CAN 软栈验证；取证/修复/清单见 `../car/README4.md` §二/§二C）。改动构建方式或外设结论时请同步本文件。*

## R49-R54（2026-10-09）：CAN v4 主链路

CAN 全面接管控制与遥测（协议 v4，见 docs/PROTOCOL.md），USB v2 冻结为备用。

**烟测已验证**：遥测 7 帧全速率、急停 CAN 锁存/ISR 快速路径、CAN 清急停（R50/R52）、
驱动链路（轮位增量与 SWD 直读 TIM CNT 逐位一致）、deadman、过滤器自愈（R51）。

### 修复史
- **R51 过滤器自愈 @1Hz**：RX 静默死亡根因 = 运行时 FA1R.FACT0 被清 + FFA1R.FFA0 被置
  （帧被 ACK 但被过滤器拒收不进 FIFO，SWD 三次稳定读数实锤），自愈整组重配
- **R52 CAN 清急停跳过 PD3 门**：实测断电后 PD3 读高，把按钮+CAN 清除全拒；
  开关藏车内难达，远程清除语义要求跳过（保留电池+非 FAULT+fast_req 门）
- **R54 回退 R53 栈扩容**：4096×4 打爆 FreeRTOS 堆 → 任务创建失败 → 800ms 复位循环

### 修复更正（R55-R57）

- **R55**: FreeRTOS 堆 15360→32768（R53 栈扩容失败的真因=堆耗尽，任务创建失败→800ms 复位循环）
- **R57 重大更正**：此前"过滤器寄存器被篡改"是**误判**——自查 CMSIS 头发现过滤器偏移
  用错（FFA1R 实为 0x214、FA1R 实为 0x21C，此前误读 FS1R/保留区）。按正确偏移复核：
  **过滤器与 IER 从未被篡改，一直配置正确**。R51 自愈因此每秒空跑（无害），R57 已修正地址。
- **仍未解**：START/CLEAR（cmd=1/5）在新 boot 上不被执行（cmd=4 急停正常，同一 ISR 同一
  CRC 门），且无 estop 锁存干扰时也复现。cmd=4 与 cmd=1/5 的差异是唯一线索。
  下一步：gdb 断点 app_can.c case 1（需 ELF 符号）+ 逻辑分析仪抓 0x010 帧线上的真实字节。



**estop 幽灵再锁存**：CAN 流量期间 estop 数分钟级被重新锁存（3+ 次复现），
静默 2 分钟不复发。已排除：串口路径（latch_generation=0）、Jetson 残帧、
按钮长按（PE0 实测高）。怀疑方向：栈溢出踩踏（R53 扩栈尝试因堆耗尽回退，
需改用 FreeRTOS heap 扩容 + uxTaskGetStackHighWaterMark 实测）或
car_estop_fast_req 的未知置位者（gdb 硬件观察点 0x200051C1 可抓）。
规避：出现锁存后用 CAN CLEAR_ESTOP（010#050000000000001C）1 秒内可恢复。
