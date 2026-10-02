# vehical_rak_chassis — 麦轮底盘固件（STM32F407VET6 / C30D）

四轮麦轮底盘板：四路 AT8236 电机驱动 + 四路正交编码器 + SSD1306 OLED + CAN 从机。
上位机（Jetson）经 CAN 以 100 Hz 下发体速度 `(Vx, Vy, ωz)`，本板做逆运动学 + 速度闭环。

行为约束看本仓 [`AGENTS.md`](AGENTS.md)，**分层设计看 `docs/PROTOCOL.md`（见 docs/PROTOCOL.md 与 AGENTS.md）**，**协议看 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)**。本文件只讲**这个工程**。

**新成员直接从 [`docs/QUICKSTART.md`](docs/QUICKSTART.md) 开始**（装环境 → 构建 → 烧录 → 首次上板清单 → 常见坑）。

### 仓库结构与文档布局

本目录是**独立 git 仓**（可单独 clone、单独构建，`Drivers/` 与 `Middlewares/` 已纳入版本库）：

```
vehical_rak_chassis/
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

**远端**：`https://github.com/VANexus/vehical_rak_chassis`（private，默认分支 `main`，团队 `RAK-嵌入式开发组` 有 maintain 权限）。
协作三步：**开工先 `git pull`** → 改动 → **收工 `git add -A && git commit && git push`**；提交前本仓必须 `python3 tools/can_check.py` 与 `cmake --build --preset Debug` 双双零问题。

---

## 1. 当前状态：CubeMX 骨架 + 已打通构建，业务代码 0 行

| 项 | 状态 |
| --- | --- |
| CubeMX 生成 | ✅ FW_F4 V1.28.3，TargetToolchain=CMake |
| Debug / Release 构建 | ✅ **两端命令已在 Linux 实测通过**（下表数据来自实测输出） |
| 上板运行 | ❌ 未做（当前无板子接在这台机器上） |
| 应用代码 | ❌ 未开始，`Core/Src/` 只有 CubeMX 生成文件 |
| 外设配置 | ❌ 只开了 FreeRTOS + SWD，**电机/编码器/CAN/OLED/ADC/IWDG 全部未配** |

实测构建结果（CubeCLT 14.3.rel1，`arm-none-eabi-size`）：

| 预设 | text | data | bss | FLASH 占比 | RAM 占比 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 16 116 | 16 | 43 384 | 16 140 B / 512 KB = 3.08% | 43 400 B / 128 KB = 33.11% |
| Release | 10 224 | 16 | 43 376 | 10 240 B / 512 KB ≈ 1.95% | 同上 |

> RAM 的 43 KB 主要是启动栈 `0x4000` + newlib heap `0x2000` + FreeRTOS `heap_4` 15 360 B，不是应用占用。

---

## 2. 构建与烧录（Windows / Linux 完全同一套命令）

```bash
# ① 加载共享工具链（STM32CubeCLT 1.22.0）
. ./env/activate.sh          # Windows: . .\env\activate.ps1

# ② 构建
cmake --preset Debug && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
# 产物：build/<预设>/vehical_rak_chassis.{elf,hex,bin} + .map
# hex/bin 由顶层 CMakeLists.txt 的 POST_BUILD 生成（CubeMX 不生成这两样）

# ③ 烧录（两端同一个 CLI）
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_chassis.hex -v -rst
```

`build/`、`cmake-build-*/` 属生成物，已在根 `.gitignore` 里排除。
Linux 上 udev 规则（`49-stlinkv2*.rules`）已存在，`dialout` 组已加入，插拔 ST-LINK 后无需额外配置。

---

## 3. 骨架里已经有什么

| 内容 | 值 / 位置 |
| --- | --- |
| IP | `FREERTOS`、`NVIC`、`RCC`、`SYS`（仅 4 个，见 `.ioc` 的 `Mcu.IPNb`） |
| FreeRTOS | CMSIS-OS **v2**，tick 1 kHz，`configTOTAL_HEAP_SIZE=15360`（heap_4），`configMAX_PRIORITIES=56`，只有一个 `defaultTask`（栈 128 字，normal 优先级）→ `Core/Src/freertos.c` |
| 时钟 | **HSI** + PLL → SYSCLK 168 MHz，APB1 /4（定时器 84 MHz）、APB2 /2（定时器 168 MHz），`FLASH_LATENCY_5` → `main.c` 的 `SystemClock_Config()`。`.ioc` 里把 PH0/PH1 设成了 HSE 振荡器模式，但**时钟树没有用 HSE** |
| 调试 | SWD 已开（PA13/PA14） |
| HAL 模块 | 仅基集：CORE/RCC/GPIO/DMA/EXTI/PWR/FLASH。`stm32f4xx_hal_conf.h` 里 TIM/CAN/ADC/IWDG/UART **都是注释状态** |
| newlib 线程安全 | `STM32_THREAD_SAFE_STRATEGY=4` + `newlib_lock_glue.c` + `stm32_lock.h`（CubeMX 生成，勿手删） |
| 链接脚本 | `STM32F407xx_FLASH.ld`：FLASH 512 K / RAM 128 K / CCMRAM 64 K，`_estack` = RAM 顶 |
| GPIO 初始化 | `MX_GPIO_Init()` 只使能 GPIOH/GPIOA 时钟，**按键、LED、OLED 引脚一个都没配** |

⚠️ 本工程与 `vehical_rak_arm`（姊妹仓，VANexus 下） 目前**逐字节同构**：`.ioc` 只差工程名两行，生成的 `.bin` SHA-256 完全相同。机械臂那份不是"另一块板的配置"，只是复制品。

---

## 4. 待补清单（按依赖顺序）

### Phase A：外设（CubeMX 里配，配完重新生成 + 编译）

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
| TIM7 | HAL 时基（1 kHz）；SysTick 已让给 FreeRTOS | |
| **RCC** | **HSI → HSE 8 MHz**（PH0/PH1 已有晶振） | **上总线前必改的第一项**；位时序按 42 MHz PCLK1 / BRP=3 / BS1=11 / BS2=2 得到 85.7 % 采样点 |

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

*最后更新：2026-10-02。改动构建方式或外设结论时请同步本文件。*
