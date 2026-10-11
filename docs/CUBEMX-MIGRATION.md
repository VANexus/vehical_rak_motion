# CubeMX 全量迁移操作清单（R19 → 标准 CubeMX/HAL）

> **给谁看**：由**你**在 CubeMX 里执行；执行完我来改应用层并验证编译。
> **分支**：`cubemx-standard-migration`
> **工具**：`/home/xrak/STM32CubeMX/STM32CubeMX`（**6.18.1**，与本 `.ioc` 匹配）
> ⚠️ **不要用 CubeMX2 1.1.1** —— 那是另一个产品，`.ioc` 格式不同，会改工程布局。
> **固件包**：STM32Cube FW_F4 V1.28.3（已装，与本工程一致）

## 目标

所有外设初始化由 CubeMX/HAL 生成；`app_*.c` 只留**逻辑**（控制环、协议、状态机），不再自己写寄存器初始化。
25 kHz 闭环 ISR 的**函数体**仍保持寄存器级（这是时序预算的要求，不是"非标准"），但它改由生成的 handler 调用。

---

## 0. 开工前

```bash
git checkout cubemx-standard-migration      # 已建好
cp vehical_rak_motion.ioc /tmp/ioc.bak     # CubeMX 会就地改写 .ioc
```

打开 CubeMX → `File > Load Project` → 选 `vehical_rak_motion.ioc`。

**建议按顺序配**：时钟树 → 外设 → NVIC → Project Manager。

---

## 1. Clock Configuration 标签页

时钟树（HSE 8 MHz 已在 `.ioc` 里配好）：

| 项 | 值 |
| --- | --- |
| PLL Source | HSE |
| PLLM | 4 |
| PLLN | 168 |
| PLLP | 2 |
| PLLQ | 7 |
| SYSCLK | 168 MHz |
| AHB Prescaler | /1 → HCLK 168 MHz |
| APB1 Prescaler | /4 → PCLK1 42 MHz（APB1 定时器时钟 84 MHz） |
| APB2 Prescaler | /2 → PCLK2 84 MHz（APB2 定时器时钟 168 MHz） |
| **ADC Prescaler** | **/4** ← ⚠️ 必改。默认 `/2` 会得到 42 MHz，**超出 F407 的 36 MHz 上限** |

> ⚠️ **红线（AGENTS.md §3）**：HSE 必须保持 8 MHz。1 Mbps CAN 依赖它。
> 生成后必查 `SystemClock_Config` 有没有被抹回 HSI。

---

## 2. Pinout & Configuration

### 2.1 定时器

**TIM1 / TIM9 / TIM10 / TIM11 —— 电机 PWM，25 kHz**

| 项 | TIM1 | TIM9 | TIM10 | TIM11 |
| --- | --- | --- | --- | --- |
| 通道 | CH1 CH2 CH3 CH4 | CH1 CH2 | CH1 | CH1 |
| 模式 | PWM Generation CHx（四个都选） | PWM Generation CH1/CH2 | PWM Generation CH1 | PWM Generation CH1 |
| Prescaler | 0 | 0 | 0 | 0 |
| Counter Period (ARR) | **6719** | **6719** | **6719** | **6719** |
| Pulse | **0** | **0** | **0** | **0** |
| PWM Mode | PWM mode 1 | 同 | 同 | 同 |
| CH Polarity | High | 同 | 同 | 同 |
| Auto-reload preload | **Enable** | Enable | Enable | Enable |
| Fast Mode | Disable | Disable | Disable | Disable |
| NVIC | 不勾 | 不勾 | 不勾 | 不勾 |

> `6719 = 168 MHz / 25 kHz - 1`（APB2 定时器时钟 168 MHz）
> **Pulse 必须是 0** —— 上电不得驱动电机（AGENTS.md §2）。

**TIM6 —— 25 kHz 闭环中断源**

| 项 | 值 |
| --- | --- |
| Mode | Clock Source = **Internal Clock** |
| Prescaler | 0 |
| Counter Period (ARR) | **3359** |
| NVIC | ✅ **TIM6 global interrupt**，Preemption Priority = **2**，Sub = 0 |

> `3359 = 84 MHz / 25 kHz - 1`（APB1 定时器时钟 84 MHz）
> ⚠️ CubeMX **不会**启动定时器（`HAL_TIM_Base_Init` 不置 CEN，已核实）。启动仍由
> `car_start_control_isr()` 里的 `HAL_TIM_Base_Start_IT(&htim6)` 负责 ——
> **R3 的"25 kHz 延迟到调度器启动后再开"设计因此得以保留**。

**TIM8 / TIM12 —— 舵机，50 Hz**

| 项 | TIM8 | TIM12 |
| --- | --- | --- |
| 通道 | CH1 CH2 CH3 CH4 → PWM Generation | CH1 CH2 → PWM Generation |
| Prescaler | **167** | **83** |
| Counter Period (ARR) | **19999** | **19999** |
| Pulse | **1500** | **1500** |
| PWM Mode | PWM mode 1 | 同 |
| CH Polarity | High | 同 |
| Auto-reload preload | Enable | Enable |
| NVIC | 不勾 | 不勾 |

> `PSC 167 → 168 MHz/168 = 1 MHz`；`PSC 83 → 84 MHz/84 = 1 MHz`；`ARR 19999 → 20 ms`
> Pulse 1500 = 中位 90°（防上电乱甩）。

**TIM2 / TIM3 / TIM4 / TIM5 —— 正交编码器**

| 项 | TIM2 | TIM3 | TIM4 | TIM5 |
| --- | --- | --- | --- | --- |
| Combined Channels | **Encoder Mode** | Encoder Mode | Encoder Mode | Encoder Mode |
| Encoder Mode | **Encoder Mode TI1 and TI2** | 同 | 同 | 同 |
| Prescaler | 0 | 0 | 0 | 0 |
| Counter Period (ARR) | **4294967295** | **65535** | **65535** | **4294967295** |
| IC1/IC2 Polarity | Rising Edge | 同 | 同 | 同 |
| IC1/IC2 Filter | 0 | 0 | 0 | 0 |
| NVIC | 不勾 | 不勾 | 不勾 | 不勾 |

> TIM2/TIM5 是 32 位计数器（ARR 全 1）；TIM3/TIM4 是 16 位。
> 这是原代码 `t->ARR = (i==1||i==2) ? 0xFFFF : 0xFFFFFFFF` 的对应值。

**TIM7** —— 已在 `.ioc` 里（SYS Timebase），不动。

### 2.2 ADC1

| 项 | 值 |
| --- | --- |
| IN8（PB0） | ✅ 勾选（IN8 Single-ended） |
| Mode | Independent mode |
| Clock Prescaler | **在 Clock Configuration 标签页设 /4**（见 §1） |
| Resolution | 12 bits |
| Scan Conversion Mode | Disabled |
| Continuous Conversion Mode | Disabled |
| DMA Continuous Requests | Disabled |
| End Of Conversion Selection | EOC flag at the end of single conversion |
| External Trigger Conversion Source | **Regular Conversion launched by software** |
| Rank 1 | Channel 8，Sampling Time = **480 Cycles** |
| NVIC | 不勾（轮询） |

> 480 周期是原代码的修复值（100k:10k 分压源阻抗 ~9.1 kΩ）。CubeMX 会自动生成
> `HAL_ADCEx_Calibration_Start()` —— 原代码缺的那一步。
> **注意 CubeMX 默认 Rank 的采样时间不是 480**，要手动改。

### 2.3 IWDG

| 项 | 值 |
| --- | --- |
| Activated | ✅ |
| IWDG counter clock prescaler | **IWDG_PRESCALER_64** |
| Down-counter reload value | **1000** |

> LSI 32 kHz → 超时 1000×64/32000 = **2.0 s**。
> ⚠️ **保持勾选**：CubeMX 生成的是 `HAL_IWDG_Init()`（只配置，不启动）。
> 启动仍是 `HAL_IWDG_Start()`，调用点在 `car_init()` 里、`imu_init()` **之后**
> （IMU 探测最坏 ~7 s，必须等它跑完）。

### 2.4 USART1 / USART3 —— 921600 8N1 + DMA

两个口配置完全相同，只有 DMA 流不同。

**Mode / Parameter Settings**

| 项 | 值 |
| --- | --- |
| Mode | **Asynchronous** |
| Baud Rate | **921600** Bits/s |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Data Direction | Receive and Transmit |
| Over Sampling | 16 Samples |
| Hardware Flow Control | None |

**DMA Settings**（点 `Add` → 从下拉里选，CubeMX 会自动填流/通道）

| 口 | 请求 | Stream / Channel（CubeMX 自动填，仅供核对） | Direction | Mode | Data Width | Priority |
| --- | --- | --- | --- | --- | --- | --- |
| USART1 | `USART1_RX` | DMA2 Stream2 / Ch4 | Peripheral To Memory | **Circular** | Byte / Byte | Medium |
| USART1 | `USART1_TX` | DMA2 Stream7 / Ch4 | Memory To Peripheral | **Normal** | Byte / Byte | Medium |
| USART3 | `USART3_RX` | DMA1 Stream1 / Ch4 | Peripheral To Memory | **Circular** | Byte / Byte | Medium |
| USART3 | `USART3_TX` | DMA1 Stream3 / Ch4 | Memory To Peripheral | **Normal** | Byte / Byte | Medium |

> USART3_RX = DMA1_Stream1/Ch4 已从 ST 官方例程核实；USART1 的两条按 RM0090 Table 43。
> **以 CubeMX 下拉里能选到的为准** —— 它只让选合法组合。

> **RX 为什么是 Circular**：COBS 帧是变长的，靠空闲线（IDLE）断帧。Normal 模式下每次
> IDLE 后 DMA 已停，要重新武装，这个窗口里来的字节会丢（连续噪声下尤其容易）。
> Circular 模式 DMA 永不停止，位置由应用层跟踪、处理环形回绕。**这部分我在应用层做。**
>
> **TX 是 Normal**：TX 是"一帧一个 DMA 传输"，传完由 TC 中断接下一帧。
> 现有的"软件环形缓冲 + TXE 中断续发"（`txq_push` / `serial_poll_tx`）会被换成
> **帧队列 + DMA**，`HAL_UART_TxCpltCallback` 里启动下一帧。
>
> ⚠️ **回调分处两个优先级，别搞混**：
> - **TX 完成回调**跑在 DMA 中断（优先级 **5**）→ 5 ≥ 阈值 5，**可以**调 FreeRTOS API。
> - **RX 空闲/错误回调**跑在 USART 中断（优先级 **4**）→ 4 < 阈值 5，**绝对不能**调
>   FreeRTOS API。它只许写内存（记录长度 + 置标志），解码由 1 kHz 任务做。
> - 所以 `HAL_UART_Transmit_DMA` 的缓冲区在传输完成前**必须一直有效** → 用帧队列，
>   不能用会被覆盖的临时栈缓冲。

**NVIC Settings**

| 项 | 值 |
| --- | --- |
| USART1 global interrupt | ✅ Enable，Priority = **4** |
| USART3 global interrupt | ✅ Enable，Priority = **4** |
| DMA2 stream2 / stream7 global interrupt | ✅ Enable，Priority = **5** |
| DMA1 stream1 / stream3 global interrupt | ✅ Enable，Priority = **5** |

> ⚠️ **优先级 4 是有意的，别改**（R18 的教训：串口 ISR 被 RTOS 临界区推迟 → ORE 丢字节）。
> 代价是：**4 高于 FreeRTOS 的 `configMAX_SYSCALL_INTERRUPT_PRIORITY`(=5)，
> 串口 ISR 和 DMA 空闲回调里绝对不能调任何 FreeRTOS API**。回调只许写内存
> （置长度/标志），解码交给任务。这一点我会在应用层保证。

### 2.5 CAN1

| 项 | 值 |
| --- | --- |
| Master Mode | ✅ |
| Bit Timings: Prescaler | **2** |
| Bit Timings: Time Quanta in BS1 | **16** |
| Bit Timings: Time Quanta in BS2 | **4** |
| Bit Timings: ReSynchronization Jump Width | **1** |
| Automatic Bus-Off Management | Enable |
| Automatic Wake-Up Mode | Enable |
| Automatic Retransmission | Enable |
| Receive Fifo Locked Mode | Disable |
| Transmit Fifo Priority | Disable |
| NVIC | ✅ **CAN1 RX0 interrupt**，Priority = **6** |

> 核对：tq = 2/42 MHz = 47.62 ns，位时间 = (1+16+4)×47.62 = **1000 ns = 1 Mbps**，
> 采样点 (1+16)/21 = **80.95%**。
> 6 低于阈值 5 → CAN ISR **可以**调 FreeRTOS API（现在的 `xQueueSendFromISR` 就靠这个）。
>
> ✅ **数值已交叉核实**：CubeMX 的 "Time Quanta in Bit Segment 1 = 16" 对应 HAL 的
> `CAN_BS1_16TQ` = 寄存器 TS1 字段 = **15**，与现有代码 `BTR = (3<<20)|(15<<16)|(2-1)`
> **逐位等价**（BS2=4→TS2=3、SJW=1→0 同理）。所以填 16/4/1/2 不会改变波特率。

> **走 HAL（已定）**：TX 用 `HAL_CAN_AddTxMessage`（HAL 自动找空邮箱，原来的"三邮箱
> 分派"变成隐式）；RX 在 `CAN1_RX0_IRQHandler` 里用 `HAL_CAN_GetRxMessage`。
> CAN ISR 优先级 6 ≥ 阈值 5 → **可以**继续用 `xQueueSendFromISR`，这个不变。
>
> **Filter Configuration 标签页留空**：接收过滤器（FIFO0、掩码模式、全通）和
> `HAL_CAN_Start()` 我在 `USER CODE` 里配，避免和 CubeMX 生成的过滤器冲突。

### 2.6 GPIO

在芯片图上**右键引脚 → 选功能**。选完 CubeMX 会自动填 AF 号。

| 引脚 | 功能 | 备注 |
| --- | --- | --- |
| PA0 / PA1 | `TIM5_CH1` / `TIM5_CH2` | 编码器 D |
| PA9 / PA10 | `USART1_TX` / `USART1_RX` | |
| PA13 / PA14 | SYS SWDIO / SWCLK | 已配 |
| PA15 | `TIM2_CH1` | 编码器 A |
| PB0 | `ADC1_IN8` | 电池分压 |
| PB3 | `TIM2_CH2` | 编码器 A |
| PB4 / PB5 | `TIM3_CH1` / `TIM3_CH2` | 编码器 B |
| PB6 / PB7 | `TIM4_CH1` / `TIM4_CH2` | 编码器 C |
| PB8 | `TIM10_CH1` | 电机 A PWM+ |
| PB9 | `TIM11_CH1` | 电机 A PWM− |
| PB10 | GPIO_Output，**Open Drain**，No pull，Low speed | IMU 软 I2C SCL |
| PB11 | GPIO_Output，**Open Drain**，No pull，Low speed | IMU 软 I2C SDA |
| PB14 / PB15 | `TIM12_CH1` / `TIM12_CH2` | 舵机 ch4/ch5 |
| PC6 / PC7 / PC8 / PC9 | `TIM8_CH1`…`CH4` | 舵机 ch0–ch3 |
| PD0 / PD1 | `CAN1_RX` / `CAN1_TX` | ⚠️ 板载 CAN 收发器在这两个脚 |
| PD3 | GPIO_Input，**Pull-down** | 使能开关 |
| PD8 / PD9 | `USART3_TX` / `USART3_RX` | |
| PD11–PD14 | GPIO_Output，Push Pull，No pull，Low speed | OLED 软 SPI |
| PE0 | GPIO_Input，**Pull-up** | USER 按键（低有效） |
| PE5 / PE6 | `TIM9_CH1` / `TIM9_CH2` | 电机 B PWM |
| PE9 / PE11 / PE13 / PE14 | `TIM1_CH1`…`CH4` | 电机 C / D PWM |
| PH0 / PH1 | HSE | 已配 |

> **引脚真值以 `docs/BOARD-C30D.md` 为准**。参考课设里的 TB6612 / STBY / PB8-PB9 当 CAN
> **都是错的，不要照抄**。
>
> **IMU / OLED 也交给 CubeMX（已定）**：这两个是**软件协议**（位操作 I2C / SPI），不是硬件
> 外设，所以 CubeMX 里只配成普通 GPIO —— 但方向/类型由 CubeMX 单一真源定义，
> `app_imu.c` / `app_oled.c` 里的 `MODER`/`OTYPER`/`PUPDR` 写入会全部删掉。
> - PB10/PB11 必须是 **Open Drain**（I2C 线与），应用层仍靠读 `IDR` 取 SDA —— OD 输出模式照样能读。
> - PD11–PD14 是 Push-Pull 输出（DC/RES/DAT/CLK），初始电平无所谓，应用层初始化时会自己置。

### 2.7 System Core → NVIC

| 项 | 值 |
| --- | --- |
| Priority Group | **4 bits for pre-emption priority / 0 bits for subpriority**（`NVIC_PRIORITYGROUP_4`） |
| TIM6_DAC global interrupt | Enable，**2** |
| USART1 / USART3 global | Enable，**4** |
| CAN1 RX0 | Enable，**6** |
| DMA 各流 | Enable，**5** |
| TIM7（时基） | 由 SYS 自动，15 |

> ⚠️ 这三个数（2 / 4 / 6）是设计值，不是随手填的：
> **2 = 高于 RTOS 临界区阈值（25 kHz 不被推迟）；6 = 低于阈值（CAN 可用 RTOS API）；4 是 R18 折中**。

---

## 3. Project Manager

**Project 标签页**

| 项 | 值 |
| --- | --- |
| Toolchain / IDE | **CMake**（保持） |
| Project Name / Location | 保持 `vehical_rak_motion` / 当前目录 |
| Application Structure | Advanced |

**Code Generator 标签页**

| 项 | 值 | 理由 |
| --- | --- | --- |
| **Generate peripheral initialization as a pair of .c/.h files per peripheral** | ✅ **勾上** | 现在只有 `gpio.c`；12 个定时器全塞进 `main.c` 会炸。勾上后生成 `tim.c` / `usart.c` / `can.c` / `adc.c` / `iwdg.c` / `dma.c` |
| Copy only the necessary library files | ✅ | 保持 |
| Generate under root | ❌ | 保持 |
| Set all free pins as analog | ❌ | **必须不勾** —— 会把用不到的脚全设成模拟，可能影响板上其他电路 |

**Advanced Settings 标签页**

| 项 | 值 |
| --- | --- |
| 每个外设的 `Generate IRQ handler` | ✅ 勾（我们要用生成的 handler 承载 ISR） |
| 每个外设的 `Call HAL handler` | ⚠️ **TIM6 选"不调用 HAL handler"**（见下） |

> ⚠️ **重要**：TIM6 的 `Call HAL handler` 若勾上，生成的 handler 会是
> `HAL_TIM_IRQHandler(&htim6);` —— 那套回调链在 25 kHz 下开销太大（AGENTS.md 的 35% 预算）。
> **TIM6 必须选"不调用 HAL handler"**，让它只生成空的 `TIM6_DAC_IRQHandler`，
> 我们把控制环放进它的 `USER CODE BEGIN TIM6_DAC_IRQn 0` 块里。
> 其余外设保持"调用 HAL handler"。

---

## 4. 生成后立刻要做的检查（AGENTS.md 红线）

生成完先别急着编译，逐条核对：

1. **`stm32f4xx_it.c` 里不得出现 `SVC_Handler` / `PendSV_Handler` / `SysTick_Handler`**
   （`.ioc` 里 `NVIC.Saved*IrqHandlerGenerated=true` 应该挡住了，但要眼见为实）。
2. **`SystemClock_Config()` 的 HSE 还在**（不能被抹回 HSI）。
3. **`main.c` 里 `/* USER CODE BEGIN 2 */` 的位置**：确认 `car_init()` 等调用落在
   **所有 `MX_*_Init()` 之后**。CubeMX 把 USER CODE 2 块放在 `MX_GPIO_Init()` 后面，
   加了外设之后可能插在中间 —— 若如此，把调用块**整体挪到最后一个 `MX_*_Init()` 之后**
   （仍在 USER CODE 块内，重生成不会丢）。
4. **`MX_DMA_Init()` 必须在 `MX_USART1_UART_Init()` 之前**（CubeMX 默认就满足）。
5. `main.c` 里 `MX_*_Init()` 的调用顺序：GPIO → DMA → ADC → CAN → IWDG → TIM×12 → USART×2。

---

## 5. 生成完之后我来做的事（你不用管）

**会删掉的应用层代码**（这些是"寄存器级初始化"，CubeMX 接管后就没用了）：

- `app_car.c`：`gpio_af()`、`pwm_timer_init()`、`motor_pwm_init()`、`encoder_init()`、
  `button_init()`、`enable_switch_init()`、`battery_adc_init()`、`watchdog_init()`、
  `pid_timer_init()` 的寄存器部分
- `app_serial.c`：`serial_init()` 的寄存器部分
- `app_can.c`：`can_init()` 的寄存器部分
- `app_servo.c`：`servo_init()` 的寄存器部分、`gpio_af()`

**会改写成"被生成 handler 调用"的 ISR**（避免重复符号，且逻辑仍留在应用层）：

| 现在 | 改成 | 由谁调用 |
| --- | --- | --- |
| `TIM6_DAC_IRQHandler`（app_car.c） | `car_control_isr(void)` | `stm32f4xx_it.c` 的 `USER CODE BEGIN TIM6_DAC_IRQn 0` |
| `CAN1_RX0_IRQHandler`（app_can.c） | `car_can_rx_isr(void)` | `USER CODE BEGIN CAN1_RX0_IRQn 0` |
| `USART1_IRQHandler` / `USART3_IRQHandler`（app_serial.c） | `serial_uart_isr(0/1)` | 对应 USER CODE 块 |

**会换成 HAL 的调用**：

| 现在 | 改成 |
| --- | --- |
| `TIM1->CCR1 = pulse` 等 | `__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse)`（宏，开销相同） |
| `pwm_arr` 运行时计算 | 读 `htim1.Init.Period`（CubeMX 固定值），删掉时钟兜底 |
| `ADC1->CR2 \|= SWSTART` + 自旋 | `HAL_ADC_Start` / `HAL_ADC_PollForConversion` / `HAL_ADC_GetValue` |
| `IWDG->KR = 0xAAAA` | `HAL_IWDG_Refresh(&hiwdg)` |
| `pid_timer_init()` | `HAL_TIM_Base_Start_IT(&htim6)` + DWT 使能 |
| USART RX：逐字节 + COBS 状态机 | **循环 DMA + 空闲线**（`HAL_UARTEx_ReceiveToIdle_DMA`）；回调只记长度置标志，COBS 解码移到 1 kHz 任务 |
| USART TX：`txq_push` 环形缓冲 + TXE 中断续发 | **帧队列 + DMA**（`HAL_UART_Transmit_DMA`），`HAL_UART_TxCpltCallback` 启动下一帧 |
| CAN 直接读写邮箱 | `HAL_CAN_AddTxMessage` / `HAL_CAN_GetRxMessage` |
| `app_imu.c` / `app_oled.c` 的 `MODER`/`OTYPER`/`PUPDR` 写入 | 删掉，改由 CubeMX 的 `MX_GPIO_Init()` 单一真源 |

**会保住的**：25 kHz ISR 的函数体（仍是整数运算、无浮点/除法链/阻塞调用）、
急停快速路径（CAN ISR 置标志 → 25 kHz ISR ≤40 µs 刹车）、
麦轮 IK、稳定器、UART v1 协议语义、CAN v3 协议。

**顺带消失的三个问题**（其中两个是我上一轮修的）：
`v1_send` 静态缓冲的 ISR/任务竞争、双路 COBS 状态共用、以及 TXE 中断与
`serial_poll_tx` 对读指针的并发 —— RX/TX 都 DMA 化之后这条路径整体重写，问题不复存在。
**但重写本身要重新验证**，别当成等价重构。

---

## 6. 已定决策（2026-10-06）

| # | 决策 | 接受的代价 |
| --- | --- | --- |
| 1 | **串口 RX + TX 都走 DMA** | TX 侧"软件环形缓冲 + TXE 中断续发"（`txq_push` / `serial_poll_tx`）整体换成**帧队列 + DMA**。RX 用**循环** DMA。 |
| 2 | **CAN 走 HAL** | `HAL_CAN_GetRxMessage` 在 CAN ISR（优先级 6）里调，比直读寄存器多约 100–200 周期 —— 1 kHz 帧率下可忽略；急停快速路径仍只是置标志，不受影响。 |
| 3 | **IMU / OLED 引脚交给 CubeMX** | 两个模块的 GPIO 初始化代码删除，方向/类型由 `MX_GPIO_Init()` 单一真源定义。 |

### 一个我主动保留、不随"全 DMA"改的东西

**USART 中断优先级保持 4**（高于 FreeRTOS 阈值 5），不改回 5。

R18 把串口从 7 提到 4，是因为 USART IRQ 被 RTOS 临界区推迟 → RXNE 没及时取 → ORE 丢字节。
DMA 之后这个理由**理论上消失了**（搬字节不需要 CPU 参与），放回 5 就能在回调里用
FreeRTOS API、也更"标准"。

但这条路径在台架上被 ORE 折磨过（见 `docs/UART-v1-ALIGN.md`），**不动它比"理论上更好"值钱**。
代价很小：回调只写内存（记长度 + 置标志），解码放到已有的 1 kHz 任务节拍里 ——
100 Hz 的指令率下最坏延迟 1 ms，host 的 200 ms lease 有 200× 余量。

若台架验证稳定，**以后**可以单独一个提交把它放回 5，那时才是纯粹的简化。

---

## 7. 验收

生成后我负责：`cmake --preset Debug/Release` 双配置零警告、`tools/can_check.py` 通过、
复查 §4 的 5 条、以及**上板前的静态复核**。

⚠️ **上板前必须重做台架验证**：这是对 25 kHz 控制环、急停路径、串口 RX 的**结构性改动**，
不是等价重构。冷启动后看 OLED 的 `build_tag`（我会升到 R20）。
