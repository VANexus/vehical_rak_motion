# CubeMX 标准作业流程（SOP）

> **这份文件的存在理由**：本仓在 R20 之前，所有外设（TIM/CAN/USART/ADC/IWDG/GPIO）
> 都是应用代码里手写寄存器初始化的。那套做法的代价在 R19/R20 集中爆发：
> ADC 采样时间写错寄存器、ADC 时钟超规格（42 MHz > 36 MHz 上限）、CAN 位时序漏配
> （875 kbps 而不是 1 Mbps）、缺 TIM2/TIM5/USART3、多出 CAN2——**每一条都是
> "手写配置漏了一项"**，而 CubeMX 本来会免费帮你配好。
>
> R20 起改为：**外设配置一律由 CubeMX 生成，应用层只写逻辑。**
> 本 SOP 规定怎么做、以及什么情况算违规。

---

## 一、铁律

> **外设的初始化（时钟使能、引脚 AF、PSC/ARR、波特率、位时序、DMA、NVIC）
> 只允许出现在 CubeMX 生成的文件里。应用代码里出现 `RCC->xxxENR`、
> `GPIOx->MODER`、`GPIOx->AFR`、`TIMx->PSC`、`CAN1->BTR` 这类写法，就是违规。**

判断标准很简单——**问自己：这项配置在 CubeMX 的图形界面里有没有对应勾选框？**
有，就必须在 CubeMX 里配。

### 为什么

1. **手写配置的错误是静默的**。ADC 采样时间写到错误的寄存器，代码照样编译、照样跑，
   只是读数偏低。CAN 位时序漏配，代码照样跑，只是总线上没人听得懂。
   CubeMX 会校验参数范围、会算出时钟分频、会保证 DMA 流/通道组合合法。
2. **单点定义**。引脚 AF、定时器分频这些值只应有一处真源，就是 `.ioc`。
   手写一份 + CubeMX 一份 = 两处真源 = 迟早不一致。
3. **重生成安全**。只要配置在 `.ioc` 里，CubeMX 重新生成时就会把它重建出来；
   手写的部分则要靠人记住。

### 唯一的例外

**25 kHz 闭环 ISR 的函数体**（`app_car.c` 的 `car_control_isr()`）保持寄存器级。
理由：AGENTS.md §1 的硬实时预算（ISR ≤ 周期 35 %），HAL 的回调链开销吃不下。
但它**只被**生成的 `TIM6_DAC_IRQHandler` 通过 `USER CODE` 块调用——
**外设配置仍然全部由 CubeMX 完成**，只是中断里那段循环用裸寄存器写。

这个例外是"中断里怎么写"，不是"外设怎么配"。**别拿它当借口扩大例外范围。**

---

## 二、文件归属：哪些能动，哪些不能动

| 文件 | 归属 | 规则 |
| --- | --- | --- |
| `vehical_rak_motion.ioc` | **真源** | 只通过 CubeMX 改（GUI 或无头脚本），不手改文本 |
| `Core/Src/{main,gpio,freertos,adc,can,dma,iwdg,tim,usart,stm32f4xx_it,stm32f4xx_hal_msp,stm32f4xx_hal_timebase_tim}.c`<br>`Core/Inc/{main,gpio,adc,can,dma,iwdg,tim,usart,stm32f4xx_it,stm32f4xx_hal_conf,FreeRTOSConfig}.h`<br>`cmake/stm32cubemx/CMakeLists.txt`<br>`cmake/gcc-arm-none-eabi.cmake` | **生成物** | **只往 `/* USER CODE BEGIN xxx */ … /* USER CODE END xxx */` 之间写**。块外的任何改动都会在下次生成时丢失 |
| `Core/Src/app_*.c`、`Core/Inc/app_*.h` | **我们的** | 随便写。但**不许**写外设初始化寄存器 |
| `CMakeLists.txt`（顶层） | **我们的** | 只生成一次，不覆盖。新增 `app_*.c` 加进这里的 `target_sources` |
| `docs/`、`tools/` | **我们的** | — |

### 新增一个应用模块时

1. 新建 `Core/Src/app_xxx.c` + `Core/Inc/app_xxx.h`
2. 加进**顶层** `CMakeLists.txt` 的 `target_sources` / `target_include_directories`
   （**不要**动 `cmake/stm32cubemx/CMakeLists.txt`，那份会被覆盖）
3. 如果它需要新的外设 → 走下面第三节的流程

---

## 三、改一个外设的标准流程

### 第 0 步：先分类

| 你要做的事 | 走哪条路 |
| --- | --- |
| 改波特率 / PSC / ARR / 采样时间 / NVIC 优先级 / 引脚 / 加个外设 / 加条 DMA | **配置改动** → CubeMX |
| 改控制算法 / 协议编解码 / 状态机 / 任务结构 | **逻辑改动** → 只动 `app_*.c` |
| 中断里那段循环怎么写 | 例外：可裸寄存器，但外设本身仍在 CubeMX 配 |

### 第 1 步：改 `.ioc`

**方式 A —— GUI（推荐给结构性改动）**

```bash
/home/xrak/STM32CubeMX/STM32CubeMX      # 6.18.1，与本 .ioc 匹配
```
`File > Load Project` 选 `.ioc` → 改 → `GENERATE CODE`。
**不要用 CubeMX2**（另一个产品，`.ioc` 格式不同，会改工程布局）。

**方式 B —— 无头脚本（推荐给标量改动、或要在 CI/无人值守下做）**

见第四节。无头方式**必须**用脚本命令，**不许**直接编辑 `.ioc` 文本（原因见第五节）。

### 第 2 步：生成

GUI 按 `GENERATE CODE`；无头用 `project generate`。

### 第 3 步：生成后必查（**每次都要**）

```bash
cd <repo>
# ① 生成物不该出现的东西
grep -nE "SVC_Handler|PendSV_Handler|SysTick_Handler" Core/Src/stm32f4xx_it.c && echo "❌ 三个 handler 被恢复了" || echo "✅"
grep -n "HSEState\|RCC_HSE_ON" Core/Src/main.c | head -3      # ② 时钟源还是 HSE 吗
grep -n "USER CODE BEGIN 2" Core/Src/main.c                    # ③ USER CODE 2 在所有 MX_*_Init() 之后吗
grep -nE "MX_DMA_Init" Core/Src/main.c                         # ④ MX_DMA_Init 在 MX_USART*_Init 之前吗
grep -oE "MX_[A-Z0-9_]+_Init\(\)" Core/Src/main.c | tr '\n' ' ' # ⑤ 该有的 Init 一个不少、不该有的没有
```

**⑤ 尤其重要**——R20 那次就是这里漏了 TIM2/TIM5/USART3。跟 `docs/BOARD-C30D.md`
的引脚表对一遍。

### 第 4 步：应用层适配

生成物变了，`app_*.c` 可能要跟着改：

- **新增了外设** → 在 `app_*.c` 里 `#include "xxx.h"`，用生成的 HAL 句柄
  （`htim1` / `huart1` / `hcan1` / `hadc1` / `hiwdg`）
- **新增了中断** → 不要自己定义 `XXX_IRQHandler`（会和生成的撞名，链接失败）。
  正确做法：把处理逻辑写成普通函数（如 `car_control_isr()`），
  在生成 handler 的 `USER CODE BEGIN XXX_IRQn 0/1` 块里调用
- **改的是 PWM/比较值** → 用 `__HAL_TIM_SET_COMPARE(&htimx, TIM_CHANNEL_n, v)`
- **改的是 ADC/IWDG** → 用 `HAL_ADC_*` / `HAL_IWDG_Refresh`

### 第 5 步：验证

```bash
. ./env/activate.sh
cmake --preset Debug   && cmake --build --preset Debug      # 必须零警告
cmake --preset Release && cmake --build --preset Release    # 必须零警告
python3 tools/can_check.py                                  # 动了协议就必须过
bash tools/cubemx_lint.sh                                   # 外设初始化越界检查
```

**上板**：真断电冷启动（电池关 → 拔 ST-LINK → 等 OLED 全黑 → 再上电），
看屏上 `build_tag` 判别实际在跑哪版。**每次有效改动升一档标签**（定义在 `app_car.c`）。

---

## 四、无头 CubeMX 命令参考

CubeMX 6 的 `-q` 脚本模式有一整套命令。**这是官方机器接口，改 `.ioc` 应该走它。**

```bash
cat > /tmp/gen.txt <<'EOF'
config load /绝对路径/vehical_rak_motion.ioc
<命令…>
project generate
exit
EOF
timeout 300 /home/xrak/STM32CubeMX/STM32CubeMX -q /tmp/gen.txt
```

### 命令表

| 命令 | 作用 |
| --- | --- |
| `help` | **列出所有命令**（忘了就喂这个） |
| `config load <ioc>` | 载入工程 |
| `set mode <外设> <模式名>` | 激活一个外设模式（**最常用**） |
| `set ip parameters <外设> <参数> <值>` | 设外设参数 |
| `set pin <引脚> <信号名>` | 指定引脚功能 |
| `get modes <外设>` | 该外设所有叶子模式名 |
| `get mode active\|available\|all <外设>` | 当前 / 可选 / 全部模式（**含 ModeId**） |
| `get mode_param_list <外设> <模式>` | 该模式的参数列表 |
| `get mode_param_possvalue <外设> <模式> <参数>` | **参数合法取值**（拿不准就查这个） |
| `project generate` | 生成代码 |
| `exit` | 退出 |

### 实战示例（R20 用的就是这几条）

```
config load .../vehical_rak_motion.ioc
set mode TIM2 Encoder_Interface                  # 激活 TIM2 编码器模式
set ip parameters USART3 BaudRate 921600         # 改 USART3 波特率
project generate
exit
```

### 踩过的坑（别再踩）

| 坑 | 现象 | 正确做法 |
| --- | --- | --- |
| **一次脚本里连做两个 `set mode`** | CubeMX 挂死（超时），日志出现 `IP not ready for code generation` | **一次脚本只做一个 `set mode`**，分次跑 |
| 参数值当数字填 | `IP (CAN1) : Parameter (BS1) has invalid value (16)` | `.ioc` 里**枚举型参数存的是字符串**：`CAN_BS1_16TQ` 而不是 `16`。用 `get mode_param_possvalue` 查合法值 |
| 手改 `Mcu.IP*` / `Mcu.Pin*` 块 | CubeMX 拒绝新增的 IP；更糟的是把 `TIM6`/`NVIC` 弄成 `not ready`，之后生成全挂 | **永远不手改这两个块**，用 `set mode` / `set pin` 让 CubeMX 自己维护 |
| 不设超时 | 无头模式下若 CubeMX 想弹对话框，会永久阻塞 | **一定加 `timeout`** |
| **GPIO 参数用 `set ip parameters` 设** | 返回 `OK` 但 `.ioc` 纹丝不动（实测：`set ip parameters USART2 BaudRate 115200` 也是假 OK，值是碰巧的默认值才对） | **GPIO 引脚参数走 `set gpio parameters <引脚> <参数名> <值>`**。这条不在 `help` 的命令表里，是从 `set gpio` 的 usage 提示里挖出来的。例：`set gpio parameters PE0 GPIO_PuPd GPIO_PULLUP` |
| 只设 `set pin <脚> GPXTI0` | CubeMX 给默认值 `GPIO_MODE_IT_RISING` + `NOPULL` —— 低有效按键配成上升沿、还浮空 | 信号与参数**分开设**：先 `set pin PE0 GPXTI0`，再 `set gpio parameters PE0 GPIO_PuPd GPIO_PULLUP` 和 `... GPIO_ModeDefaultEXTI GPIO_MODE_IT_RISING_FALLING` |
| NVIC 用脚本使能 | `set ip parameters NVIC EXTI0_IRQn true` 返回 `OK` 但不生效，`.ioc` 里没有条目、`stm32f4xx_it.c` 里也没有 handler | **NVIC 使能只能 GUI 勾**（`System Core → NVIC`）。脚本设不了 |
| **只看 `cmake --build` 退出码就以为接好了** | 工程开了 `-Wl,--gc-sections`：新写的模块**没有调用者**时整条链被回收，构建照样零警告零错误通过，但符号根本不在 `.elf` 里 —— 上板什么都不发生 | **生成后必须查符号**：`arm-none-eabi-nm build/Debug/*.elf \| grep <你的前缀>`。空 = 被 gc 了，说明还没接进任何任务 |

### 日志怎么看

```bash
grep -E "OptionalMessage_ERROR|not ready" /tmp/gen.log
```
`(OptionalMessage_ERROR)` 是真问题；`IP not ready for code generation: TIM6` 是**无害提示**
（TIM6 是基本定时器，没有"模式"可选，照样会生成 `MX_TIM6_Init`）。
`exit=0` 且日志尾部是 `OK / Bye bye` 才算成功。

---

## 五、反模式（看到就改）

| ❌ 不要 | ✅ 应该 |
| --- | --- |
| 在 `app_*.c` 里写 `RCC->AHB1ENR \|= ...`、`GPIOB->MODER = ...`、`TIM6->PSC = 0` | 在 CubeMX 里配，应用层用 HAL 句柄 |
| 在 `app_*.c` 里定义 `void TIM6_DAC_IRQHandler(void)` | 写成普通函数，从生成 handler 的 `USER CODE` 块调用 |
| 直接编辑 `.ioc` 文本 | 用 CubeMX GUI，或 `-q` 脚本命令 |
| 改 `stm32f4xx_it.c` / `gpio.c` / `tim.c` 等生成物的 `USER CODE` 块**外面** | 只写块内；块外改动下次生成就没了 |
| 在 `cmake/stm32cubemx/CMakeLists.txt` 里加源文件 | 加进**顶层** `CMakeLists.txt` |
| 为了"少改一次生成"而在应用层打补丁绕过配置问题 | 改 `.ioc`，让 CubeMX 重新生成 |

---

## 六、提交要求

动了 `.ioc` 或生成物的提交，说明里**必须写明**：

1. 改了哪个外设的哪个参数（旧值 → 新值）
2. **为什么**（需求 / 修 bug / 对齐硬件）
3. 有没有跑生成后必查（第三节第 3 步）
4. `build_tag` 升到哪一档

### 跨仓同步

以下文件在**两个 MCU 仓各存一份**，改任一份时**同一批提交两个仓**（手工 `cp`，无同步工具），
并在提交说明写明：

- `docs/PROTOCOL.md`、`docs/protocol-signals.yaml`、`docs/gen/can_protocol.h`
- `docs/BOARD-C30D.md`、`tools/can_check.py`、`env/activate.*`

---

## 七、相关文档

- [`AGENTS.md`](../AGENTS.md) — 硬性约束（本 SOP 是其中"外设配置"一条的展开）
- [`CLAUDE.md`](../CLAUDE.md) — 架构与模块划分
- [`docs/BOARD-C30D.md`](BOARD-C30D.md) — **引脚与硬件真相**（生成后必查的对账单）
- [`docs/CUBEMX-MIGRATION.md`](CUBEMX-MIGRATION.md) — R19→R20 那次迁移的逐项参数与理由
  （历史档案，新改动不必再读）
