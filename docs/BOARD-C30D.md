> **两份副本约定**：本文件在 `vehical_rak_motion` 与 `vehical_rak_edge_ai` 各存一份，内容必须保持一致——改动时**同一批提交两个仓**（手工 cp 即可，没有同步工具）。

# C30D 板级资源表（权威）

**唯一出处**（本仓内，可复核）：`docs/WHEELTEC L150Pro 智能小车资料_V2.1_2026.04.20.zip` 内的
`9.原理图/C30D 原理图和资源分配表/STM32F407VET6(C30D-V2.1_V2.2)主板资源分配说明.pdf`。
该 PDF 标题写 C30D-**V2.0**，与同目录 `旧版/STM32F407VET6(C30D-V2.0)主板资源分配说明.pdf`
**逐字节相同**（MD5 `42c671a7d5809e5d9e22f888949b25d3`），故引脚表跨版本通用。

**版本差异**（据包内 `旧版/2.0版和2.1版区别说明.txt`、`旧版/2.1和2.2版本区别说明.txt` 原文）：
- 2.0 → 2.1：换 MOS 管、调整显示屏位置元器件摆放 —— **「资源引脚无调整」**
- 2.1 → 2.2：电源电路优化、CAN 改 XH2.54 端子、去掉 SBUS 航模接口、加版本识别电阻
  —— **「尺寸孔位、控制使用、代码，2.1 和 2.2 版本之间无差异」**

→ **V2.0 / V2.1 / V2.2 三版引脚分配完全一致**，本表通用。

> ⚠️ 板上 MCU = **STM32F407VET6**，两块叶子板**硬件完全相同，只是装配角色不同**（用户确认）。所以本表同时是运动板和边缘 AI 板的板级定义。

---

## 1. 电机与编码器（4 路，板载驱动）

| 电机 | 驱动输入 1 | 驱动输入 2 | PWM 定时器 | 编码器 | 编码器定时器 |
| --- | --- | --- | --- | --- | --- |
| A | **PB8** | **PB9** | TIM10_CH1 / TIM11_CH1（PWMA） | PA15 / PB3 | TIM2_CH1/CH2 |
| B | **PE5** | **PE6** | TIM9_CH1/CH2（PWMB） | PB4 / PB5 | TIM3_CH1/CH2 |
| C | **PE9** | **PE11** | TIM1_CH1/CH2（PWMC） | PB6 / PB7 | TIM4_CH1/CH2 |
| D | **PE13** | **PE14** | TIM1_CH3/CH4（PWMD） | PA0 / PA1 | TIM5_CH1/CH2 |

- 驱动芯片 = **板载 AT8236**（原理图 Sheet 3/3 上四颗 AT8236）。**双输入控制，没有 STBY 引脚**；`IN1=IN2=1` 动态刹车，`IN1=IN2=0` 滑行。
- ⚠️ 五份 AI 设计草案里写的 `4× TB6612FNG`、`STBY 拉低保护`、`CAN1_RX=PB8/CAN1_TX=PB9` **与本板不符**（PB8/PB9 是电机 A 的输入），一律以本表为准。
- 编码器在**电机（减速箱输入）侧**，硬件正交 ×4；61 600 计数/输出轴圈（A 路手转实测 61 602；B/C/D 未分别标定）。

## 2. 通信

| 接口 | 引脚 | 板上用途 |
| --- | --- | --- |
| **CAN1** | **PD0 = RX，PD1 = TX** | **板载 CAN 收发器**（PDF 备注"主控板板载 CAN 芯片"；具体型号本表不认定，接线看 CANH/CANL 端子） |
| USART1 | PA9 / PA10 | TYPE-C USB 串口 1（CH9102F）→ **调试串口首选**，与草案一致 |
| USART3 | PD8 / PD9 | TYPE-C USB 串口 3（CH9102F） |
| USART2 | PD5 / PD6 | 串口蓝牙（默认给 APP 通信） |
| USB OTG | PA11 / PA12 | USB 手柄接口（选配） |
| SWD | PA13 / PA14 | 调试 |

## 3. 人机与传感器

| 资源 | 引脚 | 说明 |
| --- | --- | --- |
| 用户按键 | PE0 | 低有效，[AI] R48 起为 EXTI 双沿中断（GPXTI0 + 上拉，EXTI0 优先级 7），消抖仍在 100 Hz 任务 |
| LED | PE8 | 低亮 |
| 蜂鸣器 | PA8 | 需插跳线帽 |
| OLED 128×64 SSD1306 | PD11=DC、PD12=RES、PD13=DATA、PD14=CLK | 软 SPI |
| 使能开关 | PD3 | 输入 |
| **板载 IMU ICM20948** | **PB10 / PB11** | 软 I2C（PB10=SCL、PB11=SDA，开漏+外部上拉）。[AI] R46 澄清：早期文档写的"PB12 外部中断"**未启用**——.ioc 无 PB12，固件用 100 Hz 轮询（`imu_update`），PB12 非本驱动依赖 |
| 电池电压 ADC | PB0（ADC1_IN8） | 100k:10k 分压，`mV = raw × 3300 × 11 / 4095` |
| 车型选择电位器 | PB1（ADC） | 出厂用途，本项目可挪用但要留档 |
| 预留 ADC | PC4 / PA4 / PA5 | "CCD/ELE 接口"预留 |

⚠️ **全板没有每通道电流采样**。草案里的"电流保护/电流检测"**没有硬件基础**：堵转与过流只能靠软件判据（占空比高 + 编码器不动 / 速度骤降），或用预留 ADC 通道外接检流电阻后自行标定。

⚠️ 板载 ICM20948 与"IMU 直连 Jetson 的 LSM6DSRTR"是**两个不同的 IMU**，谁是里程计/航向的权威源需要定稿（见 [`protocol.md`](PROTOCOL.md) §9 待确认项）。

## 4. 舵机 / 航模口（机械臂底座的落点）

| 资源 | 引脚 | 定时器 |
| --- | --- | --- |
| 航模/舵机接口 6 路 | **PC6 PC7 PC8 PC9** + **PB14 PB15** | **TIM8**（PC6-9 = CH1-4）+ **TIM12**（PB14/15） |

- 备注原文：阿克曼小车（舵机）使用 PB15。
- 参考课设的 `代码/bsp/bsp_c30d.c` 也正是这么用的：`servo_init()` = "arm servos: TIM8 PC6-9 + TIM12 PB14/15 @100Hz"，APB2 168 MHz、`PSC=167` → 1 µs 步进。
- 因此机械臂**底座步进电机（脉冲+方向）可以落在 PC6 = TIM8_CH1（脉冲）+ 任一空闲 GPIO（方向）**；驱动器需外接，不占板载 AT8236 通道。

## 5. 空闲可用 IO（PDF 汇总）

- 引出预留 IO：`PA2 PA3 PA6 PA7 PA8 PA13 PA14 PC0 PC1 PC2 PC3 PC5 PC10 PC11 PC12 PD2 PD15`
- 未占用：`PB13`；`PC13 PC14 PC15`；`PD4 PD7 PD10`；`PE1 PE2 PE3 PE4 PE7 PE10 PE12 PE15`（合计 15 个）
- ⚠️ `PA13/PA14` 已在 SWD 清单里，PDF 的"引出预留"与调试口重叠，**做输出用前先确认没被调试器占用**。

## 6. 时钟与 1 Mbps 的前提（必须处理）

**已核实（2026-10-14，读生成代码）**：两个工程的时钟树**都已经是 HSE**，无需再切。
`Core/Src/main.c` 的 `SystemClock_Config()` 实测：

```c
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLM = 4;   RCC_OscInitStruct.PLL.PLLN = 168;
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;   /* 8/4*168/2 = 168 MHz */
```

> ⚠️ 本节此前写「骨架当前是 **HSI** + PLL…工程化第一步把时钟树切到 HSE」，是**过期描述**（与生成代码和 `README.md` §2 均不符），已删除。留着它会让团队去做一件早已完成的事，而掩盖真正的 1 Mbps 风险点。

**CAN 1 Mbps 对时钟精度极其敏感，HSI 的量级不够**（HSI 是百分比分级的温度相关精度，CAN 1 Mbps 需要 ppm 级）。板上 PH0/PH1 的 8 MHz 晶振已启用。相关风险与验证方法见 [`PROTOCOL.md`](PROTOCOL.md) §3。

## 7. 交叉验证过的历史事实（来自台架期）

- 物理方向：`forward_dir = {1,-1,-1,1}` 使四个输出轴均顺时针（2026-09-30 现场确认，换装需重测）。
- 起步静摩擦门槛约 75% 占空比；实用限幅 85%。
- 刹车后每路滑行 6.1–6.7 k 计数（0.10–0.11 圈）。
- 烧录后必须真断电冷启动（ART 指令缓存），以屏上 `build_tag` 判别实际运行版本。
- 动力电源 = 板顶蓝色螺丝端子 + ON/OFF 开关；ST-LINK 读到的 3.2 V 只代表 MCU 侧。

板级事实的最终出处 = 上述 WHEELTEC 资料包内的原理图（`9.原理图/C30D 原理图和资源分配表/STM32F407VET6控制器原理图（C30D_V2.2）.pdf`）
与资源分配说明 PDF，两者都在本仓 `docs/` 的 zip 内，**可复核**。

⚠️ 本节（§7）的历史实测数据出自**已弃用的 `car2` 台架归档**（工作区已移除，见 `AGENTS.md` §1）。
按事实分级它们属于「**转述，待重新实测**」——上板前不要当作已确认值。
