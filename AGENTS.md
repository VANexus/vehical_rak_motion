# AGENTS.md — 本仓工作约束（底盘 MCU 固件）

> ⚠️ **v4 已于 2026-10-30 整体下线。** 本文中所有对 `docs/PROTOCOL.md`、`docs/protocol-signals.yaml`、
> `tools/can_check.py`、`docs/gen/can_protocol.h`、`APP_USE_CANOPEN` 的引用**全部作废**——
> 这些文件已删除，固件里没有 v4 分支。协议唯一规范是 [`docs/CANOPEN-V5.md`](CANOPEN-V5.md)（README 同目录），
> 门禁是 `python3 tools/canopen_gate.py --repo .`。历史说明保留原文只为追溯，**不要照着执行**。



> **CAN 协议已于 2026-10-29 切到 v5（CANopen）** —— 规范件 `docs/CANOPEN-V5.md`，
> 门禁 `python3 tools/canopen_gate.py`，台架验收 `python3 tools/canopen_bringup.py`。
> 本文里凡提到 `can_check.py` / `protocol-signals.yaml` / `PROTOCOL.md`(v4) / `can_protocol.h`
> 的条目**只适用于 v4 回滚分支**（`cmake -DAPP_USE_CANOPEN=OFF`）。
> 当前在板 build_tag：底盘 **R73** / 机械臂 **A14**；Jetson `rak` 侧仍是 v4 主站，
> 与 v5 板子不兼容，联调暂用上面的 `canopen_bringup.py`（生产主站 = 待办 P3）。


C30D（STM32F407VET6 + 板载 AT8236）**四轮麦轮底盘**固件。姊妹仓 `vehical_rak_edge_ai`
（git 仓名历史遗留）是**同一块板子**的**机械臂板 `vehical_rak_arm`** 角色（6 路舵机），
上位机是 `VANexus/rak`（Jetson）。**2026-10-28 恢复双板架构**：舵机从本板迁出，本板收敛为
pure 底盘（4 轮闭环 + IMU + CAN）——见 `README.md` §0 与 [`docs/KNOWN-ISSUE-runaway-wheel.md`](docs/KNOWN-ISSUE-runaway-wheel.md)（左前轮自转问题，issue rak#8）。

## 先读

- `CLAUDE.md`：架构与模块划分（启动链、任务表、25 kHz ISR 位置、CubeMX 边界）——本文件不重复其内容
- `docs/QUICKSTART.md`：两端（Windows / Linux）从装环境到烧录的完整流程 + 常见坑
- `docs/BOARD-C30D.md`：**引脚与硬件真相**（板载 CAN 收发器在 PD0/PD1；AT8236 无 STBY；无电流采样）
- `docs/PROTOCOL.md` + `docs/protocol-signals.yaml`：CAN 协议 **v4（规范 + 机器可读定义，2026-10-09 起内容冻结 FROZEN）**——CAN 是唯一主链路，USB v2 冻结备用；确需修订走 YAML→两端实现→`can_check.py`→两 MCU 仓同批提交的提案流程
- 控制契约：**4 轮速 + 7 轴位置**，麦轮 IK 在**上位机**（`docs/ROS2-CONTROL-ARCH.md`，⚠️ 该文写于"臂板烧毁、只剩 USB"时期，**部分过期**，见文首横幅）；USB v1/v2 对齐文档（`docs/UART-v1-ALIGN.md` / `docs/UART-v2-ALIGN.md`）同属历史备用链路

## 硬性约束

1. **25 kHz 硬实时闭环**是设计规定。ISR（`app_car.c` 的 `TIM6_DAC_IRQHandler`）内禁止浮点、除法链、`printf`、阻塞调用；转速/位置/占空比归 ISR 单一写者，任务只读。ISR 占用率必须实测（DWT 或翻转 GPIO），预算 ≤ 周期 35 %。
2. **上电不得自动驱动电机**；电机启动只能来自显式指令（CAN v4 `DRIVE_CMD`+`START`；USB v2 为备用路径）。[AI] R43 按钮生产语义已被 **R59/R61 冻结语义取代**：PE0 按钮实测硬件故障（恒低）**禁用**，急停置位/清除**全权走 CAN `SAFETY_CMD`（0x010 cmd=4/5）**；按钮固件路径保留，硬件修复后可恢复。测试先架空车轮。
3. **1 Mbps CAN 必须跑在 HSE 8 MHz 上**——时钟树已按 HSE 配置（`.ioc` + `SystemClock_Config`，R2 起）。红线是**别被 CubeMX 重生成抹回 HSI**；联机偶发错误帧先查时钟源，再查协议。
4. 上板后**必须真断电冷启动**（电池关 → 拔 ST-LINK → 等 OLED 全黑 → 再上电），并看屏上 `build_tag` 判别实际在跑哪版（单点定义在 `app_car.c`）；每次有效改动升一档标签。
5. 25 kHz 能否出扭矩有一条**未裁决的冲突记录**（台架说只有 10 kHz 能出扭矩）。实测不通过就停下上报，**不许擅自改频率**。
6. 共享常量（引脚、定时器、编码器计数）**单点定义**在板级头文件或 `app_car.c` 顶部，一个物理量只允许一个常数；参考课设与 AI 设计草案里的选型（TB6612、STBY、PB8/PB9 当 CAN、电流保护）**都是错的，不得引用**。
7. **注释署名约定**：AI/工具产出的注释一律以 `[AI]` 前缀标记（如 `/* [AI] 说明 */`），便于人工区分与回溯（`grep -rn "\[AI\]" Core/` 可列出全部）；**用户原有注释永不删除或改写**，需要调整先征询。
8. `freertos.c` 的 `defaultTask` 为空转（R41 起舵机测试代码已移除）——别把它当控制入口去“修”。按钮语义见硬性约束 #2（R43 生产化：短按清急停、长按急停）。
9. **外设配置一律由 CubeMX 生成（R20 起）**。时钟使能、引脚 AF、PSC/ARR、波特率、CAN 位时序、DMA、NVIC——**凡是 CubeMX 界面里有勾选框的，都不许在应用代码里手写寄存器**。改外设走 `.ioc`（GUI 或 `STM32CubeMX -q` 无头脚本，**不手改 `.ioc` 文本**）；应用层只写逻辑、只用生成的 HAL 句柄。唯一例外是 25 kHz ISR 的函数体（时序预算），且它也只由生成的 handler 经 `USER CODE` 块调用。**完整流程、无头命令参考、生成后必查清单、反模式见 [`docs/CUBEMX-SOP.md`](docs/CUBEMX-SOP.md)；门禁 `bash tools/cubemx_lint.sh`。**

## 构建 / 烧录 / 提交

```bash
. ./env/activate.sh            # Linux（Windows 用 . .\env\activate.ps1；只需 STM32CubeCLT 1.22.0）
cmake --preset Debug   && cmake --build --preset Debug     # 还要跑 Release
cmake --preset Release && cmake --build --preset Release
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_motion.hex -v -rst
python3 tools/canopen_gate.py --repo .  # 动了协议/OD 就必须通过（退出码 0）
python3 tools/canopen/gen_eds.py docs/canopen/<node>.json tools/canopen/template.eds docs/canopen/<node>.eds
python3 tools/canopen_bringup.py --can can0 --seconds 3   # 台架：NMT/SYNC/PDO 速率/SDO
bash tools/cubemx_lint.sh             # 动了 .ioc / 外设就必须通过（退出码 0）；见 docs/CUBEMX-SOP.md
```

- **每条命令都要在同一个 shell 里先 source `env/activate.sh`**——它只改当前 shell 的 PATH，不落盘；分开执行会报 `arm-none-eabi-gcc not found`。
- 若 activate 报「没找到 STM32CubeCLT」，说明本机没装工具链：构建/烧录**无从验证，别声称构建通过**；此时唯一能跑的门禁是 `python3 tools/canopen_gate.py --repo .`（纯 Python）。
- 本仓**没有单元测试框架**；两个自动化门禁：`canopen_gate.py`（协议/OD）与 `cubemx_lint.sh`（外设配置越界）。动了协议必须跑前者，动了 `.ioc` / 外设必须跑后者。
- `docs/gen/can_protocol.h` 目前**没有任何固件源文件 include**（`app_can.c` 自带编解码）——重新生成它不会改变固件行为。
- 新 `.c/.h` 加进**顶层** `CMakeLists.txt`（`cmake/stm32cubemx/` 那份会被 CubeMX 覆盖）；只往 `USER CODE BEGIN/END` 标记对之间写。
- CubeMX 重生成后必查（完整清单见 `docs/CUBEMX-SOP.md` §三第 3 步）：`stm32f4xx_it.c` 不得恢复 SVC/PendSV/SysTick 三个 handler；`SystemClock_Config` 的 HSE 设置是否被抹回 HSI；**`main.c` 的 `USER CODE BEGIN 2` 是否排在所有外设 `MX_*_Init()` 之后**；`.ioc` 里启用的每个外设是否都有对应的 `MX_*_Init()`（R20 就漏过 TIM2/TIM5/USART3）。`bash tools/cubemx_lint.sh` 覆盖了其中大部分。
- 提交前：Debug + Release 零警告 + `canopen_gate.py` 通过（动了协议）+ `cubemx_lint.sh` 通过（动了外设）。开工先 `git pull`，收工就 push。
- 改 `docs/PROTOCOL.md` / `protocol-signals.yaml` / `docs/BOARD-C30D.md` / `tools/can_check.py` / `env/activate.*` 时，**同一批把姊妹仓 `vehical_rak_edge_ai` 的那份一起改**（两仓各存一份，手工 `cp`，内容保持一致），并在提交说明里写明改了什么。
