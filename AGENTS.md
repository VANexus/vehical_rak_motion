# AGENTS.md — 本仓工作约束（底盘 MCU 固件）

C30D（STM32F407VET6 + 板载 AT8236）四轮麦轮底盘固件。姊妹仓 `vehical_rak_arm` 是**同一块板子**的机械臂角色，上位机是 `VANexus/rak`（Jetson）。

## 先读

- `docs/QUICKSTART.md`：两端（Windows / Linux）从装环境到烧录的完整流程
- `docs/BOARD-C30D.md`：**引脚与硬件真相**（板载 CAN 收发器在 PD0/PD1；AT8236 无 STBY；无电流采样）
- `docs/PROTOCOL.md` + `docs/protocol-signals.yaml`：CAN 协议 v3（规范 + 机器可读定义）

## 硬性约束

1. **25 kHz 硬实时闭环**是设计规定。ISR 内禁止浮点、除法链、`printf`、阻塞调用；转速/位置/占空比归 ISR 单一写者，任务只读。ISR 占用率必须实测（DWT 或翻转 GPIO），预算 ≤ 周期 35 %。
2. **上电不得自动驱动电机**；必须按键或收到显式指令（`0x010 SAFETY_CMD` + 指令帧 `enable`）才动。测试先架空车轮。
3. **1 Mbps CAN 必须用 HSE 8 MHz**，骨架现在是 HSI——上总线前先改时钟树，否则偶发错误帧查到你怀疑人生。
4. 上板后**必须真断电冷启动**（电池关 → 拔 ST-LINK → 等 OLED 全黑 → 再上电），并看屏上 `build_tag` 判别实际在跑哪版；每次有效改动升一档标签。
5. 25 kHz 能否出扭矩有一条**未裁决的冲突记录**（台架说只有 10 kHz 能出扭矩）。实测不通过就停下上报，**不许擅自改频率**。
6. 共享常量（引脚、定时器、编码器计数）**单点定义**在板级头文件里，一个物理量只允许一个常数；参考课设与 AI 设计草案里的选型（TB6612、STBY、PB8/PB9 当 CAN、电流保护）**都是错的，不得引用**。
7. **注释署名约定**：AI/工具产出的注释一律以 `[AI]` 前缀标记（如 `/* [AI] 说明 */`），便于人工区分与回溯（`grep -rn "\[AI\]" Core/` 可列出全部）；**用户原有注释永不删除或改写**，需要调整先征询。

## 构建 / 烧录 / 提交

```bash
. ./env/activate.sh            # Linux（Windows 用 . .\env\activate.ps1；只需 STM32CubeCLT 1.22.0）
cmake --preset Debug && cmake --build --preset Debug     # 还要跑 Release
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_chassis.hex -v -rst
python3 tools/can_check.py                                 # 动了协议就必须通过
```

- 新 `.c/.h` 加进**顶层** `CMakeLists.txt`（`cmake/stm32cubemx/` 那份会被 CubeMX 覆盖）；只往 `USER CODE BEGIN/END` 标记对之间写。
- CubeMX 重生成后必查：`stm32f4xx_it.c` 不得恢复 SVC/PendSV/SysTick 三个 handler；`SystemClock_Config` 的 HSE 设置是否被抹回 HSI。
- 提交前：Debug + Release 零警告 + `can_check.py` 通过。开工先 `git pull`，收工就 push。
- 改 `docs/PROTOCOL.md` / `protocol-signals.yaml` / `docs/BOARD-C30D.md` 时，**同一批把姊妹仓 `vehical_rak_arm` 的那份一起改**（两仓各存一份，内容保持一致），并在提交说明里写明改了什么。
