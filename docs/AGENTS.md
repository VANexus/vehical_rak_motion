# AGENTS.md — 本工作区的行为约束

> 面向对象：AI 编码代理 + 人类成员（Windows / Linux 混合团队）。
> 目标硬件**永远是同一块板**：轮趣 C30D（STM32F407VET6 + 板载 AT8236）。差异只在开发机。

---

## 1. 现在的状态（别搞错阶段）

- 三份规范件是各自的 SSOT，与本文冲突时以它们为准（并记得回来修正本文）：
  - 分层与职责边界 → [`docs/architecture.md`](ARCHITECTURE.md)
  - 通信协议 → [`docs/protocol.md`](PROTOCOL.md) + [`docs/protocol-signals.yaml`](protocol-signals.yaml)（机器可读单一来源）
  - 板级硬件引脚 → [`docs/board-c30d.md`](BOARD-C30D.md)
- `vehical_rak_chassis/`、`vehical_rak_arm/` = **正式开发区的两个 MCU 工程**（同一块 C30D，只是角色不同），CubeMX 空骨架，应用代码从零开始。
- `rak/` = **Jetson 上位机（独立 git 仓，ROS 2 colcon）**。⚠️ **全仓没有任何 CAN 代码**（`nerve` 不存在、`CanFrame.msg` 已删；grep 命中的 `frame` 全是相机帧）。而且**两边都不干净**：本地 HEAD `3588521`（Orin 运行树的 snapshot，领先 origin 1 个提交）+ 未提交改动；Orin HEAD 仍是更早的 `1b2869e` 却有 34 个脏文件。**动 `rak` 前必须先对齐哪边为准**，否则会把对方的在制品覆盖掉。
- `car2/` = **只读归档**。那是之前在 Windows 上做的台架/极限测试，整个板子怎么设计、哪个引脚对应什么，那边的 `docs/` 与 `README3.md` 写得很全。**当参考资料用，不要在那里面开发，也不要"顺手整理"它。**
- 上一轮工程化（H5–H7 固件）的源码在 Windows 侧 `D:\STMproject\407`，不在本工作区。本仓库里任何"上一轮如何做"的描述都来自 `car2/README2.md`/`README3.md` 的转述，**不是本仓库可验证的代码**。
- 根目录还有 `README.md`（人读的总览/进度），本文件只写约束与操作。

### 事实分级（本项目最容易出错的地方）

五份 AI 设计草案（DeepSeek 对话）只参考**思想**（分层、频率、职责划分）。它的**选型与字段多处是幻觉**，已逐条核实为错：`4×TB6612FNG`、`STBY 引脚`、`SN65HVD230`、`CAN1_RX=PB8/TX=PB9`、`电流保护/电流检测`。
实物是：**板载 AT8236（双输入、无 STBY）+ 板载 CAN 收发器（PD0/PD1）+ 全板无电流采样通道**（详见 [`docs/board-c30d.md`](BOARD-C30D.md)）。

引用任何事实前先分级，措辞必须区分：**实测过（附出处或命令）→ 转述（待复核）→ 推断（明标）→ 草案幻觉（不得引用）**。把幻觉当既定事实写进代码注释或提交信息，比不知道更有害。

## 2. 跨端工具链（团队统一口径）

两端都用 **STM32CubeCLT 1.22.0**，所以工具版本、命令、产物完全一致：

| | Linux（已实测） | Windows（按 ST 默认安装位置推断，未实测） |
| --- | --- | --- |
| CubeCLT | `/opt/st/stm32cubeclt_1.22.0` | `C:\st\STM32CubeCLT_1.22.0` |
| 编译器 | GNU Tools for STM32 **14.3.rel1**（`arm-none-eabi-gcc` 14.3.1） | 同上 |
| cmake / ninja | 4.3.1 / 1.13.2 | 同上 |
| 烧录 | `STM32_Programmer_CLI` v2.23.0 | 同上 |
| 激活环境 | `. ./env/activate.sh` | `. .\env\activate.ps1` |

```bash
# 两端逐字相同（PowerShell 用 cmake.exe 也一样）
. ./env/activate.sh          # 唯一区别：Windows 用 activate.ps1
cd vehical_rak_chassis
cmake --preset Debug && cmake --build --preset Debug
# → build/Debug/vehical_rak_chassis.{elf,hex,bin} + .map
```

规则：

1. **先 source env，再谈"编译不过"。** 未加载环境时报的 `arm-none-eabi-gcc not found` 不算真问题。
2. **命令、预设、产物形态两端必须一致**；不一致就是 bug。只允许差异出现在 `.sh/.ps1` 内部的路径探测。
3. **禁止在源码、CMake、脚本里写任何绝对路径或盘符**（`D:\...`、`/home/xxx/...`）。要路径就走 `env/` 探测或 CMake 相对路径。
4. 别用 `car2/tools/arm-gnu/`（那是 Windows x86 的 GCC 12.2.1，只服务于 `car2/**/build.ps1` 旧脚本），正式工程统一 CubeCLT 14.3。
5. 编译器版本差异会改变二进制。**跨端比对不要比字节，要比 `.map` 里的 section 大小 + 上板行为**；`.bin` 的 SHA-256 只在同一台机器同一预设内做存档指纹用。
6. Linux 侧无 openocd；调试用 `ST-LINK_gdbserver` + `arm-none-eabi-gdb`，或干脆用 STM32CubeIDE/CLion 的 STM32CubeCLT 调试器。烧录一律 `STM32_Programmer_CLI`。
7. CubeMX 有两种可用方式：GUI（两端都有，`~/STM32CubeMX` / 开始菜单），Linux 侧另有无 GUI CLI `cube mx ...`（版本 1.1.1，**尚未在本项目验证过 `.ioc` 兼容性**，第一次用先备份 `.ioc`）。

## 3. 代码写在哪（分层约定）

分层定义与模块对应关系只在一处维护：**[`docs/architecture.md`](ARCHITECTURE.md) §3-§4**（L0-L5）。与本文冲突时以它为准，并记得回来修正本文。本处只列约束：

| 层 | 位置 | 规矩 |
| --- | --- | --- |
| L0 外设 | CubeMX 生成的 `main.c / gpio.c / freertos.c / stm32f4xx_it.c / *_hal_msp.c` | 只当骨架，**只往 `/* USER CODE BEGIN xxx */ ... /* USER CODE END xxx */` 之间写**；区域外重生成会被抹掉 |
| L1 驱动 / L2 实时 / L4 安全 / L5 通信 | `platform/`（两端共用，**尚未创建**，Phase 3 建立） | 两个工程的**顶层** `CMakeLists.txt` 引用同一份源码；`platform/` 内不得 include 工程专属头文件，板级差异全部走每工程各一份的 `board_c30d.h` |
| L3 解算 | 底盘 `app_mecanum`；机械臂 `app_traj` + `app_limit` | **唯一允许两端不一致的一层** |

硬实时层（L2）的额外约束：

- **25 kHz ISR 是全系统最高优先级，不受调度器影响**；ISR 内禁止：浮点、除法链、`printf`、阻塞队列/mutex、大数组操作。
- 转速、位置累计、占空比 **归 ISR 单一写者**，上层只读。
- ISR 占用率必须实测（DWT 周期计数，或翻转 GPIO 用逻辑分析仪），预算 ≤ 周期 35%；没实测数据不算完成。
- 定点运算优先；常数换算全部集中在 `board_c30d.h`（一个物理量只允许一个常数）。

其他：

- 每个新 `.c/.h` 都要加进**顶层** `CMakeLists.txt` 的 `target_sources` / `target_include_directories`。
- 源码统一 **UTF-8 + LF**，注释允许中文；`.gitattributes` 已强制源码 LF，防止 Windows 成员写进 CRLF。
- 单个模块对外只暴露必要的函数与常量；共享常量（引脚表、编码器计数、CAN 帧定义）**单点定义**，不要各处复制。
- 职责边界不要越层放错地方：**IK 与轨迹规划在 Jetson（MoveIt），SLAM/Nav2 在 Jetson，IMU（LSM6DSRTR）融合在 Jetson**；下位机只做闭环 + 执行 + 本地安全兜底。注意板上还有一颗 ICM20948（PB10/11/12），**谁是航向权威源未定**，不要默默把 MCU 侧 IMU 代码写进去当结论。

## 4. 板级事实的取用规则

`car2/docs/hardware.md` 与 `car2/README3.md` §6 有完整引脚表（PWM/编码器/CAN/OLED/按键/电池 ADC/使能开关/IWDG），以及台架期实测出的驱动边界（静摩擦约 75% 起步占空比、刹车滑行 0.10–0.11 圈、禁止反转消过冲、40 µs 测速窗不可用）。

**PWM / 控制频率 = 25 kHz 硬实时闭环，这是设计规定**（底盘与机械臂各自 4 路，共 8 路）。台架期有一条与它冲突的记录（10 kHz 才能出扭矩），把它当**风险项**看待：首次上板实测确认；**不通过就停下上报，不自作主张降频**。

- 用之前**先复核**：`Tools > CubeMX` 里当前 `.ioc` 的配置 + `Drivers/` 的实际引脚，才是本工程的真相；car2 那张表描述的是另一份代码。
- 已知未定量或待复核的条目（`IK_PERMM`、`lx+ly`、每轮符号、PID 增益、FreeRTOS 栈深、**25 kHz 能否出额定扭矩**）——**不要把它们当已确认事实写进代码注释或提交信息**；要用就实测，实测就把数据记进当次提交说明。
- 硬件编号/相位（哪个电机是 A/B/C/D、各自接线角位）历史上凭印象出过错。**以原理图 PDF 或实拍图为准**，路径 `car2/tools/references/smart-transport-robot/01_课程设计-智能搬运机器人/装配参考资料/`。

## 5. 上板规矩（物理安全优先）

1. **烧录成功 ≠ 新固件在跑**：必须真断电——电池开关关 → 拔 ST-LINK → 等 OLED 全黑 → 开电池 → 插 ST-LINK。用固件里的 `build_tag` 判别实际运行版本。
2. 上电**不得自动驱动电机**；必须按键或收到显式指令才动。测试时**车轮架空**。
3. 电机不转先量蓝色端子电压，再怀疑代码（历史上 2.89 V 过放造成过"全不转"的假故障）。
4. 每次有效改动**升一个构建标签**（建议 `E0/E1/...`），冷启动后先在屏上确认标签再判行为。
5. 每次实测记录：`.bin` 的 SHA-256、供电电压、四路计数、终态、目视转向。没数据不许下结论。

## 6. 改之前 / 提交之前

- 改之前：`git status` 干净；读目标模块现状；一次只改一个变量。
- **动到通信就必须跑协议门禁**：`python3 tools/can_check.py` 退出码 0；改了 `docs/protocol-signals.yaml` 则同时跑 `--update` 与 `--c-header`，并把 YAML、生成头、`protocol.md` 的向量表 **一次性同提交**（三端对不上的根因就是只改了一处）。
- 提交之前：`cmake --build --preset Debug` 与 `--preset Release` **双双通过、零警告**；`.ioc` 若变了必须连同生成代码一起提交。
- 不许提交：`build/`、`cmake-build-*/`、IDE 工作区文件（已在 `.gitignore`）。
- **禁止**：`git push --force`、`reset --hard`、跳过钩子、未经确认的擦除/写 Flash 命令、删除 `car2/backup/`。
- 声称"已验证"必须附命令与实际输出；做不到就写"未验证"。

## 7. 工作约定

- 长任务：授权自主决策、一次性做完，不必中途反复确认；但结论必须区分"实测"与"推断"。
- 优先成熟现成方案（HAL/CubeMX/官方中间件），**不要手写驱动、启动文件、RTOS 内核这类替代实现**。
- 与用户交流用中文；代码与标识符用英文。
- 每个工程目录有自己的 `README.md`（该工程的构建、当前进度、待办）；改动影响构建或约定时**同步更新**，别让文档漂在代码后面。

## 8. 仓库拓扑与版本控制

四个目录，**各自是独立 git 仓**：

| 仓 | 内容 | 现状 |
| --- | --- | --- |
| `vehical_rak_chassis/` | 底盘 MCU 固件 | 已 `git init`（main），**尚未首次提交**；自包含可单独 clone 编译 |
| `vehical_rak_arm/` | 机械臂 MCU 固件 | 同上 |
| `rak/` | Jetson 上位机（ROS 2 colcon） | 有远端（`VANexus/rak`）。本地领先 origin 1 个 snapshot 提交 + 有脏文件；Orin 停在更早 HEAD + 34 个脏文件 → **先对齐** |
| 工作区根（伞形） | `docs/`（规范件源）、`tools/`、`env/`、`AGENTS.md`、`README.md` | 已 `git init`；`.gitignore` **不纳管**上面三个子仓 |

规则：

1. **规范件只能在伞形仓改**：`docs/protocol.md`、`docs/protocol-signals.yaml`、`docs/board-c30d.md`、`docs/architecture.md`、`AGENTS.md`、`tools/*.py`。改完跑 `python3 tools/sync_docs.py` 把它同步进两个 MCU 仓，**源与拷贝同一次提交**。
2. `sync_docs.py --check` 是漂移门禁（退出码 1 = 有人在仓里直改了副本）；各仓内的 `docs/PROTOCOL.md` 等**永远是同步产物，不直接编辑**。工具会顺手重写文内链接，使副本在仓内视角下依然可点。
3. 提交前在本仓跑：`python3 tools/can_check.py`（协议）+ `cmake --build --preset Debug`（构建），两者零问题。`Drivers/`、`Middlewares/` **要入库**（保证单独 clone 能编译）；`build/`、`cmake-build-*/`、`.idea/` 不入库。
4. 跨机协作铁律：**开工先 pull，收工先 push**；不要两个地方同时改同一个文件。`rak` 改动前先确认改的是哪一份（本地旧快照 vs Orin 工仓），避免把别人新推的东西又提回去。
5. 本轮只建仓不提交；伞形仓已排除两个带明文口令的历史脚本（`.expA.sh`、`.runtest.sh`，属其他任务遗留，请自行处置）。
