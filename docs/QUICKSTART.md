# 快速上手（底盘固件仓）

面向新加入的成员（Windows 或 Linux）。目标：**20 分钟内从 clone 到板上看到自己的固件版本号**。

---

## 0. 这个仓库是什么

`vehical_rak_chassis` = 麦轮**底盘**的 STM32F407VET6 固件（轮趣 C30D 一体板）。
同硬件的姊妹仓是 `vehical_rak_arm`（机械臂），上位机是 `rak`（Jetson/ROS 2）。三者靠 **CAN 2.0A @1 Mbps** 协作，协议见 [`docs/PROTOCOL.md`](PROTOCOL.md)。

**当前进度**：CubeMX 骨架能编译（Debug/Release 双预设，0 警告），**应用代码 0 行**，外设一个都没配。别期待板上已有闭环行为。

## 1. 装工具链（一次性）

只需要一个东西：**STM32CubeCLT 1.22.0**（含 GNU Tools for STM32 14.3.rel1、CMake、Ninja、STM32CubeProgrammer）。

| 系统 | 安装位置 | 备注 |
| --- | --- | --- |
| Linux | `/opt/st/stm32cubeclt_1.22.0`（本机已装） | 用 apt 装的 deb，或 [ST 官网](https://www.st.com/en/development-tools/stm32cubeclt.html) |
| Windows | 默认 `C:\st\STM32CubeCLT_1.22.0` | 装完不用手加 PATH，脚本会找 |

**版本必须对齐**：两端都 1.22.0（GCC 14.3）。编译器不同产出的二进制不可字节比对（见仓库根 `AGENTS.md` §2）。

## 2. 加载环境（每次开工）

```bash
. ../env/activate.sh          # Linux / macOS（bash 与 zsh 都实测可用）
```
```powershell
. ..\env\activate.ps1         # Windows PowerShell
```

找不到 CubeCLT 就显式指定：`STM32CubeCLT_DIR=/你的路径 . ../env/activate.sh`。
加载成功后会打印编译器/cmake/ninja/烧录工具的绝对路径——**先确认这几行再报"编译不过"**。

## 3. 构建

```bash
cmake --preset Debug && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
# 产物：build/Debug/vehical_rak_chassis.{elf,hex,bin,map}
```

当前基线（改动后应能对上数量级）：Debug FLASH 16 140 B（3.08 %）/ RAM 43 400 B（33.11 %）。

要加自己的 `.c/.h`：**改顶层 `CMakeLists.txt`** 的 `target_sources`/`target_include_directories`。
`cmake/stm32cubemx/CMakeLists.txt` 是 CubeMX 生成的，重生成会被整体覆盖，别往那里加。

## 4. 烧录

```bash
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_chassis.hex -v -rst
```

- Linux：ST-LINK 的 udev 规则已在 `/etc/udev/rules.d/49-stlinkv2*.rules`，用户在 `dialout` 组，插上即用。
- Windows：需装 STSW-LINK009 驱动（`car2/tools/stsw-link009`）。
- **烧完必须真断电冷启动**（见下一节），否则你测的还是旧固件。

## 5. 首次上板自检清单（按顺序做）

1. **车轮架空**，动力端子先不接；
2. ST-LINK 接好，上电，看 OLED；
3. 屏幕上确认**构建标签**（`build_tag`）就是你这次烧的版本；
4. 若屏幕黑：先量 MCU 供电与 OLED 接线（PD11-14），别急着改代码；
5. 接动力电源前，**量蓝色端子电压**（历史教训：2.89 V 过放被当成"电机全不转"的软件故障）；
6. 上电动作必须由按键或 CAN 指令显式触发，固件不得上电自转。

### 断电冷启动的标准动作（写进肌肉记忆）

> 电池开关关 → 拔 ST-LINK → 等 OLED **完全黑** → 开电池 → 插 ST-LINK。

原因：烧录成功不等于新固件在跑，内核的 ART 指令缓存要真断电才刷新。软复位后可能仍在执行旧代码。

## 6. 常见坑

| 现象 | 真原因 | 处置 |
| --- | --- | --- |
| `arm-none-eabi-gcc not found` | 没 source env（不是缺工具链） | 回到第 2 步 |
| 改了代码但行为没变 | 没真断电 | 第 5 节流程 |
| `.ioc` 改完后编译报重复定义 | CubeMX 把 SVC/PendSV/SysTick handler 加回来了 | 删掉这三个 handler（由 FreeRTOS 提供） |
| 自定义源文件消失了 | 写进了 `cmake/stm32cubemx/CMakeLists.txt` | 挪到顶层 `CMakeLists.txt` |
| 联机偶发收不到帧 | 时钟用 HSI 跑 1 Mbps | 切 HSE 8 MHz（见 `docs/PROTOCOL.md` §2） |
| 25 kHz 下电机不出扭矩 | AT8236 驱动边界（台架期有记录） | **停下上报**，不要擅自降频（见仓库根 README §3） |

## 7. 文档索引

| 文件 | 作用 |
| --- | --- |
| [`README.md`](../README.md) | 本仓现状、外设待配清单、模块划分 |
| [`docs/PROTOCOL.md`](PROTOCOL.md) | **CAN 协议 v3 规范**（本仓拷贝，源在工作区 `docs/protocol.md`） |
| [`docs/protocol-signals.yaml`](protocol-signals.yaml) | 协议机器可读单一来源 |
| [`docs/BOARD-C30D.md`](BOARD-C30D.md) | **板级引脚与外设资源表**（硬件真相） |
| [`docs/gen/can_protocol.h`](gen/can_protocol.h) | 由校验器生成的帧宏，MCU 侧直接 include |
| [`tools/can_check.py`](../tools/can_check.py) | 协议校验器 + 参考编解码（`python3 tools/can_check.py` 必须退出码 0） |
| 工作区根 [`../AGENTS.md`](AGENTS.md) | 跨端工具链、代码分层、上板规矩 |

⚠️ `docs/PROTOCOL.md`、`docs/BOARD-C30D.md`、`docs/protocol-signals.yaml`、`docs/gen/*`、`tools/can_check.py` 是**同步产物**（`tools/sync_docs.py` 从工作区根拷来），**不要在本仓直接改**；改源，再同步。
