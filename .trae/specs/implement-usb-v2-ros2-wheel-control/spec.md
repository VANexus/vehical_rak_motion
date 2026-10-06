# USB v2 轮端控制与 ROS 2 链路规格

## Why

当前 F407 运行 UART v1 机体速度 endpoint，而 v2 仅有独立 codec。需要优先完成下位机 UART v2 endpoint、板载 IMU 轮级稳定和可信轮状态/里程计上报；本阶段不实现上位机 ROS 2 硬件接口。

## What Changes

- **BREAKING**：F407 与 Jetson 的活动 USB 控制协议切换为 UART v2；不得自动回退到 v1。
- 接入完整 v2 required features 0/1/2：四轮速度命令、六轴位置命令 endpoint、轮编码器里程计。
- F407 接收四轮轮速目标、执行有界 IMU 轮级补偿和四轮等比饱和，保留 25 kHz 电机闭环；ARM endpoint 实现但未 commissioning 时锁闭实机输出。
- 完成每轮编码器、有效轮半径、轮向和麦轮几何标定；F407 以 v2 DRIVE_STATE 上报测量轮位/轮速。
- 将本阶段轮里程计作为下位机测量状态链路验收对象；Nav2/EKF 和 Jetson SPI IMU 里程计留待后续阶段。
- IMU 无法初始化或数据过期时，drive 保持或转入 DISARMED，并通过 v2 wheel-control fault 上报；不得静默退化为无稳定补偿运行。
- 禁用当前 PC6 上电舵机自检扫动；启动和重连保持 DISARMED。
- 上位机现有 v2 codec 只作为协议对照/测试工具；本阶段不修改上位机控制、Nav2、MoveIt2 或 `ros2_control` 代码。

## Impact

- Affected specs: UART v2 wire contract、F407 轮端控制、轮状态/里程计、IMU 轮级稳定和 fail-closed 安全状态机。
- Affected code: 下位机 `Core/Src/app_serial.c`、`Core/Src/app_protocol_v2.c`、`Core/Src/app_car.c`、`Core/Src/app_imu.c`、启动舵机自检路径及下位机 v2 验证工具。
- Host boundary: 不修改 `rak` 源码或配置；允许调用已有 v2 codec/测试实现作为只读协议对照或串口 peer。
- 外设配置原则：不改 `.ioc`；若实施中证明必须改 CubeMX 外设配置，单独按 `docs/CUBEMX-SOP.md` 操作并通过 CubeMX 门禁。
- Hardware acceptance: 本规格要求 DISARMED USB 握手、编码器轮状态、轮级 IMU 稳定逻辑和轮式里程计标定/验证；不授权电机点动或 ARM 实机动作。

## ADDED Requirements

### Requirement: UART v2 是唯一活动控制协议

系统 SHALL 在 Jetson 与 F407 的 USB serial 链路上使用协议 v2 的 COBS 定界、CRC-16/CCITT-FALSE、独立序号、严格 payload 长度、HELLO/HELLO_ACK、诊断、安全消息和独立 lease。

v2 握手 SHALL 要求 feature bits 0、1、2 全部存在。v1 机体速度协议 SHALL NOT 被静默协商或用作 fallback。串口设备 SHALL 使用稳定设备身份路径，且同一时刻只能由一个主机进程打开。

#### Scenario: v2 完整能力握手

- **WHEN** 唯一 USB 主机向 F407 发送合法 HELLO
- **THEN** F407 回报 board ID、firmware version、boot ID 和 features 0/1/2
- **AND** 任一 required feature 缺失、版本不匹配或 CRC/序号无效时，主机保持 DISARMED
- **AND** 主机不得改用 v1 格式重试控制

### Requirement: 四轮速度命令由上位机做麦轮 IK

v2 DRIVE_COMMAND SHALL 固定使用前左、前右、后左、后右顺序，在线路上传输 rad/s 轮速。本阶段 F407 SHALL 只接收四轮目标，不得在 MCU 对 body Twist 执行麦轮逆运动学；上位机控制器实现不属于本阶段。

F407 SHALL 对最终四轮目标执行一个共同饱和系数，不得逐轮独立截断。轮目标到 MCU counts/ms 的转换 SHALL 在任务上下文完成，不得在 25 kHz ISR 中执行浮点或阻塞工作。

#### Scenario: 同时平移与旋转的轮目标超限

- **WHEN** 上位机输出的四轮目标任一超出已标定轮速上限
- **THEN** F407 将四个目标乘以同一饱和系数
- **AND** 四轮目标间的相对比例保持不变

### Requirement: 板载 IMU 执行轮级稳定

F407 SHALL 保留板载 IMU 的轮级稳定/滑移补偿职责。稳定补偿 SHALL 作用于四轮目标域，使用 IMU 与编码器反馈产生有界修正；SHALL NOT 在 MCU 侧改写上位机 body `wz` 命令或生成独立机体航向目标。

v2 实现 SHALL NOT 复用 R21 中“修改 body `wz` 后再做 MCU IK”的稳定路径。Nav2 请求零偏航时，IMU 稳定逻辑 SHALL NOT 建立航向保持指令与 Nav2 竞争。IMU 与补偿计算 SHALL 位于任务上下文，25 kHz ISR 仅消费已计算目标。

若 IMU 初始化失败，或最后一次有效 IMU 数据超过 20 ms，系统 SHALL 拒绝 drive ARM；运行中出现该条件时 SHALL 进入 DISARMED 并报告 v2 fault bit 5（wheel control/motor fault）。恢复有效 IMU 数据后不得自动重新 ARM。

#### Scenario: IMU 稳定输入失效

- **WHEN** IMU 无法初始化，或有效样本年龄超过 20 ms
- **THEN** F407 拒绝或撤销 drive ARM
- **AND** F407 保持零目标/DISARMED 并上报 wheel-control fault
- **AND** 恢复样本后等待新的显式使能，不自动恢复运动

#### Scenario: IMU 轮级补偿有界

- **WHEN** IMU 与编码器检测到偏航跟踪误差或滑移
- **THEN** 稳定器只产生限幅后的四轮目标修正
- **AND** 补偿与四轮共同饱和一起验证
- **AND** 不生成独立 body yaw setpoint，不抵消 Nav2 的有效 `angular.z` 命令

### Requirement: F407 上报测量轮状态并完成轮式里程计

F407 SHALL 基于四路编码器的一致采样，提供轮位置、轮速度、采样时刻、valid mask 和 encoder epoch。轮状态 SHALL 是传感器测量值，不得由命令目标伪造。

F407 SHALL 以 v2 DRIVE_STATE 发布测量轮状态；四轮 counts/rev、有效滚动半径、轮向及麦轮几何 SHALL 经本阶段测量并形成签核记录。上位机如何消费 wheel odometry 不属于本阶段。

本阶段 SHALL 验证编码器状态的单位、方向、采样时间、有效性与 epoch，并以静止、直行、横移和原地旋转数据与外部测量对照，记录误差与不确定度。未签核误差预算前，不得宣称该轮里程计满足 Nav2/EKF 精度要求。Nav2/EKF 及 Jetson SPI IMU 里程计不属于本阶段。

#### Scenario: 轮式里程计使用有效测量

- **WHEN** 编码器样本有效且硬件参数已签核
- **THEN** 主机将轮位置/速度转换为一致的 ROS wheel state 并发布 wheel odometry
- **AND** 反馈带有采样时间、数据有效性和 encoder epoch
- **AND** F407 发送的状态可由既有 v2 codec/peer 正确解码

#### Scenario: 标定数据缺失或编码器反馈无效

- **WHEN** 任一轮参数未签核，或反馈 valid mask/数据年龄不满足要求
- **THEN** F407 清除对应轮状态有效位或拒绝 drive ARM，并报告 wheel-control fault
- **AND** 无效轮状态不得作为有效测量数据输出

### Requirement: 上电无执行器自启动

v2 固件 SHALL 在复位、冷启动和重连后保持 DISARMED，目标为零，不自动驱动电机或舵机。当前 PC6 舵机自检扫动 SHALL 在 v2 profile 中关闭；诊断扫动如保留，必须通过显式诊断入口触发且默认关闭。

#### Scenario: MCU 冷启动

- **WHEN** R22 固件上电或复位
- **THEN** drive 与 arm 均 DISARMED，四轮目标为零
- **AND** 不发生 PC6 自动扫动

## MODIFIED Requirements

### Requirement: F407 USB 控制入口

当前 UART v1 body-twist endpoint SHALL 被完整 v2 endpoint 取代为活动控制入口。v1 历史 codec/测试可保留，但运行固件与主机不得自动降级或同时占有 USB 设备。

### Requirement: 当前工作范围限于下位机与协议

本阶段 SHALL 修改和验证 F407 固件与 v2 wire contract。上位机现有 v2 codec 可作为只读 peer/参考，但 SHALL NOT 修改上位机驱动、C++ 硬件插件、Nav2、MoveIt2 或 `ros2_control` 配置。

## REMOVED Requirements

### Requirement: MCU body-twist 麦轮逆运动学

**Reason**: 统一 ROS 2 关节级控制架构要求麦轮 IK 由上位机 `mecanum_drive_controller` 完成。

**Migration**: F407 接收已计算的四轮角速度并负责定点换算、IMU 轮级稳定、共同饱和、编码器反馈和 25 kHz 闭环；上位机相关实现留待后续阶段。
