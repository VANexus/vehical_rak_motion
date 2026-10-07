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

#### Scenario: DMA 接收环形缓冲溢出

- **WHEN** USART DMA 生产位置追上或越过尚未消费的接收数据
- **THEN** F407 检测并计数 RX overrun，不得把环回到相同 NDTR 位置误判为“没有新数据”
- **AND** 丢弃不完整帧并重新同步到后续 COBS 定界符
- **AND** 被覆盖/不完整帧不得更新命令、状态或 lease
- **AND** HT/TC/IDLE 回调乱序不得回退或虚增单调生产位置

#### Scenario: DMA 重武装失败

- **WHEN** USART 初始接收或 HAL 错误回调中的 `ReceiveToIdle_DMA` 重武装返回失败
- **THEN** F407 记录 RX 错误，并由 1 kHz service 在任务上下文每端口最多发起一次非阻塞重试
- **AND** 端口停收状态不得静默持续或刷新命令 lease
- **AND** USART 错误 ISR 只置恢复标志；DMA 重装和 64 位 RX 游标重置由任务单写者执行
- **AND** 单次 service 不得进入无界重试循环，持续失败时状态和错误计数必须可观察

### Requirement: 四轮速度命令由上位机做麦轮 IK

v2 DRIVE_COMMAND SHALL 固定使用前左、前右、后左、后右顺序，在线路上传输 rad/s 轮速。本阶段 F407 SHALL 只接收四轮目标，不得在 MCU 对 body Twist 执行麦轮逆运动学；上位机控制器实现不属于本阶段。

- F407 对应用编码器方向归一化后的四轮目标 SHALL 只执行一次 encoder sign 变换；已经归一化的目标 SHALL NOT 再乘方向符号。
- F407 对稳定补偿后的四轮目标 SHALL 施加同一个饱和系数，不得逐轮独立截断。
- 四轮目标 SHALL 以一致快照提交给控制任务，不能让控制任务读到跨帧混合的轮目标；编码器位置与 encoder epoch SHALL 在重置并发时保持同一代快照。

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

F407 SHALL 基于四路编码器的一致采样，提供轮位置、轮速度、采样时刻、valid mask 和 encoder epoch。轮状态 SHALL 是传感器测量值，不得由命令目标伪造。采样时刻 SHALL 与本次编码器快照一致，年龄判断 SHALL 使用同一或更新的单调时钟样本，不得把微小的未来时间戳误判为超时。

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

### Requirement: 急停锁存优先于所有电机输出模式

F407 的急停锁存 SHALL 在 25 kHz ISR 的任何 RUNNING、CALIBRATING、bench/self-test、REMOTE 输出分支之前检查。锁存有效时，本周期及后续周期 SHALL 持续输出安全制动/零目标；任何校准脉冲、kick、PWM 更新不得覆盖急停输出。CAN v3 急停只允许置锁存/停机，不得启动或清锁。

USB v2 `SAFETY_COMMAND.LATCH_INHIBIT` SHALL 触发同一底盘急停锁存和 25 kHz 制动路径；`STOP` 只执行协议定义的受控停止，不得与锁存急停混为一谈。LATCH 在 remote、bench、self-test 和 calibration 模式都必须阻止后续 PWM 输出。

#### Scenario: 急停发生在校准或 bench 运行期间

- **WHEN** 急停锁存于 CALIBRATING、RUNNING 或 self-test 模式
- **THEN** 当前 25 kHz 周期即执行制动/零目标
- **AND** 后续 ISR 不得再次输出校准 PWM、kick duty 或其他驱动占空比
- **AND** 只有满足安全条件的显式 clear 才可解除锁存，恢复后仍保持 DISARMED

#### Scenario: 显式清除急停锁存

- **WHEN** USB v2 的合法 CLEAR_INHIBIT 通过 owner、序号及 safety condition 校验，PD3 本地使能开关关闭且内部故障已清
- **THEN** F407 原子清除 pending `motor_command`、旧 drive/arm 目标、所有 enable 和 bench/calibration 启动状态
- **AND** 车端急停 latch 与 session inhibit 作为同一清锁事务解除，不得只清其中一层
- **AND** 急停锁存解除后仍为 DISARMED/零目标
- **AND** 只有新的、后续的显式 v2 drive ARM 命令可以请求轮端使能；CAN clear 或旧排队命令不得恢复运动
- **AND** drive clear 后必须先接收更新序号的 DISARMED 零目标确认帧；此帧之前到达的 ARMED DRIVE（包括同一 RX 批次内的帧）必须拒绝且不得刷新 lease
- **AND** 操作员必须先物理确认外部急停已释放；C30D 当前没有独立急停反馈输入，固件不得声称已自动检测其释放

## MODIFIED Requirements

### Requirement: F407 USB 控制入口

当前 UART v1 body-twist endpoint SHALL 被完整 v2 endpoint 取代为活动控制入口。v1 历史 codec/测试可保留，但运行固件与主机不得自动降级或同时占有 USB 设备。

### Requirement: 当前工作范围限于下位机与协议

本阶段 SHALL 修改和验证 F407 固件与 v2 wire contract。上位机现有 v2 codec 可作为只读 peer/参考，但 SHALL NOT 修改上位机驱动、C++ 硬件插件、Nav2、MoveIt2 或 `ros2_control` 配置。

CAN v3 SHALL NOT 作为第二个 drive/arm 命令入口；本阶段 USB v2 是唯一可改变轮目标或 ARM 状态的远程控制协议。CAN 紧急停止只允许作为单向禁止/停止输入保留，CAN 速度、START 或清除锁存命令不得改变 v2 wheel target 或重新使能。

#### Scenario: CAN v3 不与 USB v2 争用执行器

- **WHEN** F407 收到 CAN v3 速度、START 或 clear 命令
- **THEN** 这些帧不得改变当前 v2 command、lease 或 arm 状态
- **AND** CAN 紧急停止可以撤销 drive/arm enable，但不得建立新的运动许可

## REMOVED Requirements

### Requirement: MCU body-twist 麦轮逆运动学

**Reason**: 统一 ROS 2 关节级控制架构要求麦轮 IK 由上位机 `mecanum_drive_controller` 完成。

**Migration**: F407 接收已计算的四轮角速度并负责定点换算、IMU 轮级稳定、共同饱和、编码器反馈和 25 kHz 闭环；上位机相关实现留待后续阶段。
