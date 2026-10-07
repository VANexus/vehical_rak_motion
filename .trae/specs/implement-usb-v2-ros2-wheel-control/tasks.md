# Tasks

- [x] Task 1：锁定 v2 wire contract 与 F407 codec 验证
  - [x] 对照 `docs/UART-v2-ALIGN.md` 和现有上位机 v2 协议实现，确认本阶段消息、required features 0/1/2、序号和独立 lease；不修改上位机文件。
  - [x] 以测试先行补齐 F407 C codec 的全部消息 payload 结构化编解码及 HELLO/ACK、DRIVE/ARM、状态、安全/诊断帧向量。
  - [x] 以测试先行覆盖非法 COBS、版本、payload 长度、CRC、序号、lease、feature mask 和 reserved field 边界。
  - [x] 编译运行下位机 standalone v2 codec 测试；如需对照，使用上位机现有 codec/test 只读验证。
- [x] Task 2：接入 F407 v2 serial endpoint 与安全状态机（依赖 Task 1）
  - [x] 以测试先行实现 COBS stream 接收、CRC/长度/版本/序号校验、命令分发和有界 TX。
  - [x] 实现 HELLO/ACK、DRIVE_COMMAND/STATE、ARM_COMMAND/STATE、DIAGNOSTICS、SAFETY、TIME_SYNC 和 FAULT_EVENT。
  - [x] 实现 drive/arm 独立 lease；非法、重复、过期或 CRC 错帧不得刷新 lease。
  - [x] 验证活动 endpoint 只接受 v2；不得同时运行或自动回退到 v1。
  - [x] required features 0/1/2 如实握手；ARM codec/状态可用，但 commissioning gate 未开启时拒绝 ARMED ARM 命令且不驱动 PWM。
  - [x] 冷启动、复位、链路重连后 drive/arm 保持 DISARMED，关闭 PC6 自动扫动。
- [x] Task 2A：封闭 DMA 覆写和会话安全回归缺口（依赖 Task 2）
  - [x] 接收 DMA 环形缓冲能检测生产位置越过未消费数据；完整环回不得静默误判为无新数据。
  - [x] 生产游标更新不依赖可能迟到/乱序的 RxEventCallback `Size`；HT/TC/IDLE 交错时以当前 DMA 状态作单调、安全的生产进度。
  - [x] overrun 后丢弃未完成帧、按 COBS 定界符重新同步并增加诊断计数；被覆盖字节不得刷新命令或 lease。
  - [x] USART 错误 ISR 只设置恢复标志；DMA 重装与 64 位生产/消费游标重置由任务单写者执行，避免与 RX 排水竞争。
  - [x] 初始 DMA 接收或错误后的 HAL 重武装失败必须可检测；每次 1 kHz service 每端口最多尝试一次非阻塞重装，不得忙循环，持续失败保持 DISARMED 并可观察。
  - [x] 增加 SAFETY STOP、LATCH/CLEAR、ARM commissioning lock、跨 UART owner 端口拒绝和 ring overrun 的自动回归测试。
- [ ] Task 3：实现 IMU 轮级稳定、编码器反馈与轮式里程计（依赖 Task 2A）
  - [ ] 保留板载 IMU 稳定职责，将 R21 body `wz` 前置修正迁为有界 wheel-target 补偿；无独立 body yaw 目标和航向保持。
  - [ ] IMU 样本年龄超过 20 ms 或初始化失败时拒绝/撤销 drive ARM，置 v2 wheel-control fault；恢复后不自动使能。
  - [ ] 四轮目标从 rad/s 在任务上下文换算为 counts/ms；编码器方向只归一化一次，稳定补偿后的四轮目标使用共同饱和；保留 25 kHz ISR 和轮速闭环。
  - [ ] 四轮目标以原子/版本化快照交给控制任务，控制任务不得消费跨帧混合目标。
  - [ ] 停用 CAN v3 作为第二个 drive/arm 命令入口；CAN 速度、START/CLEAR 不得改变 USB v2 状态，紧急停止只允许单向撤销使能。
  - [ ] USB v2 LATCH_INHIBIT 与 CAN 急停都进入车端 latch/25 kHz 制动路径；STOP 受控减速与急停锁存语义分开。
  - [ ] 急停锁存优先于 RUNNING/CALIBRATING/self-test/REMOTE 的所有 ISR 输出；USB v2 clear 原子清除旧目标、pending motor command 和 bench/calibration 状态，保持 DISARMED。
  - [ ] drive clear 后要求新序号 DISARMED 零目标确认，再允许新的 ARMED DRIVE；同一 RX 批次/clear 前已排队的 ARMED 帧必须拒绝且不刷新 lease。
  - [ ] 文档和验收流程明确 C30D 无独立急停反馈输入；物理急停释放由操作员确认，固件不得声称检测到该状态。
  - [ ] 逐轮测量 counts/rev、方向、有效滚动半径和麦轮几何，记录方法/不确定度并签核；未签核值不得作为实机配置。
  - [ ] 编码器位置、速度、sample time、valid mask 和 encoder epoch 由同一代快照生成；reset 并发不得混合新旧 epoch，时间戳不得领先 readiness 检查时钟。
  - [ ] 不经 USB 上报 F407 IMU；保持一致样本和方向处理。
  - [ ] 完成静止、直线、横移、原地旋转的手动转轮/推行对照测试，记录外部参考、误差和不确定度；Nav2/EKF 使用精度另设误差预算，不在本阶段宣称达标。
- [ ] Task 4：下位机构建、DISARMED USB 联通和协议验收（依赖 Task 3）
  - [ ] F407 v2 codec/endpoint 共享向量全部通过；运行 v2 host codec 只读回归，不修改上位机代码。
  - [ ] 运行下位机 Debug 与 Release 构建；仅在改 CubeMX 外设配置时运行 `cubemx_lint.sh`。
  - [ ] build tag 升至 R23；烧录前确认车轮架空、动力隔离、IMU 有效、PC6 启动扫动已关闭；真断电冷启动后核对 OLED 标签。
  - [ ] 用现有 v2 peer/协议工具连接稳定 WCH 设备路径，验证 HELLO/ACK、features 0/1/2、轮状态、ARM DISARMED 状态、诊断、CRC、序号、lease、encoder epoch 和数据年龄。
  - [ ] 仅进行 DISARMED 零目标握手，不发 ARMED 轮速/关节命令、不点动；若出现错误帧、数据失效、故障或执行器动作，立即停止。

## Task Dependencies

- Task 1 是 F407 endpoint 实现的前置条件。
- Task 2 依赖 Task 1。
- Task 2A 依赖 Task 2 的会话接口；Task 3 依赖 Task 2A 的 RX/TX 路径稳定。
- Task 4 依赖 Tasks 1–3。
- 本 change 不安排上位机 C++ 插件、Nav2、MoveIt2 或 ROS 2 配置实现。
