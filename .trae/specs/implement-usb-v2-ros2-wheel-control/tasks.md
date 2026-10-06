# Tasks

- [x] Task 1：锁定 v2 wire contract 与 F407 codec 验证
  - [x] 对照 `docs/UART-v2-ALIGN.md` 和现有上位机 v2 协议实现，确认本阶段消息、required features 0/1/2、序号和独立 lease；不修改上位机文件。
  - [x] 以测试先行补齐 F407 C codec 的全部消息 payload 结构化编解码及 HELLO/ACK、DRIVE/ARM、状态、安全/诊断帧向量。
  - [x] 以测试先行覆盖非法 COBS、版本、payload 长度、CRC、序号、lease、feature mask 和 reserved field 边界。
  - [x] 编译运行下位机 standalone v2 codec 测试；如需对照，使用上位机现有 codec/test 只读验证。
- [ ] Task 2：接入 F407 v2 serial endpoint 与安全状态机（依赖 Task 1）
  - [ ] 以测试先行实现 COBS stream 接收、CRC/长度/版本/序号校验、命令分发和有界 TX。
  - [ ] 实现 HELLO/ACK、DRIVE_COMMAND/STATE、ARM_COMMAND/STATE、DIAGNOSTICS、SAFETY、TIME_SYNC 和 FAULT_EVENT。
  - [ ] 实现 drive/arm 独立 lease；非法、重复、过期或 CRC 错帧不得刷新 lease。
  - [ ] 验证活动 endpoint 只接受 v2；不得同时运行或自动回退到 v1。
  - [ ] required features 0/1/2 如实握手；ARM codec/状态可用，但 commissioning gate 未开启时拒绝 ARMED ARM 命令且不驱动 PWM。
  - [ ] 冷启动、复位、链路重连后 drive/arm 保持 DISARMED，关闭 PC6 自动扫动。
- [ ] Task 3：实现 IMU 轮级稳定、编码器反馈与轮式里程计（依赖 Task 2）
  - [ ] 保留板载 IMU 稳定职责，将 R21 body `wz` 前置修正迁为有界 wheel-target 补偿；无独立 body yaw 目标和航向保持。
  - [ ] IMU 样本年龄超过 20 ms 或初始化失败时拒绝/撤销 drive ARM，置 v2 wheel-control fault；恢复后不自动使能。
  - [ ] 四轮目标从 rad/s 在任务上下文换算为 counts/ms；保留 25 kHz ISR、单写者、共同饱和和轮速闭环。
  - [ ] 逐轮测量 counts/rev、方向、有效滚动半径和麦轮几何，记录方法/不确定度并签核；未签核值不得作为实机配置。
  - [ ] 由一致编码器样本产生 wheel position/velocity、sample time、valid mask、encoder epoch；不经 USB 上报 F407 IMU。
  - [ ] 完成静止、直线、横移、原地旋转的手动转轮/推行对照测试，记录外部参考、误差和不确定度；Nav2/EKF 使用精度另设误差预算，不在本阶段宣称达标。
- [ ] Task 4：下位机构建、DISARMED USB 联通和协议验收（依赖 Task 3）
  - [ ] F407 v2 codec/endpoint 共享向量全部通过；运行 v2 host codec 只读回归，不修改上位机代码。
  - [ ] 运行下位机 Debug 与 Release 构建；仅在改 CubeMX 外设配置时运行 `cubemx_lint.sh`。
  - [ ] build tag 升至 R22；烧录前确认车轮架空、动力隔离、IMU 有效、PC6 启动扫动已关闭；真断电冷启动后核对 OLED 标签。
  - [ ] 用现有 v2 peer/协议工具连接稳定 WCH 设备路径，验证 HELLO/ACK、features 0/1/2、轮状态、ARM DISARMED 状态、诊断、CRC、序号、lease、encoder epoch 和数据年龄。
  - [ ] 仅进行 DISARMED 零目标握手，不发 ARMED 轮速/关节命令、不点动；若出现错误帧、数据失效、故障或执行器动作，立即停止。

## Task Dependencies

- Task 1 是 F407 endpoint 实现的前置条件。
- Task 2 依赖 Task 1。
- Task 3 依赖 Task 2 的 wheel command/state 接口。
- Task 4 依赖 Tasks 1–3。
- 本 change 不安排上位机 C++ 插件、Nav2、MoveIt2 或 ROS 2 配置实现。
