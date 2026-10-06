# 验收清单

## 协议与下位机 codec

- [x] v2 frame/payload 向量与现有上位机 v2 codec 逐字节一致；required features 0/1/2 不变。
- [x] F407 测试覆盖坏 COBS、错误版本/长度、坏 CRC、非法/重复/旧序号和序号回绕。
- [ ] 未知或无效帧不更新任何命令、状态或 lease。
- [ ] v1 不作为活动协议接受，也没有自动回退路径。
- [ ] 下位机 v2 endpoint 支持 HELLO/ACK、drive/arm、状态、诊断、安全、时间同步和 fault event。

## 轮端控制、IMU 与里程计

- [ ] 四轮 command 顺序、rad/s 单位和 rad/s 到 counts/ms 转换经向量/边界测试验证。
- [ ] F407 仍是四轮等比饱和，25 kHz ISR 频率、单写者和实时约束没有回归。
- [ ] 板载 IMU 补偿在四轮目标域、幅度有界；没有 body `wz` 前置注入或零偏航航向保持。
- [ ] IMU 初始化失败或样本年龄超过 20 ms 时，drive 保持/转入 DISARMED、报告 wheel-control fault；恢复后不自动 ARM。
- [ ] 四轮 counts/rev、轮向、有效滚动半径和麦轮几何有实测记录、不确定度及签核。
- [ ] F407 上报轮位/轮速来自编码器一致采样，并包含 sample time、valid mask、encoder epoch。
- [ ] 编码器重置或丢失连续性时 epoch 更新；无效或过期样本不伪装为有效零速。
- [ ] 静止、直线、横移和原地旋转对照测试记录外部参考、wheel odometry 误差和不确定度。
- [ ] 未签核 wheel odometry 误差预算前，不宣称达到 Nav2/EKF 精度，也不接入 Nav2/EKF。
- [ ] F407 IMU 不经 USB 上报；Jetson SPI IMU 和 IMU/EKF 不在本阶段。

## ARM 与启动安全

- [ ] ARM v2 帧和状态可解析；commissioning gate 关闭时拒绝任何 ARMED ARM 命令，不驱动 ARM PWM。
- [ ] ARM_STATE measured valid mask 为零、source 为 unavailable，不将命令回显作为实测值。
- [ ] PC6 上电自检扫动已关闭；复位、冷启动、链路恢复均保持 DISARMED。
- [ ] 25 kHz ISR 中没有新增浮点、USB I/O、阻塞或协议解析。

## 构建与 DISARMED USB 验收

- [ ] F407 Debug/Release 零警告构建通过；只有改动 CubeMX 外设时才运行并通过 `cubemx_lint.sh`。
- [ ] build tag 为 R22；真断电冷启动后 OLED 标签与镜像一致。
- [ ] WCH 串口稳定设备路径识别正确，串口无其他进程占用。
- [ ] DISARMED USB HELLO/ACK、features 0/1/2、DRIVE_STATE、ARM_STATE、DIAGNOSTICS、CRC、序号、lease 和数据年龄验证通过。
- [ ] 实机测试只发送 DISARMED 零目标；无 ARMED 轮速、ARM 命令、非零目标或执行器动作。
- [ ] 断链、IMU stale、编码器无效和恢复链路测试均保持 fail-closed。
- [ ] 任何错误帧、故障、反馈异常或执行器动作均停止验收，不进入点动。

## 范围确认

- [ ] 本阶段没有新增或修改上位机源代码、C++ `rak_hardware`、Nav2、MoveIt2 或 ROS 2 controller 配置。
- [ ] 上位机现有 v2 codec 仅作为只读协议对照/测试 peer。
- [ ] 电机点动、Nav2 实车行驶、ARM/MoveIt 实机执行均未进行；这些动作需另行授权。
