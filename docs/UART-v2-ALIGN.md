# Jetson-F407 UART v2 alignment

> ⚠️ **历史备用链路（2026-10-28）**：本文的 USB v2 是**单板时期**的备用链路。
> 当前 **CAN v4 是唯一主链路**，机械臂板已恢复（`vehical_rak_arm`），舵机迁到该板；
> 本文的端点契约仍作为 USB 备用路径的参考，但**不是当前主链路**。

Status: wire contract fixed in the Jetson repository at
[`docs/protocol/usb_serial_v2.md`](https://github.com/VANexus/rak/blob/main/docs/protocol/usb_serial_v2.md).
This file records the
firmware-side implementation boundary. It is not a claim of endpoint
compatibility or physical acceptance.

## Agreed contract

- One Jetson and one F407 use the dedicated USB serial connection; CAN v3 is
  retained as dormant historical material and is not on this host link.
- UART framing remains COBS, zero delimiter, CRC-16/CCITT-FALSE, little-endian,
  independent 16-bit sequence spaces and exact payload lengths. The protocol
  version is 2. A v1 body-twist peer is incompatible and is never an automatic
  fallback.
- The host sends four wheel angular velocities in `rad/s`, ordered front-left,
  front-right, rear-left, rear-right; six arm joint positions in `rad`; an
  optional seventh calibrated servo angle is reserved for the independent
  gripper. All wire physical values are signed micro-units (`1e-6`).
- Wheel feedback is measured cumulative encoder position and measured speed,
  with an encoder reset epoch, validity mask and MCU acquisition uptime.
  Command echo is not physical feedback.
- The six arm axes are positional slots. The actual joint names, signs, zeroes,
  limits, PWM mapping, feedback sensor and gripper channel remain installation
  calibration data. No values are inferred from the simulation URDF.
- Drive and arm have independent 10-200 ms leases; 100 ms is the recommended
  initial v2 setting at 100 Hz. This does not alter the v1 bench setting
  (200 ms) or claim that the v2 value has been bench validated.
- Jetson SPI LSM6DSRTR is the body IMU for EKF. Board IMU stays local; it must
  not create a competing body-yaw command. EKF alone publishes `odom` to
  `base_link`; slam_toolbox alone publishes `map` to `odom`.

## Implemented in this repository

`Core/Inc/app_protocol_v2.h` and `Core/Src/app_protocol_v2.c` contain standalone
frame/CRC/COBS and typed codecs for the UART v2 messages. They are included by
the top-level firmware CMake target. `tools/uart_v2_codec_test.c` is a host
compiler golden-vector test:

```bash
cc -std=c11 -Wall -Wextra -Werror -Wpedantic -ICore/Inc \
  Core/Src/app_protocol_v2.c tools/uart_v2_codec_test.c \
  -o /tmp/uart_v2_codec_test
/tmp/uart_v2_codec_test
```

`Core/Inc/app_serial_session.h` and `Core/Src/app_serial_session.c` implement
the host-testable v2 session: bounded COBS stream framing, HELLO/ACK and
required-feature negotiation, independent RX/TX sequence spaces, independent
drive/arm leases, safety/fault handling, typed state/diagnostics/time-sync
transmission, and bounded per-port TX queues. `tools/uart_v2_session_test.c`
exercises the runtime with the host compiler.

`app_serial.c` connects this session to USART1/USART3 circular RX DMA and
single-frame TX DMA. The active serial path accepts v2 only; the v1 endpoint
and raw `ID:A` banner are not active. Required feature bits 0/1/2 are
advertised and negotiated. ARM commands remain commissioning-locked. Drive
ARM remains closed in this task because wheel calibration and valid fresh
four-wheel feedback are not yet available; DRIVE_STATE reports a zero validity
mask and no sample timestamp rather than synthesizing a measurement.

This is endpoint implementation status, not physical acceptance. Do not
authorize nonzero wheel or arm commands until the remaining calibration,
feedback, and supervised hardware gates below are complete.

## Remaining firmware and hardware work

1. Task 3 must provide signed wheel calibration, valid/fresh encoder sampling,
   wheel-domain IMU compensation, and the approved counts-to-wheel conversion.
   Until those checks pass, the application deliberately supplies false
   calibration and feedback readiness to the v2 drive session.
2. Complete calibrated six-axis arm routing and active-hold behavior only after
   commissioning approval. The present endpoint parses arm commands and
   publishes unavailable measured state without driving servo PWM.
3. Perform supervised physical acceptance with wheels clear and the arm
   mechanically supported. This implementation has only host tests and
   Debug/Release cross-build evidence; it has not been tested on hardware.

The Jetson SPI sensor bridge, ros2_control hardware plugin, EKF configuration,
wheel geometry/calibration, six-axis joint calibration and gripper channel are
separate gates. A passing codec test does not close any of them.

## 急停清锁约定

C30D 没有独立的外部 E-stop 反馈输入。操作者发送 USB v2 `CLEAR_INHIBIT`
前，必须亲自确认外部急停按钮/回路已经释放；该命令代表操作者确认，不代表
固件检测到了外部急停状态。固件另外要求 PD3 本地 enable switch 处于 OFF、
内部车控故障已清且没有待处理的 CAN 急停请求。CAN clear/START 不清锁、不启动。

清锁事务会丢弃旧电机命令、bench/calibration/run 状态和轮目标/enable，并保持
DISARMED。清锁本身不恢复运动：host 必须先发送序号晚于 CLEAR 的零值 DISARMED
DRIVE_COMMAND 确认；确认前到达的任何 ARMED DRIVE_COMMAND 都会被拒绝且不刷新
drive lease。确认后仍须 IMU、标定、反馈和故障门禁全部就绪，新的 ARMED 命令才可生效。
该重确认屏障只约束 DRIVE，不改变 ARM 子系统语义。若清锁执行前条件发生变化，车端拒绝清锁并让 USB session 重新保持 inhibit。
