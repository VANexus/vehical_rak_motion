# Jetson-F407 UART v2 alignment

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
frame/CRC/COBS plus DRIVE_COMMAND and ARM_COMMAND codecs. They are included by
the top-level firmware CMake target. `tools/uart_v2_codec_test.c` is a host
compiler golden-vector test:

```bash
cc -std=c11 -Wall -Wextra -Werror -Wpedantic -ICore/Inc \
  Core/Src/app_protocol_v2.c tools/uart_v2_codec_test.c \
  -o /tmp/uart_v2_codec_test
/tmp/uart_v2_codec_test
```

The codec is not connected to `app_serial.c`, does not advertise v2 features,
and cannot actuate wheels or servos. The existing UART endpoint remains v1.
Do not advertise v2 capability bits or enable host hardware until the full
endpoint and physical gates below are complete.

## Firmware work still required

1. Replace the v1-only serial dispatch with bounded v2 stream decoding,
   sequence validation, strict feature negotiation and independent drive/arm
   command leases. Invalid input must not refresh either lease.
2. Route four wheel targets to the 25 kHz loops with one common saturation
   factor. Remove host-body-twist inverse kinematics from the live control path.
   Review the current IMU stabilizer: its source currently adjusts body `wz`
   before inverse kinematics and therefore is not yet the required wheel-only
   compensation.
3. Publish encoder position/speed from a consistent sample, maintain encoder
   epoch, validity and acquisition uptime. Do not report command-derived
   velocity as measured odometry.
4. Route six calibrated servo position targets without boot sweep. A seventh
   PWM channel requires a confirmed pin/timer assignment; do not invent one.
   Servo command echo must remain distinct from measured joint state.
5. Implement diagnostic counters, safety commands, physical E-stop latch,
   startup DISARMED state and arm active-hold behavior. Verify power-loss
   mechanics independently.
6. Reconcile CubeMX/board startup behavior and verify Release and Debug with
   STM32CubeCLT. Then compare host/F407 vectors and perform supervised physical
   acceptance with wheels clear and arm mechanically supported.

The Jetson SPI sensor bridge, ros2_control hardware plugin, EKF configuration,
wheel geometry/calibration, six-axis joint calibration and gripper channel are
separate gates. A passing codec test does not close any of them.
