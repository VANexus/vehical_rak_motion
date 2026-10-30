#ifndef APP_CANOPEN_H
#define APP_CANOPEN_H

#include <stdint.h>

/**
 * app_canopen.h —— CANopen v5 适配层对外接口（**唯一**的 CAN 链路实现）
 *
 * 2026-10-30：v4（`app_can.c` + 手写帧表）已**整体删除**，`APP_USE_CANOPEN`
 * 编译开关一并移除 —— 协议只允许 v5 一套，回滚分支不再存在。
 * 下面四个符号是**历史接口名**，保留只为让 `main.c` / `freertos.c` 的调用点
 * 不必改写；语义现在全部由 CANopenNode + `app_canopen_step()` 承担。
 * 新代码请直接用 `app_canopen_step()`，不要再用这些名字描述行为。
 */

/* 硬件 + 协议栈初始化（先于调度器调用）。内部 = canopen_app_init()。 */
void can_init(void);

/* 接收侧：跑 CANopen 后台处理（内部自带 1 ms 让出）。由 Task_CAN_Rx 调用。 */
void can_rx_task_step(void);

/* 1 ms 节拍：驱动 CANopenNode 的 SYNC / RPDO / TPDO 处理。 */
void can_tx_task_step(uint32_t now_ms);

/* CRC8（poly 0x07）。CANopen 自身的 CRC 在栈内；本函数只服务 USB v2 串口通道。 */
uint8_t crc8(const uint8_t *p, uint8_t n, uint8_t init);

/* 100 Hz 调用：把 CANopen RPDO 命令落到 app_car，再把反馈写回 OD。
 * 由 task_control 在 car_control_step() 之前调用。 */
void app_canopen_step(uint32_t now_ms);

#endif /* APP_CANOPEN_H */
