#ifndef APP_CAR_H
#define APP_CAR_H

#include <stdint.h>

/* Four-wheel chassis application layer.
   - 25 kHz closed loop: TIM6 ISR does encoder accounting, target slew and
     the speed PI; all four motors chase the same target speed.
   - 100 Hz supervisor (Task_Control): bench run test, Jetson velocity
     commands (mecanum inverse kinematics), deadman, estop, telemetry.
   - Task_Status feeds the watchdog; telemetry goes out over CAN (0x200). */

extern const char build_tag[];   /* [AI]  版本标签，单点定义在 app_car.c */

void car_init(void);
void car_start_control_isr(void);      /* [AI]  R3: 25kHz ISR 启动点,Task_Control 首拍调用 */
void car_control_step(uint32_t now);   /* 100 Hz, Task_Control */
void car_feed_watchdog(void);          /* Task_Status */

uint32_t car_battery_mv(void);
uint8_t car_enable_switch(void);
uint8_t car_mode(void);

/* CAN command inputs (Task_CAN_Rx) */
void car_can_velocity(int32_t vx_mmps, int32_t vy_mmps, int32_t wz_crads,
                      uint8_t enable, uint32_t now);
void car_can_control(uint8_t cmd, uint32_t now);   /* [AI]  v3 SAFETY_CMD 码 */
void car_estop_request_fast(void);                 /* [AI]  CAN 中断急停快速路径 */

/* Telemetry outputs (Task_CAN_Tx) */
void car_get_velocity(int32_t *vx_mmps, int32_t *vy_mmps, int32_t *wz_crads);
uint8_t car_can_status(void);

#endif /* APP_CAR_H */