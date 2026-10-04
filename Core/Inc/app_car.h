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

/* [AI]  R13 调试显示数据出口（app_oled 用） */
uint8_t car_reset_cause(void);   /* 'B'/'I'/'S'/'P'/'-' */
uint8_t car_mt_index(void);      /* 自检序列当前步 */
uint8_t car_active_motor(void);  /* FAULT 时的电机号 */
uint8_t car_flags(void);         /* bit0=遥控中 bit1=急停锁存 bit2=自检中 */
void car_wheel_speeds(int32_t *out);  /* counts/ms,四轮快照 */
uint8_t car_control_ready(void);      /* [AI]  R16: 1=25kHz 已启动(BOOTING 结束) */

/* [AI]  R13 自稳定诊断量（定义在 app_car.c,SWD/OLED 直读） */
extern volatile int32_t  dbg_stab_corr;    /* 航向/速率修正(0.01rad/s,已限幅) */
extern volatile uint16_t dbg_stab_scale;   /* 平移降额比例(256=不降) */
extern volatile uint8_t  dbg_slip;         /* 打滑判定标志 */

#endif /* APP_CAR_H */