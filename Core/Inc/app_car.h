#ifndef APP_CAR_H
#define APP_CAR_H

#include <stdint.h>
#include "app_wheel_control.h"

#define CAR_BATTERY_UV_THRESHOLD_MV 10500U

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

/* [AI] R20 CubeMX 迁移: 25kHz 闭环真身(原 TIM6_DAC_IRQHandler)。
   由 stm32f4xx_it.c 的 TIM6_DAC_IRQHandler 在 USER CODE 块内调用;
   优先级 2 高于 RTOS 临界区阈值,内部禁用 FreeRTOS API/浮点/阻塞。 */
void car_control_isr(void);

uint32_t car_battery_mv(void);
uint8_t car_enable_switch(void);
uint8_t car_mode(void);
uint8_t car_estop_clear_allowed(void);

/* [AI] R22: USB v2 轮级直通(前左/前右/后左/后右 urad/s),绕过 body IK;
   内部换算 counts/ms 并钳位。enable=0 即零目标+松链接(deadman)。 */
#define CAR_MOTOR_COUNT WHEEL_COUNT
uint8_t car_wheel_calibration_ready(void);
void car_wheel_command_urad(const int32_t wheel_urad_s[CAR_MOTOR_COUNT],
                            uint8_t enable, uint32_t now);
uint8_t car_estop_clear(void);          /* [AI] 仅清锁成功返回非零；由 USB v2 DRIVE clear 调用 */
uint8_t car_estop_clear_can(void);      /* [AI] R52: CAN 清急停（跳过 PD3 门，其余同按钮路径） */
extern volatile uint8_t car_drive_gate_dbg; /* [AI] R58: 驱动门自诊断位图（DIAG STATE bit6-9） */
/* [AI] R42: USB v2 ARM 命令落地(7 轴位置)。
   轴序单点定义: axis[0..6] = J1(腰,PB15) J2(大臂,PB14) J3(肘,PC9) J4(小臂,PC8)
   J5(腕俯仰,PC7) J6(腕圆周,PC6) 夹爪(暂无通道)。
   位置语义: 舵机输出轴角,0~270° 满量程 ↔ 500~2500µs(实测 MG995/996R 口径);
   J2 机械限位 0~135° 由本体挡块兜底。J1/J6 实测为连续旋转舵机——
   该两轴 position_urad 解释为"速度偏置"(中位=停转),由上位机 TUI/标定层约定。
   valid_mask 第 i 位=1 才更新该轴,否则该轴保持 1500µs 中位。 */
void car_arm_command_urad(const int32_t position_urad[7], uint8_t valid_mask);
uint64_t car_uptime_us(uint32_t tick_ms);
/* [AI] R22: 编码器一致采样快照(DRIVE_STATE 组包用):归一化位置 urad、
   速度 urad/s、valid mask(encoder_sign 已标定)、epoch、采样时刻 µs。 */
void car_encoder_snapshot(int64_t pos_urad[CAR_MOTOR_COUNT],
                          int32_t vel_urad_s[CAR_MOTOR_COUNT],
                          uint8_t *valid_mask, uint32_t *epoch,
                          uint64_t *sample_us);
/* [AI] CAN 急停与已校验的 USB v2 DRIVE LATCH 共用此 25 kHz 快速锁存请求。 */
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
void car_wheel_positions(int32_t *out); /* [AI] R45: 累计位置 counts,四轮(CAN WHEEL_POS) */
uint8_t car_control_ready(void);      /* [AI]  R16: 1=25kHz 已启动(BOOTING 结束) */

/* [AI]  R13 自稳定诊断量（定义在 app_car.c,SWD/OLED 直读） */
extern volatile int32_t  dbg_stab_corr;    /* 航向/速率修正(0.01rad/s,已限幅) */
extern volatile uint16_t dbg_stab_scale;   /* 平移降额比例(256=不降) */
extern volatile uint8_t  dbg_slip;         /* 打滑判定标志 */

#endif /* APP_CAR_H */
