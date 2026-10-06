/*
 * app_servo.c — 6 路舵机驱动（TIM8 + TIM12，标准 50Hz 舵机信号）
 *
 * 【原理大白话】
 *   舵机每 20ms 要收到一个脉冲，脉宽 0.5~2.5ms 代表 0~180°。
 *   用硬件定时器实现：把定时器调成"每 1µs 数一格、数到 20000 归零"
 *   = 20ms 周期；比较值(CCR) = 500 就是 0.5ms 脉冲，1500 就是 1.5ms。
 *   写好 CCR 后硬件自己循环发，CPU 完全不用管——比电机那套轻松一万倍。
 *
 * 【为什么不用 FreeRTOS 管】
 *   纯硬件定时器输出，中断都不用，主循环随便干别的。
 */
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include "app_servo.h"

#define SERVO_MIN_US   500U     /* 0°  对应脉宽 */
#define SERVO_MAX_US   2500U    /* 180° 对应脉宽 */
#define SERVO_MID_US   1500U    /* 90° 中位 */

void servo_init(void)
{
    /* [AI] R20 CubeMX 迁移：TIM8/TIM12 由 MX_TIM8/12_Init 配置
       （PSC=167/83 → 1MHz tick、ARR=19999 → 50Hz、各通道 Pulse=1500 中位、
       引脚 AF 已配），本层只重设中位（双保险）+ 启动 PWM 输出。
       高级定时器 TIM8 的 MOE 由 HAL_TIM_PWM_Start 置位。 */
    static const uint32_t ch8[4] = {
        TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
    for (unsigned k = 0; k < 4U; ++k) {
        __HAL_TIM_SET_COMPARE(&htim8, ch8[k], SERVO_MID_US); /* PC6-9，防乱甩 */
        (void)HAL_TIM_PWM_Start(&htim8, ch8[k]);
    }
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, SERVO_MID_US);  /* PB14 */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, SERVO_MID_US);  /* PB15 */
    (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void servo_set_pulse(unsigned ch, unsigned us)
{
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    switch (ch) {
    case 0U: __HAL_TIM_SET_COMPARE(&htim8,  TIM_CHANNEL_1, us); break;  /* PC6  */
    case 1U: __HAL_TIM_SET_COMPARE(&htim8,  TIM_CHANNEL_2, us); break;  /* PC7  */
    case 2U: __HAL_TIM_SET_COMPARE(&htim8,  TIM_CHANNEL_3, us); break;  /* PC8  */
    case 3U: __HAL_TIM_SET_COMPARE(&htim8,  TIM_CHANNEL_4, us); break;  /* PC9  */
    case 4U: __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, us); break;  /* PB14 */
    case 5U: __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us); break;  /* PB15 */
    default: break;
    }
}

void servo_set_angle(unsigned ch, int deg)
{
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    /* 0° → 500µs，180° → 2500µs，线性换算 */
    unsigned us = SERVO_MIN_US + (unsigned)deg * (SERVO_MAX_US - SERVO_MIN_US) / 180U;
    servo_set_pulse(ch, us);
}

void servo_sweep_test(unsigned ch)
{
    /* 慢慢扫，别猛甩：每 5° 停 30ms */
    for (int deg = 0; deg <= 180; deg += 5) {
        servo_set_angle(ch, deg);
        osDelay(30);
    }
    osDelay(300);
    for (int deg = 180; deg >= 0; deg -= 5) {
        servo_set_angle(ch, deg);
        osDelay(30);
    }
    osDelay(300);
}
