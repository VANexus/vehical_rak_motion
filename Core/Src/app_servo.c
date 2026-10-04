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
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include "app_servo.h"

#define SERVO_MIN_US   500U     /* 0°  对应脉宽 */
#define SERVO_MAX_US   2500U    /* 180° 对应脉宽 */
#define SERVO_MID_US   1500U    /* 90° 中位 */

/* 把引脚切到定时器复用功能 */
static void gpio_af(GPIO_TypeDef *gpio, unsigned pin, unsigned af)
{
    gpio->MODER  = (gpio->MODER  & ~(3UL << (2U * pin))) | (2UL << (2U * pin));
    gpio->PUPDR  = (gpio->PUPDR  & ~(3UL << (2U * pin))) | (1UL << (2U * pin));
    gpio->OTYPER &= ~(1UL << pin);
    unsigned shift = 4U * (pin % 8U);
    gpio->AFR[pin / 8U] = (gpio->AFR[pin / 8U] & ~(15UL << shift)) | ((uint32_t)af << shift);
}

void servo_init(void)
{
    /* 时钟：TIM8 挂 APB2(定时器时钟 168MHz)，TIM12 挂 APB1(84MHz) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
    (void)RCC->APB2ENR;
    (void)RCC->APB1ENR;

    /* ---- TIM8（PC6~PC9 四个舵机）：168MHz / 168 = 1MHz，ARR=19999 → 20ms ---- */
    TIM8->CR1 = 0;
    TIM8->PSC = 167U;                 /* 168MHz ÷ 168 = 1MHz → 每格 1µs */
    TIM8->ARR = 19999U;               /* 20000 格 × 1µs = 20ms → 50Hz */
    TIM8->CCMR1 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);  /* CH1/CH2 PWM */
    TIM8->CCMR2 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);  /* CH3/CH4 PWM */
    TIM8->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM8->CCR1 = SERVO_MID_US;        /* 上电去中位 90°，防止乱甩 */
    TIM8->CCR2 = SERVO_MID_US;
    TIM8->CCR3 = SERVO_MID_US;
    TIM8->CCR4 = SERVO_MID_US;
    TIM8->BDTR = TIM_BDTR_MOE;        /* 高级定时器必须开"主输出开关" */
    TIM8->EGR  = TIM_EGR_UG;
    TIM8->CR1  = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* ---- TIM12（PB14/PB15 两个舵机）：84MHz / 84 = 1MHz，ARR=19999 → 20ms ---- */
    TIM12->CR1 = 0;
    TIM12->PSC = 83U;                 /* 84MHz ÷ 84 = 1MHz */
    TIM12->ARR = 19999U;
    TIM12->CCMR1 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);
    TIM12->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM12->CCR1 = SERVO_MID_US;
    TIM12->CCR2 = SERVO_MID_US;
    TIM12->EGR  = TIM_EGR_UG;
    TIM12->CR1  = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* ---- 引脚：PC6-9 = TIM8_CH1-4(AF3)，PB14/15 = TIM12_CH1/2(AF9) ---- */
    gpio_af(GPIOC, 6, 3);
    gpio_af(GPIOC, 7, 3);
    gpio_af(GPIOC, 8, 3);
    gpio_af(GPIOC, 9, 3);
    gpio_af(GPIOB, 14, 9);
    gpio_af(GPIOB, 15, 9);
}

void servo_set_pulse(unsigned ch, unsigned us)
{
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    switch (ch) {
    case 0U: TIM8->CCR1  = us; break;   /* PC6  */
    case 1U: TIM8->CCR2  = us; break;   /* PC7  */
    case 2U: TIM8->CCR3  = us; break;   /* PC8  */
    case 3U: TIM8->CCR4  = us; break;   /* PC9  */
    case 4U: TIM12->CCR1 = us; break;   /* PB14 */
    case 5U: TIM12->CCR2 = us; break;   /* PB15 */
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
