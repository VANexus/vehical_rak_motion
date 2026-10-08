/*
 * app_servo.h — 6 路舵机驱动
 *
 * 【什么是"驱动"】
 *   舵机听不懂"转到90度"这种话，它只认一种"方言"：
 *   每 20 毫秒（50Hz）给它一个脉冲，脉冲宽度决定角度——
 *     0.5 毫秒脉宽 → 转到 0°
 *     1.5 毫秒脉宽 → 转到 90°（中间）
 *     2.5 毫秒脉宽 → 转到 180°
 *   "驱动"就是那个翻译官：你写 servo_set_angle(3, 45)，
 *   它负责把 45° 翻译成 1.0ms 的脉冲、并让硬件定时器准时发出去。
 *
 * 【接哪个口】板上 6 个舵机接口（信号线接这里，电源线接外部电源！）
 *   ch0=PC6  ch1=PC7  ch2=PC8  ch3=PC9（TIM8 四个通道）
 *   ch4=PB14 ch5=PB15（TIM12 两个通道）
 */
#ifndef APP_SERVO_H
#define APP_SERVO_H

/* 初始化 6 路舵机信号（50Hz），上电所有舵机自动去中位 90° */
void servo_init(void);

/* 直接设脉宽 µs（500~2500），ch = 0~5（见上面接口表） */
void servo_set_pulse(unsigned ch, unsigned us);

/* [AI] R45: 读回当前脉宽 µs（CAN v4 SERVO_STATE 遥测用），ch = 0~5 */
unsigned servo_get_pulse(unsigned ch);

/* 设定角度 0~180°，ch = 0~5；越界会自动夹到 0~180 */
void servo_set_angle(unsigned ch, int deg);

/* 自检：让 1 个舵机从 0° 慢慢扫到 180° 再扫回来（约 5 秒），
   方便你确认"哪个口接的是哪个舵机、方向对不对"。
   ⚠️ 只在舵机没装在机械臂上时用！ */
void servo_sweep_test(unsigned ch);

#endif /* APP_SERVO_H */
