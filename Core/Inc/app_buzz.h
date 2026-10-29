#ifndef APP_BUZZ_H
#define APP_BUZZ_H

#include <stdint.h>

/*
 * app_buzz.h — [AI] A8 板载蜂鸣器（PA8，需插跳线帽；高电平响，有效电平见 .c）
 *
 * 为什么做：C30D 的蜂鸣器从 R20 起一直没被用过。台架上电机一响、串口一屏字，
 * 人眼很难同时盯住"撞了没有/故障了没有"。蜂鸣器给的是**耳朵能听的状态码**：
 *
 *   BOOT   1 短          —— 上电自检通过（= 固件跑起来了、任务都建住了）
 *   JOG    2 极短        —— 点动开始（提醒"手离远点"）
 *   IMPACT 3 急促        —— 冲击/碰撞确认（与 app_joint 的 collide 联动）
 *   FAULT  1 长 1 短     —— 位置环故障/急停锁存
 *
 * 设计约束：
 *   - 非阻塞：状态机按时间推进，`buzz_tick()` 由 1 kHz 任务调用，不在里面 delay；
 *   - 优先级：FAULT > IMPACT > JOG > BOOT（新事件覆盖旧的，避免排队积压）；
 *   - 关断安全：上电先拉低（不响），`buzz_init()` 由 init 阶段调用。
 */

typedef enum {
    BUZZ_NONE = 0,
    BUZZ_BOOT,
    BUZZ_JOG,
    BUZZ_IMPACT,
    BUZZ_FAULT
} buzz_event_t;

/* 上电初始化（输出低 = 不响） */
void buzz_init(void);

/* 触发一个提示音（可在任务上下文调用；可在 CAN ISR 里调，见 .c 注释） */
void buzz_alert(buzz_event_t ev);

/* 1 kHz 推进（由 task_can_tx 调用即可） */
void buzz_tick(uint32_t now_ms);

/* 当前是否在响（调试/OLED 用） */
uint8_t buzz_active(void);

#endif /* APP_BUZZ_H */
