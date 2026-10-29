/*
 * app_buzz.c — [AI] A8 板载蜂鸣器（PA8，非阻塞状态机）
 *
 * 硬件：C30D 蜂鸣器接 PA8，板上有跳线帽；CubeMX 已把 PA8 配成推挽输出。
 * 驱动方式：直接置位/清零 ODR（不占定时器、不出 PWM —— 音调靠"有无"就够用）。
 *
 * 节奏表（每一步 = 开/关时长 ms；0 = 序列结束）：
 *   BOOT    [60/0]                       上电自检
 *   JOG     [25,40,25,0]                 点动开始
 *   IMPACT  [40,40,40,40,40,0]           冲击/碰撞确认
 *   FAULT   [500,150,120,0]              故障/急停
 *
 * 线程安全：`s_cur`/`s_step`/`s_next_ms` 只被 buzz_tick 写；
 *   buzz_alert 只写 `s_req`（单字），ISR 里调用也安全（无阻塞、无队列）。
 */
#include "app_buzz.h"

#include "main.h"

#define BUZZ_PORT   GPIOA
#define BUZZ_PIN    GPIO_PIN_8

typedef struct {
    uint16_t ms[8];
} buzz_seq_t;

static const buzz_seq_t k_seq[BUZZ_FAULT + 1] = {
    [BUZZ_NONE]   = {{0U}},
    [BUZZ_BOOT]   = {{180U, 140U, 180U, 0U}},   /* [AI] 上电两声长响：难以漏听 */
    [BUZZ_JOG]    = {{25U, 40U, 25U, 0U}},
    [BUZZ_IMPACT] = {{40U, 40U, 40U, 40U, 40U, 0U}},
    [BUZZ_FAULT]  = {{500U, 150U, 120U, 0U}}
};

static uint8_t  s_cur;
static volatile uint8_t s_req;
static uint8_t  s_step;
static uint8_t  s_on;
static uint32_t s_next_ms;

static void buzz_write(uint8_t on)
{
    HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void buzz_init(void)
{
    s_cur = BUZZ_NONE;
    s_req = BUZZ_NONE;
    s_step = 0U;
    s_on = 0U;
    s_next_ms = 0U;
    buzz_write(0U);              /* 上电不响 */
}

void buzz_alert(buzz_event_t ev)
{
    if (ev == BUZZ_NONE) return;
    /* 抢占规则：只有更重要的事件能覆盖正在播的（避免 IMPACT 被 JOG 打断） */
    if (ev < s_cur) return;
    s_req = (uint8_t)ev;
}

uint8_t buzz_active(void) { return s_on; }

void buzz_tick(uint32_t now_ms)
{
    uint8_t req = s_req;
    if (req != BUZZ_NONE) {
        s_req = BUZZ_NONE;
        s_cur = req;                 /* 重新开始播 */
        s_step = 0U;
        s_on = 1U;
        buzz_write(1U);
        s_next_ms = now_ms + k_seq[s_cur].ms[0];
        return;
    }

    if (s_cur == BUZZ_NONE) return;
    if ((int32_t)(now_ms - s_next_ms) < 0) return;

    /* 到点：开/关交替，序列走完就停 */
    const buzz_seq_t *q = &k_seq[s_cur];
    uint8_t next_step = (uint8_t)(s_step + 1U);
    uint16_t dur = q->ms[next_step];
    if (dur == 0U) {
        s_on = 0U;
        buzz_write(0U);
        s_cur = BUZZ_NONE;
        s_step = 0U;
        return;
    }
    s_step = next_step;
    s_on = (uint8_t)((s_step & 1U) ? 0U : 1U);   /* 奇数步 = 关闭段 */
    buzz_write(s_on);
    s_next_ms = now_ms + dur;
}
