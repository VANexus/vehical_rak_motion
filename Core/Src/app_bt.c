/*
 * app_bt.c — [AI] R67 底盘板串口蓝牙遥测（USART2 @9600，只发不收）
 *
 * 为什么是"只发"：见 app_bt.h 的定位说明（脚本设不了 NVIC → RX 等 GUI 勾中断）。
 *
 * 行格式（ASCII，手机蓝牙串口 App 直接看）：
 *   状态 1 Hz：  <TAG> B<bat>mV M<mode> Y<yaw> Z<zeroed> E<estop>\r\n
 *   事件      ：  !<MSG>\r\n            （ESTOP / FAULT / IMPACT …）
 *
 * 阻塞预算：最长 ~42 字符 ≈ 42 ms @9600；只在 10 Hz 任务里、1 Hz 触发一次。
 */
#include "app_bt.h"

#include "app_car.h"
#include "app_imu.h"
#include "main.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define BT_LINE_MAX   72U
#define BT_DASH_MS    10000U   /* [AI] R69: 每 10 s 一帧 ASCII 仪表盘（"路况记录仪"） */
#define BT_EVT_MAX    24U
#define BT_PERIOD_MS  1000U

static volatile uint8_t s_evt_pending;
static char     s_evt_msg[BT_EVT_MAX];
static uint32_t s_last_ms;
static uint32_t s_last_dash_ms;
static uint16_t s_bumps;          /* [AI] R69: 冲击事件累计（IMU impact） */
static uint8_t  s_started;

static void bt_send(const char *s, uint16_t len)
{
    if (len == 0U) return;
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)s, len, 100U);
}

void bt_init(void)
{
    char line[BT_LINE_MAX];
    int n = snprintf(line, sizeof(line), "%s BT READY\r\n", build_tag);
    if (n > 0) bt_send(line, (uint16_t)n);
    s_started = 1U;
    s_last_ms = HAL_GetTick();
    s_last_dash_ms = s_last_ms;
}

void bt_event(const char *msg)
{
    if (msg == NULL) return;
    /* ISR 上下文：只做定长拷贝 + 置标志，不碰 UART */
    unsigned i = 0U;
    for (; i + 1U < BT_EVT_MAX && msg[i] != '\0'; ++i) s_evt_msg[i] = msg[i];
    s_evt_msg[i] = '\0';
    s_evt_pending = 1U;
}

void bt_step(uint32_t now_ms)
{
    if (s_started == 0U) { bt_init(); return; }   /* 惰性初始化：首拍在任务上下文打印横幅 */

    /* 事件优先：先把事件行发掉（更短、更重要） */
    if (s_evt_pending != 0U) {
        char evt[BT_EVT_MAX + 4U];
        s_evt_pending = 0U;
        int n = snprintf(evt, sizeof(evt), "!%s\r\n", s_evt_msg);
        if (n > 0) bt_send(evt, (uint16_t)n);
        s_last_ms = now_ms;
        return;                     /* 事件行占用本拍，状态行下一拍再发 */
    }

    if ((uint32_t)(now_ms - s_last_ms) < BT_PERIOD_MS) return;
    s_last_ms = now_ms;

    char line[BT_LINE_MAX];

    /* [AI] R69: 顺带把 IMU 冲击事件记成"颠簸计数"——蓝牙当路况记录仪用 */
    if (imu_take_impact() != 0U && s_bumps < 0xFFFFU) s_bumps++;

    if ((uint32_t)(now_ms - s_last_dash_ms) >= BT_DASH_MS) {
        /* 10 s 一帧：偏航角速度条形图 + 电压条 + 颠簸数 + 模式
           |yaw| 上限 ±60°/s，一格雷 12°/s；电压 9.0–12.6 V 映射 8 格 */
        s_last_dash_ms = now_ms;
        char bar[9];
        int yaw = (int)(imu_yaw_rate_dps());
        int ay = yaw < 0 ? -yaw : yaw;
        if (ay > 96) ay = 96;
        unsigned n = (unsigned)(ay / 12);
        for (unsigned i = 0; i < 8U; ++i) bar[i] = (i < n) ? '#' : '-';
        bar[8] = '\0';

        unsigned mv = car_battery_mv();
        unsigned lvl = (mv > 9000U) ? ((mv - 9000U) / 450U) : 0U;
        if (lvl > 8U) lvl = 8U;
        char vbar[9];
        for (unsigned i = 0; i < 8U; ++i) vbar[i] = (i < lvl) ? '|' : '.';
        vbar[8] = '\0';

        int m = snprintf(line, sizeof(line), "DASH Y%c[%s] V%lu[%s] BUMPS%u M%u\r\n",
                         (yaw < 0) ? '-' : '+', bar,
                         (unsigned long)mv, vbar,
                         (unsigned)s_bumps, (unsigned)car_mode());
        if (m > 0) bt_send(line, (uint16_t)m);
        return;
    }

    /* 1 Hz 状态行 */
    int yaw = (int)(imu_yaw_rate_dps());     /* 整数 °/s，够看趋势 */
    int n = snprintf(line, sizeof(line), "%s B%lumV M%u Y%+d E%u\r\n",
                     build_tag,
                     (unsigned long)car_battery_mv(),
                     (unsigned)car_mode(),
                     yaw,
                     (unsigned)(car_mode() == 2U ? 1U : 0U));
    if (n > 0) bt_send(line, (uint16_t)n);
}
