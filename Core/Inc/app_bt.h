#ifndef APP_BT_H
#define APP_BT_H

#include <stdint.h>

/*
 * app_bt.h — [AI] R67 底盘板串口蓝牙（USART2，PD5=TX / PD6=RX，9600 8N1）
 *
 * 硬件出处：C30D 资源分配说明 PDF —— `D5.D6 → USART2 → 串口蓝牙 → 默认用于 APP 通信`；
 * 原理图里对应「蓝牙模块接口」。模块型号 BT04-A，**实测 9600**（R20 前记录）。
 *
 * 定位（为什么不给蓝牙控制权）：
 *   - 蓝牙是**只发不收**的遥测通道：1 Hz 一行 ASCII 状态 + 事件行（急停/故障）。
 *   - 原因：CubeMX 的 -q 脚本设不了 NVIC（CUBEMX-SOP §4 实测），USART2 的接收中断
 *     没法由脚本打开；而 app 层直接写 USART2->CR1 是"手写外设配置"，违反禁令。
 *     所以 RX 等 GUI 勾一次 `USART2 global interrupt` 后再加（预留 bt_poll 接口）。
 *   - 控制权仍在 CAN（Jetson）与 PE0；蓝牙**不能**改目标速度/清急停。
 *
 * 速率预算：9600 baud ≈ 1 ms/字符；1 Hz 一行 ≤48 字符 ≈ 48 ms 阻塞，
 *   放在 10 Hz 状态任务里（周期 100 ms），可接受；事件行走同一通道、行更短。
 */

/* 上电打印一行（调度器起来后调） */
void bt_init(void);

/* 10 Hz 调用；内部按 1 Hz 发状态行 */
void bt_step(uint32_t now_ms);

/* 事件行（≤23 字符）。可在 ISR 里调用：只做 memcpy + 置标志，由 bt_step 落地发送 */
void bt_event(const char *msg);

#endif /* APP_BT_H */
