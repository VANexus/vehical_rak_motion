#ifndef APP_CAN_H
#define APP_CAN_H

#include <stdint.h>

/* [AI] CAN v3 仅保留遥测/互锁兼容；USB v2 是唯一远程运动目标和使能入口。
   0x100、START、STOP、CLEAR 均不授权控制；0x010 EMERGENCY_STOP 只可单向停机，
   走 CAN 中断快速路径 → 25kHz ISR ≤40µs 刹车。串口本地通道见 app_serial.h。 */

void can_init(void);                     /* [AI]  硬件 + 队列 + NVIC（先于调度器调用） */
void can_rx_task_step(void);             /* [AI]  阻塞于队列；Task_CAN_Rx 调用 */
void can_tx_task_step(uint32_t now_ms);  /* [AI]  1kHz；Task_CAN_Tx 调用 */

uint8_t crc8(const uint8_t *p, uint8_t n, uint8_t init);   /* [AI]  协议 CRC8（串口通道复用） */

/* [AI]  R16: UART v1 故障位数据出口(对端=0x031 机械臂,app_serial 用) */
uint8_t can_arm_seen(void);        /* 1=自启动以来收到过 0x031 */
uint32_t can_arm_last_ms(void);    /* 最近一帧 0x031 的 tick */
int16_t can_arm_fault_code(void);  /* 对端报告的故障码 */

#endif /* APP_CAN_H */
