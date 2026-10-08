#ifndef APP_CAN_H
#define APP_CAN_H

#include <stdint.h>

/* [AI] R45: CAN v4 主链路（docs/PROTOCOL.md v4）。CAN 承载全部控制与遥测：
   下行 0x100 DRIVE_CMD(4 轮速)/0x110+111 ARM_CMD(7 轴)/0x010 SAFETY
   （急停 ISR 快速路径 ≤40µs + START/STOP 远程门）；上行轮位/轮速/舵机/DIAG。
   远程门：SAFETY START 开、STOP/急停关；DRIVE/ARM 断流 100ms deadman。
   USB v2 串口保留为备用链路（app_serial，帧格式冻结不变）。 */

void can_init(void);                     /* [AI]  硬件 + 队列 + NVIC（先于调度器调用） */
void can_rx_task_step(void);             /* [AI]  阻塞于队列；Task_CAN_Rx 调用 */
void can_tx_task_step(uint32_t now_ms);  /* [AI]  1kHz；Task_CAN_Tx 调用 */

uint8_t crc8(const uint8_t *p, uint8_t n, uint8_t init);   /* [AI]  协议 CRC8（串口通道复用） */

/* [AI]  R16: UART v1 故障位数据出口——v4 起互锁帧作废恒回 0，
   仅为冻结的 UART v1 备用路径可编译而保留 */
uint8_t can_arm_seen(void);        /* 恒 0（v4 起） */
uint32_t can_arm_last_ms(void);    /* 恒 0（v4 起） */
int16_t can_arm_fault_code(void);  /* 恒 0（v4 起） */

#endif /* APP_CAN_H */
