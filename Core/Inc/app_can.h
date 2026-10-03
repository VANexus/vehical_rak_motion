#ifndef APP_CAN_H
#define APP_CAN_H

#include <stdint.h>

/* [AI]  CAN 协议 v3（规范: docs/PROTOCOL.md；校验: tools/can_check.py）。
   小端、DLC=8、byte6=counter(高4位)|flags(低4位)、
   byte7=CRC8(poly 0x07, 初值=该帧 Data ID, 覆盖字节0-6)。
   本层实现:
     收  0x100 CHASSIS_VEL_CMD (100Hz, enable=bit0)
     收  0x010 SAFETY_CMD      (1=START 2=STOP 4=急停锁存 5=清除 6=清故障;
          急停走 CAN 中断快速路径 → 25kHz ISR ≤40µs 刹车)
     发  0x200 CHASSIS_VEL_FB  (100Hz, flags=mode/estop/fault)
     发  0x030 XFER_CH_TO_ARM  (1kHz, 互锁)
     发  0x230 NODE_INFO_CHASSIS (上电 1 次 + 1Hz)
   发送经 3 个 bxCAN 邮箱分派(0x030/0x200/0x230)；串口本地通道见 app_serial.h。 */

void can_init(void);                     /* [AI]  硬件 + 队列 + NVIC（先于调度器调用） */
void can_rx_task_step(void);             /* [AI]  阻塞于队列；Task_CAN_Rx 调用 */
void can_tx_task_step(uint32_t now_ms);  /* [AI]  1kHz；Task_CAN_Tx 调用 */

uint8_t crc8(const uint8_t *p, uint8_t n, uint8_t init);   /* [AI]  协议 CRC8（串口通道复用） */

#endif /* APP_CAN_H */
