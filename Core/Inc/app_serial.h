#ifndef APP_SERIAL_H
#define APP_SERIAL_H

#include <stdint.h>

/* [AI]  Type-C 串口本地测试通道（CH9102，USART1/USART3 @921600）。
   上位机与 A/B 板的"CAN 协议通信"走此 USB 串口（R12 定案）：
   8 字节 = 大端 int16 vx/vy/wz（注意与 CAN v3 的小端不同）+ byte6 enable
   + byte7 CRC8(poly 0x07, 初值 0)。上行回读为 0x200 布局(小端) @100Hz。 */

void serial_init(void);                        /* [AI]  引脚/波特率/NVIC（调度器前调用） */
void serial_push_feedback(const uint8_t *d);   /* [AI]  100Hz 回读入队（双路） */
void serial_poll_tx(void);                     /* [AI]  1kHz 续发 */

#endif /* APP_SERIAL_H */