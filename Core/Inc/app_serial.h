#ifndef APP_SERIAL_H
#define APP_SERIAL_H

#include <stdint.h>

/* [AI]  Type-C 串口本地测试通道（CH9102，USART1/USART3 @115200）。
   帧格式为独立约定（非 CAN 协议，can_check.py 不校验此通道）：
   8 字节 = 大端 int16 vx/vy/wz（注意与 CAN v3 的小端不同）+ byte6 enable
   + byte7 CRC8(poly 0x07, 初值 0)。上行回读为同帧结构 @100Hz（0x200 布局）。 */

void serial_init(void);                        /* [AI]  引脚/波特率/NVIC（调度器前调用） */
void serial_push_feedback(const uint8_t *d);   /* [AI]  100Hz 回读入队（双路） */
void serial_poll_tx(void);                     /* [AI]  1kHz 续发 */

#endif /* APP_SERIAL_H */