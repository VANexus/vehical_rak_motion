#ifndef APP_SERIAL_H
#define APP_SERIAL_H

#include <stdint.h>

/* [AI]  Type-C 串口本地测试通道（CH9102，USART1/USART3 @921600）。
   上位机与 A/B 板的"CAN 协议通信"走此 USB 串口（R12 定案）：
   8 字节 = 大端 int16 vx/vy/wz（注意与 CAN v3 的小端不同）+ byte6 enable
   + byte7 CRC8(poly 0x07, 初值 0)。上行回读为 0x200 布局(小端) @100Hz。
   R15: 上行流每秒混入一行 ASCII 识别信号（CRC 帧定界自动跳过），
   上位机靠它分辨串口对面是哪块板；换板编译时加 BOARD_ID="B"（字符串）。 */
#ifndef BOARD_ID
#define BOARD_ID "A"
#endif
#define SERIAL_BANNER "ID:" BOARD_ID "\n"

void serial_init(void);                        /* [AI]  引脚/波特率/NVIC（调度器前调用） */
void serial_push_feedback(const uint8_t *d);   /* [AI]  100Hz 回读入队（双路） */
void serial_push_text(const char *s);          /* [AI]  R15: 识别横幅入队（双路） */
void serial_poll_tx(void);                     /* [AI]  1kHz 续发 */

#endif /* APP_SERIAL_H */