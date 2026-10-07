#ifndef APP_SERIAL_H
#define APP_SERIAL_H

#include <stdint.h>

/* [AI] Type-C 活动链路采用 UART v2；USART1/USART3 均使用 CubeMX DMA 配置。
   物理帧为 COBS + 零定界 + CRC16，v1 机体速度帧不再接受或回退。 */
void serial_init(void);
void serial_poll_tx(void);
void serial_rx_service(void);
void serial_v2_tick(uint32_t now_ms);

#endif /* APP_SERIAL_H */
