#ifndef APP_SERIAL_H
#define APP_SERIAL_H

#include <stdint.h>

/* [AI]  Type-C 串口通道 —— 团队 UART v1 协议（COBS + CRC16/CCITT-FALSE）
   规范: Jetson 仓 docs/protocol/jetson_f407_uart_v1.md；固件侧对齐响应见
   本仓 docs/UART-v1-ALIGN.md。物理层: USART1/USART3 双 Type-C 同款,
   921600 8N1,指令/遥测各 100Hz。
   帧: wire = COBS(帧)||0x00;帧 = ver u8=1 + type u8 + seq u16LE +
       plen u16LE(≤128) + payload + crc16 u16LE(覆盖 ver..payload)。
   消息: 0x01 HELLO / 0x02 HELLO_ACK / 0x10 SET_BASE_VELOCITY(15B) /
         0x11 BASE_STATE(23B) / 0x12 FAULT_EVENT(6B)。
   安全: 上电 DISARMED;HELLO(附带 DISARM)后才可 ARM(mode=1,且无故障位);
         lease_ms∈[10,50] 超时自动 DISARM+锁存 bit1,HELLO 清锁存;
         序号 (cand-prev)mod65536 ∈ 1..32767,HELLO 重置;坏帧不刷新 lease。
   v1.1 原子动作(feature bit3)暂不实现,host 默认自动回退 v1。
   R15 的 'ID:A' 识别横幅保留(1Hz,host 用作设备身份确认,不参与协议)。 */

void serial_init(void);            /* [AI]  调度器前调用:引脚/波特率/NVIC/boot_id */
void serial_poll_tx(void);         /* [AI]  1kHz(Task_CAN_Tx):泵首字节并开 TXE 中断续发 */
void serial_v1_tick(uint32_t now); /* [AI]  1kHz:lease 看门狗 + 100Hz BASE_STATE + 故障事件 */
void serial_push_text(const char *s); /* [AI]  识别横幅等原始文本入双路 TX 队列 */

/* [AI]  R15/R16: 设备识别横幅(@1Hz 混入上行流,CRC 帧定界自动跳过);
   换板编译时加 BOARD_ID="B" */
#ifndef BOARD_ID
#define BOARD_ID "A"
#endif
#define SERIAL_BANNER "ID:" BOARD_ID "\n"

#endif /* APP_SERIAL_H */
