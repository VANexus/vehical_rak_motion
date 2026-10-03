/**
 * [AI]  @file  app_serial.c — Type-C 串口本地测试通道（CH9102 USART1/USART3）
 *
 * 帧格式为独立约定（见 app_serial.h）。非总线协议，can_check.py 不校验。
 * 回读：0x200 同帧经 serial_push_feedback 入队、serial_poll_tx 以 1kHz 续发。
 */
#include "app_serial.h"
#include "app_can.h"    /* [AI] crc8 共享实现 */
#include "app_car.h"    /* [AI] car_can_velocity */
#include "main.h"
#include "stm32f4xx.h"

#define SERIAL_BAUD 115200U
enum { SERIAL_PORTS = 2 };

struct serial_rx {
    uint8_t buf[16];
    uint8_t len;
};

static struct serial_rx ser_rx[SERIAL_PORTS];
static uint8_t ser_txq[SERIAL_PORTS][32];
static volatile uint8_t ser_tx_rd[SERIAL_PORTS], ser_tx_wr[SERIAL_PORTS];

static void serial_parse(struct serial_rx *s, uint8_t b)
{
    if (s->len >= sizeof(s->buf)) return;
    s->buf[s->len++] = b;
    while (s->len >= 8U) {
        if (crc8(s->buf, 7U, 0U) == s->buf[7]) {
            /* [AI]  本地速度帧（大端，同 0x100 字段） */
            car_can_velocity(
                (int32_t)(int16_t)(((uint16_t)s->buf[0] << 8) | s->buf[1]),
                (int32_t)(int16_t)(((uint16_t)s->buf[2] << 8) | s->buf[3]),
                (int32_t)(int16_t)(((uint16_t)s->buf[4] << 8) | s->buf[5]),
                s->buf[6], HAL_GetTick());
            for (uint8_t k = 0; k + 8U < s->len; ++k) s->buf[k] = s->buf[k + 8U];
            s->len = (uint8_t)(s->len - 8U);
        } else {
            /* [AI]  失步：滑 1 字节重同步 */
            for (uint8_t k = 0; k + 1U < s->len; ++k) s->buf[k] = s->buf[k + 1U];
            s->len--;
        }
    }
}

void serial_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;
    GPIOA->MODER = (GPIOA->MODER & ~((3UL << 18) | (3UL << 20))) |
                   (2UL << 18) | (2UL << 20);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0x00000FF0UL) | 0x00000770UL;
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3UL << 20)) | (1UL << 20);
    GPIOD->MODER = (GPIOD->MODER & ~((3UL << 16) | (3UL << 18))) |
                   (2UL << 16) | (2UL << 18);
    GPIOD->AFR[1] = (GPIOD->AFR[1] & ~0x000000FFUL) | 0x00000077UL;
    GPIOD->PUPDR = (GPIOD->PUPDR & ~(3UL << 18)) | (1UL << 18);

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    USART1->BRR = (HAL_RCC_GetPCLK2Freq() + SERIAL_BAUD / 2U) / SERIAL_BAUD;
    USART3->BRR = (HAL_RCC_GetPCLK1Freq() + SERIAL_BAUD / 2U) / SERIAL_BAUD;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_SetPriority(USART1_IRQn, 7U);
    NVIC_SetPriority(USART3_IRQn, 7U);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);
}

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE)) != 0U) {
        uint8_t b = (uint8_t)USART1->DR;
        if ((sr & USART_SR_RXNE) != 0U) serial_parse(&ser_rx[0], b);
    }
}

void USART3_IRQHandler(void)
{
    uint32_t sr = USART3->SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE)) != 0U) {
        uint8_t b = (uint8_t)USART3->DR;
        if ((sr & USART_SR_RXNE) != 0U) serial_parse(&ser_rx[1], b);
    }
}

void serial_push_feedback(const uint8_t *d)
{
    for (unsigned u = 0; u < (unsigned)SERIAL_PORTS; ++u) {
        for (unsigned i = 0; i < 8U; ++i) {
            uint8_t next = (uint8_t)((ser_tx_wr[u] + 1U) % sizeof(ser_txq[0]));
            if (next == ser_tx_rd[u]) break;
            ser_txq[u][ser_tx_wr[u]] = d[i];
            ser_tx_wr[u] = next;
        }
    }
}

void serial_poll_tx(void)
{
    for (unsigned u = 0; u < (unsigned)SERIAL_PORTS; ++u) {
        USART_TypeDef *uart = (u == 0U) ? USART1 : USART3;
        if ((uart->SR & USART_SR_TXE) != 0U && ser_tx_rd[u] != ser_tx_wr[u]) {
            uart->DR = ser_txq[u][ser_tx_rd[u]];
            ser_tx_rd[u] = (uint8_t)((ser_tx_rd[u] + 1U) % sizeof(ser_txq[0]));
        }
    }
}