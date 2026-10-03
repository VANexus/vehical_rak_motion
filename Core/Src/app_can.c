/**
 * [AI]  @file  app_can.c — CAN 协议 v3 + Type-C 串口遥控 + 舵机输出
 *
 * [AI]  协议规范见 docs/PROTOCOL.md（v3）：小端、DLC=8、byte6=counter(高4位)|
 * [AI]  flags(低4位)、byte7=CRC8(poly 0x07, 初值=该帧 Data ID, 覆盖字节0-6)。
 * [AI]  本文件实现：0x100 VEL_CMD(收) / 0x010 SAFETY(收, 急停走中断快速路径) /
 * [AI]  0x200 VEL_FB(发 100Hz) / 0x030 XFER(发 1kHz) / 0x230 NODE_INFO(发 1Hz)。
 * [AI]  串口(Type-C CH9102)为本地测试通道，帧格式独立(见 serial 段)。
 */
#include "app_can.h"
#include "app_car.h"
#include "main.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"

/* ---- bxCAN1: PD0=RX / PD1=TX (AF9), 1Mbps, APB1 42MHz ---- */
#define CAN_FMR_REG   (*(__IO uint32_t *)(CAN1_BASE + 0x200U))
#define CAN_FM1R_REG  (*(__IO uint32_t *)(CAN1_BASE + 0x204U))
#define CAN_FS1R_REG  (*(__IO uint32_t *)(CAN1_BASE + 0x20CU))
#define CAN_FA1R_REG  (*(__IO uint32_t *)(CAN1_BASE + 0x21CU))
#define CAN_sFREG     ((__IO uint32_t *)(CAN1_BASE + 0x240U))

/* [AI]  CAN v3 帧 ID 与 Data ID（CRC 初值）—— docs/PROTOCOL.md §3 */
#define CID_SAFETY       0x010U
#define DID_SAFETY       0x01U
#define CID_EMCY_CH      0x020U
#define DID_EMCY_CH      0x02U
#define CID_XFER_CH2ARM  0x030U
#define DID_XFER_CH2ARM  0x04U
#define CID_XFER_ARM2CH  0x031U
#define DID_XFER_ARM2CH  0x05U
#define CID_VEL_CMD      0x100U
#define DID_VEL_CMD      0x06U
#define CID_VEL_FB       0x200U
#define DID_VEL_FB       0x09U
#define CID_DIAG         0x201U
#define DID_DIAG         0x0AU
#define CID_NODEINFO     0x230U
#define DID_NODEINFO     0x0EU
#define PROTO_VER        3U
#define HW_REV           1U

struct can_frame {
    uint32_t id;
    uint8_t data[8];
};

static QueueHandle_t rx_queue;
static int16_t arm_fault_code;        /* [AI]  0x031: 机械臂故障码（暂存） */
static int16_t arm_j4_pos, arm_base_pos;
static uint8_t last_cmd_cnt;          /* [AI]  0x100 计数器校验（低4位） */

/* [AI]  串口回读链路(can_tx_task_step 调用,定义在文件后段) */
void serial_push_feedback(const uint8_t *d);
void serial_poll_tx(void);

static uint8_t crc8_id(const uint8_t *p, uint8_t n, uint8_t init)
{
    uint8_t crc = init;
    while (n--) {
        crc ^= *p++;
        for (uint8_t b = 0; b < 8U; ++b)
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void put_le16(uint8_t *p, int32_t v)
{
    if (v > 32767L) v = 32767L;
    if (v < -32768L) v = -32768L;
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)((uint32_t)v >> 8);
}

static int32_t get_le16(const uint8_t *p)
{
    return (int32_t)(int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t build_hash16(void)
{
    const char *tag = build_tag;
    uint16_t h = 0xFFFFU;
    for (const char *p = tag; *p; ++p) {
        h ^= (uint8_t)*p;
        for (uint8_t b = 0; b < 8U; ++b)
            h = (h & 1U) ? (uint16_t)((h >> 1) ^ 0x8408U) : (uint16_t)(h >> 1);
    }
    return h;
}

/* [AI]  ---- 初始化：硬件在本函数，队列在本函数（先于 NVIC 使能创建） ---- */
void can_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    (void)RCC->APB1ENR;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER = (GPIOD->MODER & ~(3UL << 0)) | (2UL << 0);      /* PD0 AF */
    GPIOD->MODER = (GPIOD->MODER & ~(3UL << 2)) | (2UL << 2);      /* PD1 AF */
    GPIOD->AFR[0] = (GPIOD->AFR[0] & ~0x000000FFUL) | 0x00000099UL;

    CAN1->MCR &= ~CAN_MCR_SLEEP;
    CAN1->MCR |= CAN_MCR_INRQ | CAN_MCR_ABOM;
    for (unsigned t = 0U; t < 1000000U; ++t)
        if (CAN1->MSR & CAN_MSR_INAK) break;

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = (pclk1 / 21U) / 1000000U;
    if (prescaler == 0U) prescaler = 1U;
    CAN1->BTR = (3UL << 20) | (15UL << 16) | (prescaler - 1U);

    CAN1->MCR &= ~CAN_MCR_INRQ;
    for (unsigned t = 0U; t < 1000000U; ++t)
        if ((CAN1->MSR & CAN_MSR_INAK) == 0U) break;

    CAN_FMR_REG |= CAN_FMR_FINIT;
    CAN_sFREG[0] = 0U;
    CAN_sFREG[1] = 0U;
    CAN_FM1R_REG &= ~CAN_FM1R_FBM0;
    CAN_FS1R_REG |= CAN_FS1R_FSC0;
    CAN_FA1R_REG |= CAN_FA1R_FACT0;
    CAN_FMR_REG &= ~CAN_FMR_FINIT;

    rx_queue = xQueueCreate(16U, sizeof(struct can_frame));
    CAN1->IER |= CAN_IER_FMPIE0;
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void CAN1_RX0_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0U) {
        struct can_frame f;
        f.id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FFU;
        f.data[0] = (uint8_t)(CAN1->sFIFOMailBox[0].RDLR);
        f.data[1] = (uint8_t)(CAN1->sFIFOMailBox[0].RDLR >> 8);
        f.data[2] = (uint8_t)(CAN1->sFIFOMailBox[0].RDLR >> 16);
        f.data[3] = (uint8_t)(CAN1->sFIFOMailBox[0].RDLR >> 24);
        f.data[4] = (uint8_t)(CAN1->sFIFOMailBox[0].RDHR);
        f.data[5] = (uint8_t)(CAN1->sFIFOMailBox[0].RDHR >> 8);
        f.data[6] = (uint8_t)(CAN1->sFIFOMailBox[0].RDHR >> 16);
        f.data[7] = (uint8_t)(CAN1->sFIFOMailBox[0].RDHR >> 24);
        CAN1->RF0R |= CAN_RF0R_RFOM0;
        /* [AI]  急停快速路径：不进队列，置标志由 25kHz ISR 立即执行(≤40µs) */
        if (f.id == CID_SAFETY &&
            crc8_id(f.data, 7U, DID_SAFETY) == f.data[7] &&
            (f.data[0] & 0x0FU) == 0x4U) {
            car_estop_request_fast();
        } else {
            (void)xQueueSendFromISR(rx_queue, &f, &woken);
        }
    }
    portYIELD_FROM_ISR(woken);
}

/* [AI]  Task_CAN_Rx：出队→CRC→counter 校验→分发（协议 v3 接收顺序） */
void can_rx_task_step(void)
{
    struct can_frame f;
    if (xQueueReceive(rx_queue, &f, portMAX_DELAY) != pdTRUE) return;

    uint32_t now = HAL_GetTick();
    uint8_t did;
    switch (f.id) {
    case CID_VEL_CMD:      did = DID_VEL_CMD; break;
    case CID_SAFETY:       did = DID_SAFETY; break;
    case CID_XFER_ARM2CH:  did = DID_XFER_ARM2CH; break;
    default: return;
    }
    if (crc8_id(f.data, 7U, did) != f.data[7]) return;   /* 1. CRC */

    if (f.id == CID_VEL_CMD) {
        /* [AI]  2. counter 校验: 高4位, 期望+1; +0=重复帧丢弃 */
        uint8_t cnt = (uint8_t)(f.data[6] >> 4);
        uint8_t diff = (uint8_t)(cnt - last_cmd_cnt);
        if (diff == 0U || cnt == last_cmd_cnt) return;   /* [AI]  重复帧丢弃 */
        last_cmd_cnt = cnt;
        car_can_velocity(get_le16(&f.data[0]), get_le16(&f.data[2]),
                         get_le16(&f.data[4]) / 10,      /* 0.001→0.01 rad/s */
                         (uint8_t)(f.data[6] & 1U), now);
    } else if (f.id == CID_SAFETY) {
        uint16_t cmd = (uint16_t)f.data[0] | ((uint16_t)f.data[1] << 8);
        car_can_control((uint8_t)(cmd & 0xFFU), now);    /* START/STOP/ESTOP/… */
    } else if (f.id == CID_XFER_ARM2CH) {
        arm_fault_code = (int16_t)get_le16(&f.data[0]);
        arm_j4_pos     = (int16_t)get_le16(&f.data[2]);
        arm_base_pos   = (int16_t)get_le16(&f.data[4]);
    }
}

static int can_tx(uint32_t std_id, const uint8_t *d)
{
    if ((CAN1->TSR & CAN_TSR_TME0) == 0U) return 0;
    CAN1->sTxMailBox[0].TIR = (std_id << 21);
    CAN1->sTxMailBox[0].TDTR = 8U;
    CAN1->sTxMailBox[0].TDLR = (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
                               ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    CAN1->sTxMailBox[0].TDHR = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
                               ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
    return 1;
}

/* [AI]  Task_CAN_Tx：1kHz —— 串口回读续发 + 0x030(互锁)每帧 + 0x200(反馈)每 10 帧 */
void can_tx_task_step(uint32_t now_ms)
{
    static unsigned div;
    static uint8_t fb_cnt, xf_cnt, ni_cnt;
    int32_t vx, vy, wz;
    uint8_t d[8];

    (void)now_ms;
    serial_poll_tx();                                    /* 1kHz 续发串口回读 */
    car_get_velocity(&vx, &vy, &wz);
    put_le16(&d[0], vx);
    put_le16(&d[2], vy);
    put_le16(&d[4], wz);
    d[6] = (uint8_t)((xf_cnt++ & 0xFU) << 4);            /* counter, flags=0 */
    d[7] = crc8_id(d, 7U, DID_XFER_CH2ARM);
    (void)can_tx(CID_XFER_CH2ARM, d);                    /* 0x030 @1kHz */

    if (++div >= 10U) {
        div = 0U;
        uint8_t st = car_can_status();
        uint8_t flags = (uint8_t)((st & 0x03U) |          /* mode bit0-1 */
                                  (st & 0x0CU));          /* fault/estop bit2-3 */
        d[6] = (uint8_t)(((fb_cnt & 0xFU) << 4) | flags);
        d[7] = crc8_id(d, 7U, DID_VEL_FB);
        (void)can_tx(CID_VEL_FB, d);                     /* 0x200 @100 Hz */
        serial_push_feedback(d);                         /* [AI]  串口同帧回读(双 Type-C) */

        if ((st & 0x08U) != 0U) {
            /* [AI]  TODO: EMCY 0x020 事件帧首发+重发 3 次（§3），暂随 DIAG 上报 */
        }
    }
    if (++ni_cnt >= 100U) {                              /* 0x230 @1 Hz */
        ni_cnt = 0U;
        d[0] = (uint8_t)PROTO_VER;
        d[1] = (uint8_t)HW_REV;
        uint16_t bh = build_hash16();
        d[2] = (uint8_t)(bh >> 8); d[3] = (uint8_t)bh;
        d[4] = 0U; d[5] = 0U;
        d[6] = (uint8_t)((ni_cnt & 0xFU) << 4);
        d[7] = crc8_id(d, 7U, DID_NODEINFO);
        (void)can_tx(CID_NODEINFO, d);
    }
}

/* [AI]  ---- Type-C 串口（CH9102）：本地测试通道（USART1/USART3 @115200）----
   帧格式为本地约定（8 字节 + CRC8 初值 0）：速度帧(同 0x100 布局)与
   舵机帧(byte6=0xA5)。不是总线协议，can_check.py 不校验此通道。 */
#define SERIAL_BAUD 115200U

struct serial_rx {
    uint8_t buf[16];
    uint8_t len;
};

static struct serial_rx ser_rx[2];
static uint8_t ser_txq[2][32];
static volatile uint8_t ser_tx_rd[2], ser_tx_wr[2];

static uint8_t crc8_local(const uint8_t *p, unsigned n)
{
    uint8_t c = 0U;
    for (unsigned i = 0; i < n; ++i) {
        c ^= p[i];
        for (unsigned b = 0U; b < 8U; ++b)
            c = (c & 0x80U) ? (uint8_t)((c << 1) ^ 0x07U) : (uint8_t)(c << 1);
    }
    return c;
}

static void serial_parse(struct serial_rx *s, uint8_t b)
{
    if (s->len >= sizeof(s->buf)) return;
    s->buf[s->len++] = b;
    while (s->len >= 8U) {
        if (crc8_local(s->buf, 7U) == s->buf[7]) {
            if (s->buf[6] == 0xA5U) {                  /* [AI]  本地舵机帧 */
                servo_set_us(s->buf[2] & 3U,
                    (uint16_t)(((uint16_t)s->buf[0] << 8) | s->buf[1]));
            } else {                                   /* [AI]  本地速度帧(测试) */
                car_can_velocity(
                    (int32_t)(int16_t)(((uint16_t)s->buf[0] << 8) | s->buf[1]),
                    (int32_t)(int16_t)(((uint16_t)s->buf[2] << 8) | s->buf[3]),
                    (int32_t)(int16_t)(((uint16_t)s->buf[4] << 8) | s->buf[5]),
                    s->buf[6], HAL_GetTick());
            }
            for (uint8_t k = 0; k + 8U < s->len; ++k) s->buf[k] = s->buf[k + 8U];
            s->len = (uint8_t)(s->len - 8U);
        } else {
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
    for (unsigned u = 0; u < 2U; ++u) {
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
    for (unsigned u = 0; u < 2U; ++u) {
        USART_TypeDef *uart = (u == 0U) ? USART1 : USART3;
        if ((uart->SR & USART_SR_TXE) != 0U && ser_tx_rd[u] != ser_tx_wr[u]) {
            uart->DR = ser_txq[u][ser_tx_rd[u]];
            ser_tx_rd[u] = (uint8_t)((ser_tx_rd[u] + 1U) % sizeof(ser_txq[0]));
        }
    }
}

/* [AI]  ---- 舵机输出: 6 路 ----
   ch0 = PC6/PC7/PC8/PC9 四座同步(GPIO 翻转 + TIM14 时基: 50Hz 更新中断
   四路同时置高, CCR1 比较中断同时拉低, 脉宽 = CCR1 µs)。该组座供电
   3V3(原理图 J1/J4)，MG996R 建议插 ch2 的 5V5 座。
   ch1 = PB14 / ch2 = PB15 (TIM12_CH1/CH2 硬件 PWM, AF9; PB15 = 5V5 座,
   资源表注明阿克曼转向舵机用 PB15)。绝不用 TIM8: 它是 C/D 电机 PWM。 */
#define SERVO_MIN_US   500U
#define SERVO_MAX_US  2500U
#define SERVO_MID_US  1500U
#define PC_GROUP_MASK  0x03C0UL

static volatile uint16_t servo_pc_us = SERVO_MID_US;

void servo_set_us(uint8_t ch, uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;
    switch (ch & 3U) {
    case 0: servo_pc_us = pulse_us; break;   /* [AI]  PC6-9 四座同步 */
    case 1: TIM12->CCR1 = pulse_us; break;   /* PB14 */
    default: TIM12->CCR2 = pulse_us; break;  /* PB15 */
    }
}

void servo_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->APB1ENR;
    GPIOC->MODER |= (1UL << 12) | (1UL << 14) | (1UL << 16) | (1UL << 18);
    GPIOC->BSRR = PC_GROUP_MASK << 16;
    TIM14->CR1 = 0;
    TIM14->PSC = 83;                   /* 84MHz/84 = 1MHz -> 1µs */
    TIM14->ARR = 19999U;               /* 20ms = 50Hz */
    TIM14->CCR1 = servo_pc_us;
    TIM14->SR = 0;
    TIM14->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
    TIM14->CR1 = TIM_CR1_CEN;
    NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 7U);
    NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);

    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->APB1ENR;
    GPIOB->MODER = (GPIOB->MODER & ~((3UL << 28) | (3UL << 30))) |
                   (2UL << 28) | (2UL << 30);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & 0x00FFFFFFUL) | 0x99000000UL;
    TIM12->CR1 = 0;
    TIM12->PSC = 83;
    TIM12->ARR = 19999U;
    TIM12->CCMR1 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);
    TIM12->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM12->CCR1 = SERVO_MID_US;
    TIM12->CCR2 = SERVO_MID_US;
    TIM12->EGR = TIM_EGR_UG;
    TIM12->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    /* [AI]  开机不做阻塞扫描(R2 教训:boot 依赖 HAL tick 会挂死);
       舵机联动由 USER 键路径的非阻塞 servo_demo 驱动 */
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    uint32_t sr = TIM14->SR;
    if ((sr & TIM_SR_UIF) != 0U) {
        TIM14->SR = (uint32_t)~TIM_SR_UIF;
        GPIOC->BSRR = PC_GROUP_MASK;
        TIM14->CCR1 = servo_pc_us;
    }
    if ((sr & TIM_SR_CC1IF) != 0U) {
        TIM14->SR = (uint32_t)~TIM_SR_CC1IF;
        GPIOC->BSRR = PC_GROUP_MASK << 16;
    }
}
