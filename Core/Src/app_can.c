/**
 * [AI]  @file  app_can.c — CAN 协议 v3（编解码/收发任务/邮箱分派）
 *
 * [AI]  协议规范见 docs/PROTOCOL.md（v3）：小端、DLC=8、byte6=counter(高4位)|
 * [AI]  flags(低4位)、byte7=CRC8(poly 0x07, 初值=该帧 Data ID, 覆盖字节0-6)。
 * [AI]  本文件实现：0x100 VEL_CMD(收) / 0x010 SAFETY(收, 急停走中断快速路径) /
 * [AI]  0x200 VEL_FB(发 100Hz) / 0x030 XFER(发 1kHz) / 0x230 NODE_INFO(发 1Hz)。
 */
#include "app_can.h"
#include "app_car.h"
#include "app_serial.h"
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

uint8_t crc8(const uint8_t *p, uint8_t n, uint8_t init)
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
            crc8(f.data, 7U, DID_SAFETY) == f.data[7] &&
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
    if (crc8(f.data, 7U, did) != f.data[7]) return;   /* 1. CRC */

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

/* [AI]  mb=0/1/2 分派三个发送邮箱：同拍连发若共用邮箱，后一帧必被 TME
   忙跳过（曾致 0x200/0x230 从未上线，见 README4 §二C） */
static int can_tx(unsigned mb, uint32_t std_id, const uint8_t *d)
{
    volatile CAN_TxMailBox_TypeDef *m = &CAN1->sTxMailBox[mb];
    const uint32_t tme = (mb == 0U) ? CAN_TSR_TME0 :
                         (mb == 1U) ? CAN_TSR_TME1 : CAN_TSR_TME2;
    if ((CAN1->TSR & tme) == 0U) return 0;
    m->TIR = (std_id << 21);
    m->TDTR = 8U;
    m->TDLR = (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
              ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    m->TDHR = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
              ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
    m->TIR |= CAN_TI0R_TXRQ;
    return 1;
}

/* [AI]  Task_CAN_Tx：1kHz —— 串口回读续发 + 0x030(互锁)每帧 + 0x200(反馈)每 10 帧 */
void can_tx_task_step(uint32_t now_ms)
{
    static unsigned div;
    static uint8_t fb_cnt, xf_cnt, ni_cnt, ni_seq;
    int32_t vx, vy, wz;
    uint8_t d[8];

    (void)now_ms;
    serial_poll_tx();                                    /* 1kHz 续发串口回读 */

    /* [AI]  邮箱死锁自愈：错误风暴后 bxCAN 可把邮箱挂在"忙但不再重试"态，
       固件只查 TME 会永远不再排帧→整机静默；任一邮箱忙超 150ms 强制
       ABRQ 释放重排（正常 ≤1ms 永不误触发，见 README4 §二C） */
    {
        static uint16_t tx_busy_ms[3];
        static const uint32_t tx_tme[3]  = { CAN_TSR_TME0, CAN_TSR_TME1, CAN_TSR_TME2 };
        static const uint32_t tx_abrq[3] = { CAN_TSR_ABRQ0, CAN_TSR_ABRQ1, CAN_TSR_ABRQ2 };
        for (unsigned m = 0U; m < 3U; ++m) {
            if ((CAN1->TSR & tx_tme[m]) != 0U) {
                tx_busy_ms[m] = 0U;
            } else if (++tx_busy_ms[m] > 150U) {
                CAN1->TSR = tx_abrq[m];
                tx_busy_ms[m] = 0U;
            }
        }
    }
    car_get_velocity(&vx, &vy, &wz);
    put_le16(&d[0], vx);
    put_le16(&d[2], vy);
    put_le16(&d[4], wz);
    d[6] = (uint8_t)((xf_cnt++ & 0xFU) << 4);            /* counter, flags=0 */
    d[7] = crc8(d, 7U, DID_XFER_CH2ARM);
    (void)can_tx(0U, CID_XFER_CH2ARM, d);                    /* 0x030 @1kHz */

    if (++div >= 10U) {
        div = 0U;
        uint8_t st = car_can_status();
        uint8_t flags = (uint8_t)((st & 0x03U) |          /* mode bit0-1 */
                                  (st & 0x0CU));          /* fault/estop bit2-3 */
        d[6] = (uint8_t)(((fb_cnt++ & 0xFU) << 4) | flags);
        d[7] = crc8(d, 7U, DID_VEL_FB);
        (void)can_tx(1U, CID_VEL_FB, d);                     /* 0x200 @100 Hz */
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
        d[6] = (uint8_t)((ni_seq++ & 0xFU) << 4);
        d[7] = crc8(d, 7U, DID_NODEINFO);
        (void)can_tx(2U, CID_NODEINFO, d);
        serial_push_text(SERIAL_BANNER);   /* [AI]  R15: 串口识别横幅 @1Hz */
    }
}
