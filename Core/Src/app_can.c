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
#include "can.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"

/* ---- bxCAN1: PD0=RX / PD1=TX (AF9), 1Mbps, APB1 42MHz ----
   [AI] R20 CubeMX 迁移：引脚/位时序/NVIC 由 MX_CAN1_Init + MspInit 完成
   （Prescaler=2, BS1=16TQ, BS2=4TQ → 1Mbps@42MHz；CAN1_RX0 IRQ=6）。
   本文件保留对 TSR 的直读/直写（发送邮箱自愈见 can_tx_task_step）。 */

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
static int16_t arm_fault_code;
static uint32_t arm_last_ms;
static uint8_t arm_seen;        /* [AI]  0x031: 机械臂故障码（暂存） */
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

/* [AI]  ---- 初始化：过滤器/启动/通知在本函数，队列先于中断使能创建 ---- */
void can_init(void)
{
    rx_queue = xQueueCreate(16U, sizeof(struct can_frame));

    /* [AI] 过滤器：mask=0 全收（协议分拣在 can_rx_task_step，与寄存器版一致） */
    CAN_FilterTypeDef flt = {0};
    flt.FilterBank = 0U;
    flt.FilterMode = CAN_FILTERMODE_IDMASK;
    flt.FilterScale = CAN_FILTERSCALE_32BIT;
    flt.FilterIdHigh = 0U;
    flt.FilterIdLow = 0U;
    flt.FilterMaskIdHigh = 0U;
    flt.FilterMaskIdLow = 0U;
    flt.FilterFIFOAssignment = CAN_RX_FIFO0;
    flt.FilterActivation = ENABLE;
    flt.SlaveStartFilterBank = 14U;
    if (HAL_CAN_ConfigFilter(&hcan1, &flt) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();   /* [AI] CAN 起不来 = 遥控全失效，boot 即暴露 */
    }
}

/* [AI] R20: 由 stm32f4xx_it.c 的 CAN1_RX0_IRQHandler → HAL_CAN_IRQHandler
   路调到本回调（FIFO0 挂起中断）。drain 到 FMP=0 再返回，减少重入延迟。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t d[8];
    BaseType_t woken = pdFALSE;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U) {
        struct can_frame f;
        for (unsigned k = 0; k < 8U; ++k) d[k] = 0U;
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, d) != HAL_OK) break;
        f.id = hdr.StdId;
        for (unsigned k = 0; k < 8U; ++k) f.data[k] = d[k];
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
        arm_seen = 1U;                   /* [AI]  R16: 对端在线证据(UART v1 fault bit2/3) */
        arm_last_ms = now;
        arm_fault_code = (int16_t)get_le16(&f.data[0]);
        arm_j4_pos     = (int16_t)get_le16(&f.data[2]);
        arm_base_pos   = (int16_t)get_le16(&f.data[4]);
    }
}

/* [AI] R20 HAL 化：HAL_CAN_AddTxMessage 自动选第一个空邮箱——原"手动分派
   0/1/2 邮箱"的目的（同拍连发不互踩，README4 §二C）由 HAL 的选箱逻辑
   天然满足；三邮箱全忙才返回失败（丢帧，下周期重排，语义同寄存器版）。
   邮箱死锁自愈仍在 can_tx_task_step（TSR 直写 ABRQ，不走 HAL）。 */
static int can_tx(uint32_t std_id, const uint8_t *d)
{
    CAN_TxHeaderTypeDef hdr = {0};
    hdr.StdId = std_id;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = 8U;
    return (HAL_CAN_AddTxMessage(&hcan1, &hdr, d, NULL) == HAL_OK) ? 1 : 0;
}

/* [AI]  Task_CAN_Tx：1kHz —— 串口回读续发 + 0x030(互锁)每帧 + 0x200(反馈)每 10 帧 */
void can_tx_task_step(uint32_t now_ms)
{
    static unsigned div;
    static uint8_t fb_cnt, xf_cnt, ni_cnt, ni_seq;
    int32_t vx, vy, wz;
    uint8_t d[8];

    serial_poll_tx();                    /* 1kHz 泵串口 TX（R20: DMA 版） */
    serial_rx_service(); /* [AI]  R20: RX 循环 DMA 排水→COBS 解码（任务侧轮询） */
    serial_v1_tick(now_ms);          /* [AI]  R16: UART v1 lease 看门狗 + BASE_STATE 节拍 */

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
    (void)can_tx(CID_XFER_CH2ARM, d);                    /* 0x030 @1kHz */

    if (++div >= 10U) {
        div = 0U;
        uint8_t st = car_can_status();
        uint8_t flags = (uint8_t)((st & 0x03U) |          /* mode bit0-1 */
                                  (st & 0x0CU));          /* fault/estop bit2-3 */
        d[6] = (uint8_t)(((fb_cnt++ & 0xFU) << 4) | flags);
        d[7] = crc8(d, 7U, DID_VEL_FB);
        (void)can_tx(CID_VEL_FB, d);                     /* 0x200 @100 Hz */

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
        (void)can_tx(CID_NODEINFO, d);
        serial_push_text(SERIAL_BANNER);   /* [AI]  R15: 串口识别横幅 @1Hz */
    }
}

/* [AI]  R16: UART v1 故障位数据出口(对端=0x031 机械臂) */
uint8_t can_arm_seen(void)       { return arm_seen; }
uint32_t can_arm_last_ms(void)   { return arm_last_ms; }
int16_t can_arm_fault_code(void) { return arm_fault_code; }
