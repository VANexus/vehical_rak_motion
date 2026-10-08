/**
 * [AI]  @file  app_can.c — CAN 协议 v4（编解码/收发任务，CAN 为主链路）
 *
 * [AI]  协议规范见 docs/PROTOCOL.md（v4）：小端、DLC=8；仅 0x010 SAFETY_CMD 为
 * [AI]  e2e 帧（byte6=counter|flags、byte7=CRC8 poly0x07 初值=Data ID），其余为
 * [AI]  raw8 纯数据帧（完整性由 CAN 硬件 CRC/ACK/重传保证）。
 * [AI]  R45 实现——下行：0x100 DRIVE_CMD(4 轮速 0.01rad/s)、0x110/111 ARM_CMD
 * [AI]  (7 轴 0.1°)、0x010 SAFETY(急停 ISR 快速路径 + START/STOP 远程门)；
 * [AI]  上行：0x200/201 WHEEL_POS(100Hz)、0x202 WHEEL_SPD(100Hz)、
 * [AI]  0x210/211 SERVO_STATE(50Hz)、0x220 DIAG(10Hz)、0x030 NODE_INFO(1Hz)。
 * [AI]  v3 的 XFER 互锁/VEL_FB/NODE_INFO@0x230 全部作废（臂板退役）。
 */
#include "app_can.h"
#include "app_car.h"
#include "app_can_control_policy.h"
#include "app_serial.h"
#include "app_servo.h"   /* [AI] R49: SERVO_STATE 遥测读回 servo_get_pulse */
#include "main.h"
#include "can.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"

/* ---- bxCAN1: PD0=RX / PD1=TX (AF9), 1Mbps, APB1 42MHz ----
   [AI] R20 CubeMX 迁移：引脚/位时序/NVIC 由 MX_CAN1_Init + MspInit 完成
   （Prescaler=2, BS1=16TQ, BS2=4TQ → 1Mbps@42MHz；CAN1_RX0 IRQ=6）。
   本文件保留对 TSR 的直读/直写（发送邮箱自愈见 can_tx_task_step）。 */

/* [AI]  CAN v4 帧 ID 与 Data ID——与 docs/protocol-signals.yaml (version:4)
   逐字段一致（单点真源在 YAML，此处为固件侧镜像；改动必须先改 YAML 并过
   tools/can_check.py 门禁）。 */
#define CID_SAFETY       0x010U
#define DID_SAFETY       0x01U
#define CID_EMCY         0x020U     /* [AI] v4 事件帧，R45 暂不发（DIAG 覆盖） */
#define DID_EMCY         0x02U
#define CID_NODEINFO     0x030U
#define DID_NODEINFO     0x03U
#define CID_DRIVE_CMD    0x100U
#define DID_DRIVE_CMD    0x04U
#define CID_ARM_CMD_A    0x110U
#define DID_ARM_A        0x05U
#define CID_ARM_CMD_B    0x111U
#define DID_ARM_B        0x06U
#define CID_WHEEL_POS_A  0x200U
#define CID_WHEEL_POS_B  0x201U
#define CID_WHEEL_SPD    0x202U
#define CID_SERVO_A      0x210U
#define CID_SERVO_B      0x211U
#define CID_DIAG         0x220U
#define PROTO_VER        4U
#define HW_REV           1U

/* [AI] v4 量化常数（协议单点定义在 YAML，换算处集中在此）
   DRIVE_CMD: int16 × 0.01 rad/s → 内部 urad/s：×10^4
   ARM_CMD:   uint16 × 0.1°   → 内部 urad  ：°→urad = ×17453.3 → raw×174533/1000
   WHEEL_SPD: 内部 urad/s → 0.01 rad/s：÷10^4 */
#define DRIVE_URAD_PER_LSB   10000L
#define ARM_URAD_NUM      174533L
#define ARM_URAD_DEN        1000L
#define SPD_URAD_PER_LSB   10000L

/* [AI] v4 deadman：DRIVE/ARM 指令流 100 ms 断流（docs/PROTOCOL.md §6.2） */
#define CMD_DEADMAN_MS     100U

struct can_frame {
    uint32_t id;
    uint8_t data[8];
};

static QueueHandle_t rx_queue;

/* [AI] SAFETY START/STOP 置位的 CAN 远程门：1=允许 DRIVE/ARM 落地。
   急停锁存期间 START 被拒（ISR 内查 car_flags）。 */
static volatile uint8_t can_remote;
static volatile uint32_t last_drive_ms;   /* [AI] 最近 DRIVE_CMD 到达 tick */
static volatile uint32_t last_arm_ms;     /* [AI] 最近 ARM_CMD 到达 tick */
static volatile uint8_t drive_stale;      /* [AI] 已执行过断流清零（边沿触发） */

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

static void put_le32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)(uint32_t)v;
    p[1] = (uint8_t)((uint32_t)v >> 8);
    p[2] = (uint8_t)((uint32_t)v >> 16);
    p[3] = (uint8_t)((uint32_t)v >> 24);
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

/* [AI] v4 STATE_u16 位图（docs/protocol-signals.yaml state_bits）：
   bit0 estop / bit1 remote / bit2 fault / bit3 cmd_stale / bit4 bat_uv /
   bit5 can_recovering。供 DIAG 与 NODE_INFO 共用。
   car_can_status()：bit0=remote_running、bit1=FAULT、bit2=estop。 */
static uint16_t state_bitmap(uint8_t busoff_now)
{
    uint8_t cs = car_can_status();
    uint16_t s = 0U;
    if ((cs & 4U) != 0U)                   s |= 1U << 0;   /* estop_latched */
    if (can_remote != 0U)                  s |= 1U << 1;   /* remote_running */
    if ((cs & 2U) != 0U)                   s |= 1U << 2;   /* fault_active */
    if (drive_stale != 0U)                 s |= 1U << 3;   /* cmd_stale */
    if (car_battery_mv() < CAR_BATTERY_UV_THRESHOLD_MV) s |= 1U << 4;
    if (busoff_now != 0U)                  s |= 1U << 5;   /* can_recovering */
    return s;
}

/* [AI] v4 MODE_u8（mode_enum）：0 IDLE / 1 REMOTE / 2 ESTOP / 3 FAULT */
static uint8_t mode_u8(void)
{
    uint8_t cs = car_can_status();
    if ((cs & 4U) != 0U) return 2U;                      /* estop 锁存最高 */
    if ((cs & 2U) != 0U) return 3U;                      /* FAULT */
    return (can_remote != 0U) ? 1U : 0U;
}

/* [AI]  ---- 初始化：过滤器/启动/通知在本函数，队列先于中断使能创建 ---- */
void can_init(void)
{
    rx_queue = xQueueCreate(16U, sizeof(struct can_frame));

    /* [AI] 过滤器：mask=0 全收（协议分拣在中断/任务侧，与 v3 一致） */
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
        /* [AI]  0x010 e2e 快速路径：CRC(DID 种子)校验在 ISR 内完成——
           EMERGENCY_STOP 置标志由 25kHz ISR 立即执行(≤40µs)；
           START/STOP 只翻远程门标志；其余命令码丢弃（CLEAR_ESTOP 仅按钮） */
        if (f.id == CID_SAFETY) {
            if (crc8(f.data, 7U, DID_SAFETY) == f.data[7]) {
                switch (f.data[0] & 0x0FU) {
                case 4U:                                    /* EMERGENCY_STOP */
                    car_estop_request_fast();
                    can_remote = 0U;
                    break;
                case 1U:                                    /* START */
                    if ((car_flags() & 2U) == 0U) can_remote = 1U;
                    break;
                case 2U:                                    /* STOP */
                    can_remote = 0U;
                    break;
                default:                                    /* 5/6/7 未授权 */
                    break;
                }
            }
            continue;
        }
        if (f.id == CID_DRIVE_CMD || f.id == CID_ARM_CMD_A || f.id == CID_ARM_CMD_B) {
            (void)xQueueSendFromISR(rx_queue, &f, &woken);
        }
        /* [AI] 其余 ID（遥测/管理/未知）本节点不消费，静默丢弃 */
    }
    portYIELD_FROM_ISR(woken);
}

/* [AI]  Task_CAN_Rx：出队→解码→落地。DRIVE/ARM 指令在任务侧换算
   （任务上下文允许乘除；ISR 内只做 CRC 与标志位）。 */
void can_rx_task_step(void)
{
    struct can_frame f;
    if (xQueueReceive(rx_queue, &f, portMAX_DELAY) != pdTRUE) return;

    uint32_t now = HAL_GetTick();

    if (f.id == CID_DRIVE_CMD) {
        int32_t w[CAR_MOTOR_COUNT];
        for (unsigned i = 0; i < CAR_MOTOR_COUNT; ++i) {
            /* [AI] raw×0.01 rad/s → urad/s；饱和已由 int16 域保证 */
            w[i] = get_le16(&f.data[i * 2U]) * DRIVE_URAD_PER_LSB;
        }
        car_wheel_command_urad(w, can_remote, now);
        last_drive_ms = now;
        drive_stale = 0U;
        return;
    }

    if (f.id == CID_ARM_CMD_A || f.id == CID_ARM_CMD_B) {
        /* [AI] 轴序单点定义见 app_car.h：axis[0..6]=J1..J6+夹爪。
           A 帧=J1..J4，B 帧=J5,J6,夹爪；凑齐 7 轴整组下发（valid=0x7F）。
           夹爪无硬件通道（app_car.h），上位机约定其值仅回显不落地。 */
        static int32_t pending[7];
        static uint8_t have_a, have_b;
        int32_t p[7];
        if (f.id == CID_ARM_CMD_A) {
            for (unsigned i = 0; i < 4U; ++i) {
                pending[i] = (int32_t)get_le16(&f.data[i * 2U]) *
                             ARM_URAD_NUM / ARM_URAD_DEN;
            }
            have_a = 1U;
        } else {
            for (unsigned i = 0; i < 3U; ++i) {
                pending[4U + i] = (int32_t)get_le16(&f.data[i * 2U]) *
                                  ARM_URAD_NUM / ARM_URAD_DEN;
            }
            have_b = 1U;
        }
        last_arm_ms = now;
        if (have_a == 0U || have_b == 0U) return;   /* [AI] 首组未齐不下发 */
        for (unsigned i = 0; i < 7U; ++i) p[i] = pending[i];
        car_arm_command_urad(p, 0x7FU);
        return;
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

/* [AI]  Task_CAN_Tx：1kHz 底拍。串口泵（USB v2 备用链路）+ 遥测调度。
   相位错开：10ms 轮位/轮速、20+5ms 舵机、50+50ms DIAG、0ms NODE_INFO，
   避免同拍挤爆 3 个发送邮箱。 */
void can_tx_task_step(uint32_t now_ms)
{
    static unsigned ph;                  /* 1kHz 相位计数 0..999 */
    static uint8_t busoff_prev;
    static uint8_t busoff_cnt;
    uint8_t d[8];

    serial_poll_tx();                    /* 1kHz 泵串口 TX（R20: DMA 版） */
    serial_rx_service(); /* [AI]  R20: RX 循环 DMA 排水→COBS 解码（任务侧轮询） */
    serial_v2_tick(now_ms);          /* [AI] UART v2 lease、安全状态与遥测节拍（备用链路） */

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

    /* [AI] DRIVE/ARM 断流（100ms）：远程门开着但指令流断了 → 轮速目标清零
       （enable=0 松链接）；舵机无目标概念，自然保持当前位置。边沿置位供
       STATE.bit3 cmd_stale。 */
    if (can_remote != 0U &&
        (uint32_t)(now_ms - last_drive_ms) > CMD_DEADMAN_MS) {
        int32_t zero[CAR_MOTOR_COUNT] = {0, 0, 0, 0};
        car_wheel_command_urad(zero, 0U, now_ms);
        drive_stale = 1U;
    }

    /* [AI] bus-off 计数：ESR.BOFF 上升沿累计（ABOM 自动恢复，bit 自清） */
    {
        uint8_t bo = (CAN1->ESR & CAN_ESR_BOFF) ? 1U : 0U;
        if (bo != 0U && busoff_prev == 0U && busoff_cnt < 255U) ++busoff_cnt;
        busoff_prev = bo;
    }

    if (ph % 10U == 0U) {                            /* 100 Hz：轮位×2 + 轮速 */
        int64_t pos_urad[CAR_MOTOR_COUNT];
        int32_t vel_urad_s[CAR_MOTOR_COUNT];
        uint8_t vm;
        uint32_t ep;
        uint64_t su;
        int32_t pos_counts[CAR_MOTOR_COUNT];

        car_encoder_snapshot(pos_urad, vel_urad_s, &vm, &ep, &su);
        car_wheel_positions(pos_counts);
        put_le32(&d[0], pos_counts[0]);
        put_le32(&d[4], pos_counts[1]);
        (void)can_tx(CID_WHEEL_POS_A, d);
        put_le32(&d[0], pos_counts[2]);
        put_le32(&d[4], pos_counts[3]);
        (void)can_tx(CID_WHEEL_POS_B, d);
        for (unsigned i = 0; i < CAR_MOTOR_COUNT; ++i) {
            put_le16(&d[i * 2U], vel_urad_s[i] / SPD_URAD_PER_LSB);
        }
        (void)can_tx(CID_WHEEL_SPD, d);
    }
    if (ph % 20U == 5U) {                            /* 50 Hz：舵机实际角 */
        /* [AI] 通道→轴映射与 car_arm_command_urad 相反：ch0..ch5=J6..J1。
           脉宽→0.1°：(us-500)×270/2000=(us-500)×27/20。
           A 帧=J1..J4（8 字节整），B 帧=J5,J6,夹爪；夹爪无通道恒发 0。 */
        static const unsigned ch_of_axis[6] = { 5U, 4U, 3U, 2U, 1U, 0U };
        for (unsigned i = 0; i < 4U; ++i) {
            unsigned us = servo_get_pulse(ch_of_axis[i]);
            put_le16(&d[i * 2U], (int32_t)(((uint32_t)(us - 500U) * 27U) / 20U));
        }
        (void)can_tx(CID_SERVO_A, d);
        for (unsigned i = 0; i < 2U; ++i) {
            unsigned us = servo_get_pulse(ch_of_axis[4U + i]);
            put_le16(&d[i * 2U], (int32_t)(((uint32_t)(us - 500U) * 27U) / 20U));
        }
        put_le16(&d[4], 0);                          /* 夹爪：无硬件通道 */
        put_le16(&d[6], 0);                          /* RSVD16 */
        (void)can_tx(CID_SERVO_B, d);
    }
    /* [AI] 相位错开原则：ph%10==0 的轮位爆发(3帧)占满 3 邮箱，DIAG/NODE_INFO
       必须落在 %10!=0 且避开 %20==5(舵机×2) 的相位，否则作为第 4 帧必然被丢。 */
    if (ph % 100U == 55U) {                          /* 10 Hz：DIAG（55: 不撞轮位/舵机） */
        uint16_t st = state_bitmap(busoff_prev);
        put_le16(&d[0], (int32_t)car_battery_mv());
        put_le16(&d[2], (st & (1U << 4)) ? 0x30 : 0);   /* BAT_UV */
        put_le16(&d[4], (int32_t)st);
        d[6] = mode_u8();
        d[7] = busoff_cnt;
        (void)can_tx(CID_DIAG, d);
    }
    if (ph == 999U) {                                /* 1 Hz：NODE_INFO（999: 独占毫秒） */
        uint16_t st = state_bitmap(busoff_prev);
        d[0] = (uint8_t)PROTO_VER;
        d[1] = (uint8_t)HW_REV;
        uint16_t bh = build_hash16();
        d[2] = (uint8_t)(bh >> 8); d[3] = (uint8_t)bh;
        put_le16(&d[4], (st & (1U << 4)) ? 0x30 : 0);
        put_le16(&d[6], (int32_t)st);
        (void)can_tx(CID_NODEINFO, d);
    }
    if (++ph >= 1000U) ph = 0U;
}

/* [AI]  R16: UART v1 故障位数据出口(对端=0x031 机械臂)
   —— v4 起互锁帧作废，恒回 0；函数保留仅为冻结的 UART v1 备用路径可编译。 */
uint8_t can_arm_seen(void)       { return 0U; }
uint32_t can_arm_last_ms(void)   { return 0U; }
int16_t can_arm_fault_code(void) { return 0; }
