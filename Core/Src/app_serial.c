/**
 * [AI]  @file  app_serial.c — Type-C 串口通道:团队 UART v1 协议
 *
 * R16: 按团队对齐包重写(host 仓 docs/protocol/jetson_f407_uart_v1.md,
 * Jetson 侧 rak_base_driver 已实现)。废弃 R12 的 8 字节 CRC8 实验帧。
 * 帧格式: wire = COBS(帧) || 0x00;
 *         帧 = ver u8=1 + type u8 + seq u16LE + plen u16LE + payload
 *              + crc16 u16LE(CCITT-FALSE,覆盖 ver..payload)。
 * 消息: 0x01 HELLO / 0x02 HELLO_ACK / 0x10 SET_BASE_VELOCITY(15B) /
 *       0x11 BASE_STATE(23B) / 0x12 FAULT_EVENT(6B)。
 * 安全: 上电 DISARMED;HELLO(附带 DISARM)后才可 ARM(mode=1,且 fault==0);
 *       lease 超时 DISARM+锁存 bit1(HELLO 清);序号 (cand-prev)mod65536
 *       ∈ 1..32767,HELLO 重置;坏帧/非法 lease 不刷新 lease。
 * 故障位: 0 急停 1 lease 超时 2 CAN 对端离线(见过 0x031 再丢>500ms 才算,
 *         台架让路) 3 对端故障(0x031 FAULT_CODE) 4 电机 FAULT 5 欠压
 *         (<10.5V) 6 过温(板无传感器恒 0) 7 反馈无效(预留恒 0)。
 *
 * R20 CubeMX 迁移: USART1/USART3 的引脚/波特率/DMA/NVIC 全部由
 * MX_USARTx_UART_Init + MspInit 配置(921600 8N1;RX=循环 DMA,TX=Normal DMA;
 * IRQ 优先级 5)。本层改为:
 *   RX  HAL_UARTEx_ReceiveToIdle_DMA 循环武装,不依赖 RxEventCallback
 *       (其 Size 语义随事件类型变化:IDLE=当前位置/TC=满/HT=半满),任务侧
 *       1kHz 用 NDTR 差值排水进 COBS 解码器;
 *   ORE 在 DMA 模式下被 HAL 视为 blocking error(中止 RX DMA 并把 RxState
 *       置 READY)→ HAL_UART_ErrorCallback 里重武装,否则一次噪声就死 RX;
 *   TX  帧队列 + 单活动帧 HAL_UART_Transmit_DMA;完成检测/入队/启动全在
 *       任务上下文(cantx 任务 1kHz 泵),无 ISR 竞态。
 * 协议层(HELLO/lease/序号/fault 位图)与 R16 完全一致。
 */
#include "app_serial.h"
#include "app_can.h"
#include "app_car.h"
#include "main.h"
#include "usart.h"
#include "stm32f4xx.h"
#include <string.h>

#define V1_PROTO_VER   1U
#define V1_FEATURES    0x00000003UL   /* bit0 velocity + bit1 telemetry */
#define V1_BOARD_ID    0x0407U
#define V1_FW_VERSION  ((1UL << 16) | (20UL << 8) | 0UL)  /* minor=R 号 */
#define V1_LEASE_MIN   10U
#define V1_LEASE_MAX   500U   /* [AI] R17 台架临时放宽(规范 50):量化电机干扰对 RX 的影响 */
#define V1_UV_MV       10500U         /* 欠压阈值(3S),可调 */
#define V1_PEER_OFF_MS 500U           /* 0x031 静默多久算对端离线 */
#define V1_PLEN_MAX    128U

#define FB_ESTOP      0x00000001UL
#define FB_LEASE      0x00000002UL
#define FB_PEER_OFF   0x00000004UL
#define FB_PEER_FAULT 0x00000008UL
#define FB_MOTOR      0x00000010UL
#define FB_UNDERVOLT  0x00000020UL
#define FB_OVERTEMP   0x00000040UL   /* 板上无温度传感器,恒 0 */
#define FB_INV_FEED   0x00000080UL   /* 反馈无效判定预留,恒 0 */

enum { SERIAL_PORTS = 2 };
enum { V1_ST_BOOTING = 0, V1_ST_DISARMED = 1, V1_ST_ARMED = 2, V1_ST_FAULT = 3 };

/* ---- CRC16/CCITT-FALSE (poly 0x1021, init 0xFFFF, "123456789"→0x29B1) ---- */
static uint16_t v1_crc16(const uint8_t *p, uint16_t n)
{
    uint16_t crc = 0xFFFFU;
    while (n--) {
        crc ^= (uint16_t)((uint16_t)(*p++) << 8);
        for (unsigned i = 0; i < 8U; ++i) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put_le32(uint8_t *p, int32_t v)
{
    union { int32_t i; uint32_t u; } c;
    c.i = v;
    p[0] = (uint8_t)c.u;         p[1] = (uint8_t)(c.u >> 8);
    p[2] = (uint8_t)(c.u >> 16); p[3] = (uint8_t)(c.u >> 24);
}

static int32_t get_le32(const uint8_t *p)
{
    union { uint32_t u; int32_t i; } c;
    c.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
          ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return c.i;
}

/* ---- COBS 编码(经典算法,已对协议 golden vector 逐字节手验) ---- */
static uint16_t v1_cobs_encode(const uint8_t *in, uint16_t n, uint8_t *out)
{
    uint16_t rd = 0U, wr = 1U, code_pos = 0U;
    uint8_t code = 1U;
    while (rd < n) {
        uint8_t b = in[rd++];
        if (b == 0U) {
            out[code_pos] = code;
            code = 1U;
            code_pos = wr++;
        } else {
            out[wr++] = b;
            if (++code == 0xFFU) {
                out[code_pos] = 0xFFU;
                code = 1U;
                code_pos = wr++;
            }
        }
    }
    out[code_pos] = code;
    return wr;
}

/* ---- v1 状态 ---- */
static uint8_t  v1_handshaked, v1_armed;
static uint16_t v1_lease_ms = 50U;
static uint32_t v1_last_vel_ms;
static uint16_t v1_echo_seq;
static uint16_t v1_rx_prev;
static uint8_t  v1_rx_prev_ok;
static uint32_t boot_id;
static uint32_t v1_fault_latch;    /* bit1 lease 超时锁存,HELLO 清 */
static uint32_t v1_fault_prev;     /* FAULT_EVENT 上升沿检测 */
static uint16_t v1_state_div;
static uint16_t v1_tx_seq = 1U;

/* [AI] R17 RX 诊断(SWD 直读):定位电机干扰下丢帧的具体形式
   R20: v1_dbg_ore 口径放宽为"USART 错误重武装次数"(ORE/FE/NE/PE) */
volatile uint32_t v1_dbg_bytes;     /* 收到的 wire 字节(含定界符) */
volatile uint32_t v1_dbg_frames;    /* 通过 CRC 的 v1 帧 */
volatile uint32_t v1_dbg_crc_bad;   /* CRC 失败帧 */
volatile uint32_t v1_dbg_rejected;  /* 长度/版本/序号/lease 拒绝 */
volatile uint32_t v1_dbg_ore;       /* USART 错误重武装(字节级丢失事件) */
volatile uint32_t v1_dbg_tx_drop;   /* [AI] R20: TX 帧队列满丢弃数 */

static uint32_t v1_fault_bitmap(uint32_t now)
{
    uint32_t fb = v1_fault_latch;
    if ((car_flags() & 2U) != 0U) fb |= FB_ESTOP;
    if (car_mode() == 4U) fb |= FB_MOTOR;
    if (car_battery_mv() != 0U && car_battery_mv() < V1_UV_MV) fb |= FB_UNDERVOLT;
    if (can_arm_seen() != 0U) {
        if ((uint32_t)(now - can_arm_last_ms()) > V1_PEER_OFF_MS) fb |= FB_PEER_OFF;
        if (can_arm_fault_code() != 0) fb |= FB_PEER_FAULT;
    }
    return fb;
}

static uint8_t v1_state_now(uint32_t fb)
{
    if (car_control_ready() == 0U) return V1_ST_BOOTING;
    if (fb != 0U) return V1_ST_FAULT;
    return v1_armed ? V1_ST_ARMED : V1_ST_DISARMED;
}

/* ---- TX: 帧队列(每口一份) + 单活动帧 DMA ----
   [AI] R20: BASE_STATE 23B→wire ≤34B,HELLO_ACK 14B,横幅 4B——
   144B 上限 = 128B payload 帧 + COBS 最坏 3B + 定界符,留足余量。
   队满按"帧"整帧丢弃(比字节队列的半帧截断干净),计 v1_dbg_tx_drop。 */
enum { TXQ_N = 8, TXQ_FRAME_MAX = 144 };
static uint8_t  txq[SERIAL_PORTS][TXQ_N][TXQ_FRAME_MAX];
static uint16_t txq_len[TXQ_N];               /* [AI] 与口无关:同一帧两份等长 */
static uint8_t  txq_head[SERIAL_PORTS], txq_tail[SERIAL_PORTS], txq_cnt[SERIAL_PORTS];
static uint8_t  tx_active[SERIAL_PORTS][TXQ_FRAME_MAX];   /* DMA 在途缓冲 */
static uint16_t tx_active_len[SERIAL_PORTS];  /* >0 = 该口有帧在发 */

static void txq_push(unsigned u, const uint8_t *d, uint16_t n)
{
    if (n > TXQ_FRAME_MAX) return;
    if (txq_cnt[u] >= TXQ_N) {
        v1_dbg_tx_drop++;
        return;
    }
    memcpy(txq[u][txq_head[u]], d, n);
    txq_len[txq_head[u]] = n;
    txq_head[u] = (uint8_t)((txq_head[u] + 1U) % TXQ_N);
    txq_cnt[u]++;
}

void serial_push_text(const char *s)        /* 原始文本(识别横幅),双路 */
{
    uint16_t n = 0U;
    while (s[n] != '\0' && n < TXQ_FRAME_MAX) n++;
    for (unsigned u = 0; u < (unsigned)SERIAL_PORTS; ++u)
        txq_push(u, (const uint8_t *)s, n);
}

static void v1_send(uint8_t type, const uint8_t *payload, uint16_t plen)
{
    static uint8_t f[6U + V1_PLEN_MAX + 2U];
    static uint8_t w[6U + V1_PLEN_MAX + 2U + 3U];
    if (plen > V1_PLEN_MAX) return;
    f[0] = V1_PROTO_VER;
    f[1] = type;
    f[2] = (uint8_t)(v1_tx_seq & 0xFFU);
    f[3] = (uint8_t)(v1_tx_seq >> 8);
    f[4] = (uint8_t)(plen & 0xFFU);
    f[5] = (uint8_t)(plen >> 8);
    for (uint16_t i = 0; i < plen; ++i) f[6U + i] = payload[i];
    uint16_t crc = v1_crc16(f, (uint16_t)(6U + plen));
    f[6U + plen] = (uint8_t)(crc & 0xFFU);
    f[7U + plen] = (uint8_t)(crc >> 8);
    uint16_t wlen = v1_cobs_encode(f, (uint16_t)(8U + plen), w);
    w[wlen++] = 0x00U;                       /* 定界符 */
    if (++v1_tx_seq > 0x7FFFU) v1_tx_seq = 1U;
    for (unsigned u = 0; u < (unsigned)SERIAL_PORTS; ++u)
        txq_push(u, w, wlen);
}

static void v1_send_state(uint32_t now, uint32_t fb)
{
    uint8_t p[23];
    int32_t vx, vy, wz;
    car_get_velocity(&vx, &vy, &wz);
    p[0] = (uint8_t)(v1_echo_seq & 0xFFU);
    p[1] = (uint8_t)(v1_echo_seq >> 8);
    p[2] = v1_state_now(fb);
    p[3] = (uint8_t)(fb & 0xFFU);          p[4] = (uint8_t)((fb >> 8) & 0xFFU);
    p[5] = (uint8_t)((fb >> 16) & 0xFFU);  p[6] = (uint8_t)((fb >> 24) & 0xFFU);
    put_le32(&p[7], vx);
    put_le32(&p[11], vy);
    put_le32(&p[15], wz * 10);             /* 0.01rad/s → mrad/s */
    put_le32(&p[19], (int32_t)now);
    v1_send(0x11, p, 23);
}

/* ---- RX 帧处理(已过 COBS/长度/CRC;R20 起跑在任务上下文) ---- */
static void v1_frame(const uint8_t *f, uint16_t len)
{
    if (len < 8U || f[0] != V1_PROTO_VER) return;
    uint16_t plen = (uint16_t)(f[4] | ((uint16_t)f[5] << 8));
    if (plen > V1_PLEN_MAX || len != (uint16_t)(6U + plen + 2U)) return;
    uint16_t crc = v1_crc16(f, (uint16_t)(6U + plen));
    if (crc != (uint16_t)(f[6U + plen] | ((uint16_t)f[7U + plen] << 8))) {
        v1_dbg_crc_bad++;
        return;
    }
    v1_dbg_frames++;
    uint16_t seq = (uint16_t)(f[2] | ((uint16_t)f[3] << 8));
    uint32_t now = HAL_GetTick();

    if (f[1] == 0x01U && plen == 8U) {       /* HELLO */
        v1_handshaked = 1U;
        v1_armed = 0U;                       /* [AI] 保守:重新握手即 DISARM */
        v1_rx_prev_ok = 0U;                  /* 序号状态随 HELLO 重置 */
        v1_fault_latch &= ~FB_LEASE;         /* 新握手清 lease 超时锁存 */
        /* [AI] R20: 本函数已迁到任务上下文（RX 排水在 serial_rx_service），
           HELLO_ACK 直接内联发送，删 R16 的中断置标志间接层 */
        uint8_t p[14];
        p[0] = (uint8_t)(V1_BOARD_ID & 0xFFU);
        p[1] = (uint8_t)(V1_BOARD_ID >> 8);
        put_le32(&p[2], (int32_t)V1_FW_VERSION);
        put_le32(&p[6], (int32_t)boot_id);
        put_le32(&p[10], (int32_t)V1_FEATURES);
        v1_send(0x02, p, 14);
        return;
    }
    if (f[1] == 0x10U && plen == 15U) {      /* SET_BASE_VELOCITY */
        if (v1_rx_prev_ok != 0U) {
            uint16_t d = (uint16_t)(seq - v1_rx_prev);
            if (d == 0U || d > 0x7FFFU) { v1_dbg_rejected++; return; }
        }
        v1_rx_prev = seq;
        v1_rx_prev_ok = 1U;
        uint16_t lease = (uint16_t)(f[7] | ((uint16_t)f[8] << 8));
        if (lease < V1_LEASE_MIN || lease > V1_LEASE_MAX) { v1_dbg_rejected++; return; }
        uint32_t fb = v1_fault_bitmap(now);
        v1_lease_ms = lease;
        v1_last_vel_ms = now;
        v1_echo_seq = seq;
        if (f[6] == 1U && v1_handshaked != 0U && fb == 0U) {
            v1_armed = 1U;
            car_can_velocity(get_le32(&f[9]), get_le32(&f[13]),
                             get_le32(&f[17]) / 10,   /* mrad/s → 0.01rad/s */
                             1U, now);
        } else {
            v1_armed = 0U;
            car_can_velocity(0, 0, 0, 0, now);
        }
        return;
    }
    /* 其余类型(v1.1 0x13 等):丢弃,host 端自动回退 v1 */
}

/* ---- RX: COBS 流式解码(任务侧逐字节喂入) ----
   [AI]  解码状态**按端口各一份**。两路 Type-C 会交错来帧：共用一份状态时，
   一路收半帧被另一路插进来就会互相撕帧（悬空端口的线级噪声尤其容易触发）。
   协议层状态(v1_armed 等)仍是单份——同一时刻只应有一个 host 在讲话。 */
struct v1_rx_state {
    uint8_t  f[6U + V1_PLEN_MAX + 2U];
    uint16_t len;
    uint8_t  code, block, zero_pending;
};
static struct v1_rx_state v1_rx[SERIAL_PORTS];

static void v1_rx_byte(unsigned u, uint8_t b)
{
    struct v1_rx_state *r = &v1_rx[u];
    v1_dbg_bytes++;
    if (b == 0x00U) {                        /* 帧定界 */
        if (r->len != 0U) v1_frame(r->f, r->len);
        r->len = 0U; r->code = 0U; r->block = 0U; r->zero_pending = 0U;
        return;
    }
    if (r->block == 0U) {                    /* 取新 code 字节 */
        if (r->code != 0U && r->zero_pending != 0U) {
            /* 上一块隐含一个 0 */
            if (r->len >= sizeof(r->f)) { r->len = 0U; r->code = 0U; return; }
            r->f[r->len++] = 0x00U;
        }
        r->code = b;
        r->zero_pending = (uint8_t)(b != 0xFFU);
        r->block = (uint8_t)((b == 0xFFU) ? 254U : (b - 1U));
        return;
    }
    if (r->len < sizeof(r->f)) r->f[r->len++] = b;
    r->block--;
}

/* ---- RX DMA 循环缓冲 + 任务侧排水 ---- */
enum { RXBUF_N = 512U };
static uint8_t rxbuf[SERIAL_PORTS][RXBUF_N];
static uint16_t rx_pos[SERIAL_PORTS];          /* 上次排水到的环形位置 */
static volatile uint8_t rx_resync[SERIAL_PORTS];  /* ORE 重武装后置位 */

/* [AI] 1kHz(cantx 任务):把 DMA 环形缓冲里的新字节灌进 COBS 解码器。
   位置 = RXBUF_N - NDTR(剩余),即 DMA 已写的字节数;差值排水,支持回绕。 */
void serial_rx_service(void)
{
    for (unsigned u = 0; u < (unsigned)SERIAL_PORTS; ++u) {
        UART_HandleTypeDef *h = (u == 0U) ? &huart1 : &huart3;
        if (h->hdmarx == NULL || h->RxState != HAL_UART_STATE_BUSY_RX) continue;
        uint16_t pos = (uint16_t)(RXBUF_N - __HAL_DMA_GET_COUNTER(h->hdmarx));
        if (rx_resync[u] != 0U) {
            /* [AI] 重武装后 DMA 从 0 重填:丢掉旧位置关系并复位解码器
               (半帧丢弃,协议层靠 CRC/序号自愈;ORE 本身就丢字节) */
            rx_pos[u] = pos;
            rx_resync[u] = 0U;
            v1_rx[u].len = 0U; v1_rx[u].code = 0U;
            v1_rx[u].block = 0U; v1_rx[u].zero_pending = 0U;
        }
        while (rx_pos[u] != pos) {
            v1_rx_byte(u, rxbuf[u][rx_pos[u]]);
            rx_pos[u] = (uint16_t)((rx_pos[u] + 1U) % RXBUF_N);
        }
    }
}

/* [AI] R20: USART 错误(ORE/FE/NE/PE)在 DMA 模式下会由 HAL 中止 RX 传输
   (stm32f4xx_hal_uart.c 把 ORE 视作 blocking error → UART_EndRxTransfer)。
   不重武装的话一次线级噪声就永久死 RX——R18 的教训在 DMA 下依旧成立。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->RxState != HAL_UART_STATE_READY) return;  /* RX 未中止(如 TX 错误) */
    unsigned u = (huart->Instance == USART1) ? 0U : 1U;
    v1_dbg_ore++;
    rx_resync[u] = 1U;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, rxbuf[u], RXBUF_N);
}

/* ---- 1kHz 泵(cantx 任务):完成检测 → 出队 → 发起 DMA ----
   [AI] 全任务上下文,无竞态:同一时刻每口至多一帧在途(tx_active 持有缓冲,
   队列槽位在复制进 tx_active 后立即释放,不被 DMA 引用)。 */
void serial_poll_tx(void)
{
    for (unsigned u = 0; u < (unsigned)SERIAL_PORTS; ++u) {
        UART_HandleTypeDef *h = (u == 0U) ? &huart1 : &huart3;
        if (tx_active_len[u] != 0U) {
            /* Normal 模式 DMA 发完由 HAL 置 gState=READY */
            if (h->gState == HAL_UART_STATE_READY) tx_active_len[u] = 0U;
            else continue;
        }
        if (txq_cnt[u] == 0U) continue;
        uint8_t slot = txq_tail[u];
        memcpy(tx_active[u], txq[u][slot], txq_len[slot]);
        tx_active_len[u] = txq_len[slot];
        txq_tail[u] = (uint8_t)((txq_tail[u] + 1U) % TXQ_N);
        txq_cnt[u]--;
        if (HAL_UART_Transmit_DMA(h, tx_active[u], tx_active_len[u]) != HAL_OK)
            tx_active_len[u] = 0U;   /* [AI] 发起失败(不应发生):弃帧保活 */
    }
}

/* ---- 1kHz 节拍:lease 看门狗 + 故障事件 + 100Hz BASE_STATE ---- */
void serial_v1_tick(uint32_t now)
{
    if (v1_armed != 0U && (uint32_t)(now - v1_last_vel_ms) > v1_lease_ms) {
        v1_armed = 0U;                       /* lease 到期:DISARM+锁存 */
        v1_fault_latch |= FB_LEASE;
        car_can_velocity(0, 0, 0, 0, now);
    }
    uint32_t fb = v1_fault_bitmap(now);
    if (fb != 0U && v1_fault_prev == 0U) {   /* 上升沿 → FAULT_EVENT */
        uint8_t p[6];
        put_le32(&p[0], (int32_t)fb);
        p[4] = (uint8_t)(v1_echo_seq & 0xFFU);
        p[5] = (uint8_t)(v1_echo_seq >> 8);
        v1_send(0x12, p, 6);
    }
    v1_fault_prev = fb;
    if (++v1_state_div >= 10U) {             /* 100Hz */
        v1_state_div = 0U;
        v1_send_state(now, fb);
    }
}

void serial_init(void)
{
    /* [AI] R20 CubeMX 迁移:引脚/波特率/DMA/NVIC 均由 MX_USARTx_UART_Init
       + MspInit 完成(RX 引脚上拉补在 usart.c 的 USER CODE 块)。这里只
       武装循环 DMA 接收 + 算 boot_id。 */
    for (unsigned u = 0; u < (unsigned)SERIAL_PORTS; ++u) {
        UART_HandleTypeDef *h = (u == 0U) ? &huart1 : &huart3;
        rx_pos[u] = 0U;
        rx_resync[u] = 0U;
        /* [AI] 武装失败仅可能因上层配置错误;RX 失效但整机照常跑,OLED/SWD 可见 */
        (void)HAL_UARTEx_ReceiveToIdle_DMA(h, rxbuf[u], RXBUF_N);
    }

    /* boot_id: 每次上电必须不同(host 据此作废旧握手)——取 boot 时刻
       CYCCNT(随 IMU 探测重试等路径抖动)混 tick,保非零 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    boot_id = DWT->CYCCNT ^ ((uint32_t)HAL_GetTick() << 12) ^ 0x1F123BB5UL;
    if (boot_id == 0U) boot_id = V1_BOARD_ID;
}
