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
 */
#include "app_serial.h"
#include "app_can.h"
#include "app_car.h"
#include "main.h"
#include "stm32f4xx.h"

#define V1_BAUD        921600U
#define V1_PROTO_VER   1U
#define V1_FEATURES    0x00000003UL   /* bit0 velocity + bit1 telemetry */
#define V1_BOARD_ID    0x0407U
#define V1_FW_VERSION  ((1UL << 16) | (16UL << 8) | 0UL)  /* minor=R 号 */
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

/* ---- 双路 TX 队列 + TXE 中断续发(100Hz×34B≈3.4kB/s,轮询泵喂不满) ---- */
enum { TXQ = 160 };
static uint8_t ser_txq[SERIAL_PORTS][TXQ];
static volatile uint8_t ser_tx_rd[SERIAL_PORTS], ser_tx_wr[SERIAL_PORTS];

static void txq_push(unsigned u, const uint8_t *d, uint16_t n)
{
    for (uint16_t i = 0; i < n; ++i) {
        uint8_t next = (uint8_t)((ser_tx_wr[u] + 1U) % TXQ);
        if (next == ser_tx_rd[u]) return;   /* 队满截断,下周期重试 */
        ser_txq[u][ser_tx_wr[u]] = d[i];
        ser_tx_wr[u] = next;
    }
}

void serial_push_text(const char *s)        /* 原始文本(识别横幅),双路 */
{
    for (unsigned u = 0; u < (unsigned)SERIAL_PORTS; ++u) {
        for (const char *p = s; *p != '\0'; ++p) {
            uint8_t next = (uint8_t)((ser_tx_wr[u] + 1U) % TXQ);
            if (next == ser_tx_rd[u]) break;
            ser_txq[u][ser_tx_wr[u]] = (uint8_t)*p;
            ser_tx_wr[u] = next;
        }
    }
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

/* [AI] R17 RX 诊断(SWD 直读):定位电机干扰下丢帧的具体形式 */
volatile uint32_t v1_dbg_bytes;     /* 收到的 wire 字节(含定界符) */
volatile uint32_t v1_dbg_frames;    /* 通过 CRC 的 v1 帧 */
volatile uint32_t v1_dbg_crc_bad;   /* CRC 失败帧 */
volatile uint32_t v1_dbg_rejected;  /* 长度/版本/序号/lease 拒绝 */
volatile uint32_t v1_dbg_ore;       /* USART 溢出(字节级丢失) */

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
    txq_push(0U, w, wlen);
    txq_push(1U, w, wlen);
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

/* ---- RX 帧处理(已过 COBS/长度/CRC;context=USART IRQ) ---- */
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

/* ---- RX: COBS 流式解码(USART IRQ 逐字节喂入) ---- */
static uint8_t  rxf[6U + V1_PLEN_MAX + 2U];
static uint16_t rx_len;
static uint8_t  rx_code, rx_block, rx_zero_pending;

static void v1_rx_byte(uint8_t b)
{
    v1_dbg_bytes++;
    if (b == 0x00U) {                        /* 帧定界 */
        if (rx_len != 0U) v1_frame(rxf, rx_len);
        rx_len = 0U; rx_code = 0U; rx_block = 0U; rx_zero_pending = 0U;
        return;
    }
    if (rx_block == 0U) {                    /* 取新 code 字节 */
        if (rx_code != 0U && rx_zero_pending != 0U) {
            /* 上一块隐含一个 0 */
            if (rx_len >= sizeof(rxf)) { rx_len = 0U; rx_code = 0U; return; }
            rxf[rx_len++] = 0x00U;
        }
        rx_code = b;
        rx_zero_pending = (uint8_t)(b != 0xFFU);
        rx_block = (uint8_t)((b == 0xFFU) ? 254U : (b - 1U));
        return;
    }
    if (rx_len < sizeof(rxf)) rxf[rx_len++] = b;
    rx_block--;
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

/* ---- 1kHz 泵:TXE 空则发首字节并开 TXE 中断,余下由中断续发 ---- */
void serial_poll_tx(void)
{
    for (unsigned u = 0; u < (unsigned)SERIAL_PORTS; ++u) {
        USART_TypeDef *uart = (u == 0U) ? USART1 : USART3;
        if ((uart->SR & USART_SR_TXE) != 0U && ser_tx_rd[u] != ser_tx_wr[u]) {
            uart->DR = ser_txq[u][ser_tx_rd[u]];
            ser_tx_rd[u] = (uint8_t)((ser_tx_rd[u] + 1U) % TXQ);
            uart->CR1 |= USART_CR1_TXEIE;
        }
    }
}

static void v1_uart_irq(USART_TypeDef *uart, unsigned u)
{
    uint32_t sr = uart->SR;
    if ((sr & USART_SR_ORE) != 0U) v1_dbg_ore++;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE)) != 0U) {
        uint8_t b = (uint8_t)uart->DR;       /* 读 DR 清 RXNE/ORE */
        if ((sr & USART_SR_RXNE) != 0U) v1_rx_byte(b);
    }
    if ((uart->CR1 & USART_CR1_TXEIE) != 0U && (sr & USART_SR_TXE) != 0U) {
        if (ser_tx_rd[u] != ser_tx_wr[u]) {
            uart->DR = ser_txq[u][ser_tx_rd[u]];
            ser_tx_rd[u] = (uint8_t)((ser_tx_rd[u] + 1U) % TXQ);
        } else {
            uart->CR1 &= ~USART_CR1_TXEIE;   /* 队列空:关中断 */
        }
    }
}

void USART1_IRQHandler(void) { v1_uart_irq(USART1, 0U); }
void USART3_IRQHandler(void) { v1_uart_irq(USART3, 1U); }

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
    USART1->BRR = (HAL_RCC_GetPCLK2Freq() + V1_BAUD / 2U) / V1_BAUD;
    USART3->BRR = (HAL_RCC_GetPCLK1Freq() + V1_BAUD / 2U) / V1_BAUD;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    /* [AI] R18: 优先级 7->4(低于 RTOS 临界区阈值 5,不被 BASEPRI 掩盖)。
       本 ISR 不调用任何 FreeRTOS API,合规;ORE 实测来自被推迟导致的字节丢失。 */
    NVIC_SetPriority(USART1_IRQn, 4U);
    NVIC_SetPriority(USART3_IRQn, 4U);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);

    /* boot_id: 每次上电必须不同(host 据此作废旧握手)——取 boot 时刻
       CYCCNT(随 IMU 探测重试等路径抖动)混 tick,保非零 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    boot_id = DWT->CYCCNT ^ ((uint32_t)HAL_GetTick() << 12) ^ 0x1F123BB5UL;
    if (boot_id == 0U) boot_id = V1_BOARD_ID;
}
