#include "app_serial.h"

#include "app_car.h"
#include "app_imu.h"
#include "app_serial_session.h"
#include "app_wheel_control.h"
#include "main.h"
#include "stm32f4xx.h"
#include "usart.h"

#define V2_BOARD_ID       0x0407U
#define V2_FW_VERSION     ((1UL << 16) | (23UL << 8))

enum {
    SERIAL_PORTS = SERIAL_V2_PORT_COUNT,
    RXBUF_N = 512U
};

static serial_v2_session_t v2_session;
static uint8_t rxbuf[SERIAL_PORTS][RXBUF_N];
static volatile uint32_t rx_dma_wraps[SERIAL_PORTS];
static uint64_t rx_produced[SERIAL_PORTS];
static uint64_t rx_consumed[SERIAL_PORTS];
static volatile uint16_t rx_rearm_failures[SERIAL_PORTS];
static uint16_t rx_rearm_failures_reported[SERIAL_PORTS];
static volatile uint8_t rx_rearm_pending[SERIAL_PORTS];
static volatile uint8_t rx_resync[SERIAL_PORTS];
static uint8_t tx_active[SERIAL_PORTS][SERIAL_V2_TX_FRAME_MAX];
static uint16_t tx_active_len[SERIAL_PORTS];
static uint32_t safety_clear_generation;
static uint32_t safety_latch_generation;

static int serial_rx_rearm(unsigned port, UART_HandleTypeDef *uart)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(uart, rxbuf[port], RXBUF_N) != HAL_OK) {
        rx_rearm_pending[port] = 1U;
        if (rx_rearm_failures[port] != 0xFFFFU)
            rx_rearm_failures[port]++;
        return 0;
    }

    rx_dma_wraps[port] = 0U;
    rx_produced[port] = 0U;
    rx_consumed[port] = 0U;
    return 1;
}

static uint64_t serial_rx_dma_cursor(unsigned port, UART_HandleTypeDef *uart)
{
    uint32_t wraps_before = rx_dma_wraps[port];
    uint32_t remaining = __HAL_DMA_GET_COUNTER(uart->hdmarx);
    uint32_t wraps_after = rx_dma_wraps[port];

    if (wraps_before != wraps_after) {
        wraps_before = wraps_after;
        remaining = __HAL_DMA_GET_COUNTER(uart->hdmarx);
    }

    uint32_t position = (RXBUF_N - (remaining % RXBUF_N)) % RXBUF_N;
    uint64_t snapshot = (uint64_t)wraps_before * RXBUF_N + position;
    if (snapshot > rx_produced[port])
        rx_produced[port] = snapshot;
    return rx_produced[port];
}

static uint32_t serial_boot_id(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t id = DWT->CYCCNT ^ ((uint32_t)HAL_GetTick() << 12) ^
                  0x1F123BB5UL;
    return id != 0U ? id : V2_BOARD_ID;
}

static uint8_t serial_estop_clear_allowed(uint32_t battery_mv)
{
    return (uint8_t)(car_estop_clear_allowed() != 0U &&
                     battery_mv >= CAR_BATTERY_UV_THRESHOLD_MV);
}

void serial_init(void)
{
    serial_v2_session_init(&v2_session, serial_boot_id(), V2_BOARD_ID,
                           V2_FW_VERSION);
    safety_clear_generation = 0U;
    safety_latch_generation = 0U;
    for (unsigned port = 0U; port < SERIAL_PORTS; ++port) {
        rx_dma_wraps[port] = 0U;
        rx_resync[port] = 0U;
        rx_rearm_pending[port] = 1U;
        rx_rearm_failures[port] = 0U;
        rx_rearm_failures_reported[port] = 0U;
        tx_active_len[port] = 0U;
    }
}

void serial_rx_service(void)
{
    uint64_t now_us = car_uptime_us(HAL_GetTick());
    serial_v2_session_set_estop_clear_allowed(
        &v2_session, serial_estop_clear_allowed(car_battery_mv()));
    for (unsigned port = 0U; port < SERIAL_PORTS; ++port) {
        UART_HandleTypeDef *uart = port == 0U ? &huart1 : &huart3;
        if (rx_rearm_failures_reported[port] != rx_rearm_failures[port]) {
            rx_rearm_failures_reported[port]++;
            serial_v2_session_note_rx_overrun(&v2_session, port);
        }
        if (rx_rearm_pending[port] != 0U ||
            uart->RxState == HAL_UART_STATE_READY) {
            rx_rearm_pending[port] = 0U;
            (void)serial_rx_rearm(port, uart);
        }
        if (uart->hdmarx == NULL || uart->RxState != HAL_UART_STATE_BUSY_RX)
            continue;

        if (rx_resync[port] != 0U) {
            rx_consumed[port] = rx_produced[port];
            rx_resync[port] = 0U;
            serial_v2_session_note_rx_overrun(&v2_session, port);
        }

        uint64_t produced = serial_rx_dma_cursor(port, uart);
        if (produced - rx_consumed[port] >= RXBUF_N) {
            rx_consumed[port] = produced;
            serial_v2_session_note_rx_overrun(&v2_session, port);
            continue;
        }
        while (rx_consumed[port] != produced) {
            uint64_t latest = serial_rx_dma_cursor(port, uart);
            if (latest - rx_consumed[port] >= RXBUF_N) {
                rx_consumed[port] = latest;
                serial_v2_session_note_rx_overrun(&v2_session, port);
                break;
            }
            serial_v2_session_feed(&v2_session, port,
                                   &rxbuf[port][rx_consumed[port] % RXBUF_N],
                                   1U, now_us);
            rx_consumed[port]++;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    (void)size;
    unsigned port;
    if (uart == NULL)
        return;
    if (uart->Instance == USART1)
        port = 0U;
    else if (uart->Instance == USART3)
        port = 1U;
    else
        return;
    if (port >= SERIAL_PORTS)
        return;
    if (HAL_UARTEx_GetRxEventType(uart) == HAL_UART_RXEVENT_TC)
        rx_dma_wraps[port]++;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart->RxState != HAL_UART_STATE_READY)
        return;
    unsigned port;
    if (uart->Instance == USART1)
        port = 0U;
    else if (uart->Instance == USART3)
        port = 1U;
    else
        return;
    if (port >= SERIAL_PORTS)
        return;
    rx_resync[port] = 1U;
    rx_rearm_pending[port] = 1U;
}

void serial_poll_tx(void)
{
    for (unsigned port = 0U; port < SERIAL_PORTS; ++port) {
        UART_HandleTypeDef *uart = port == 0U ? &huart1 : &huart3;
        if (tx_active_len[port] != 0U) {
            if (uart->gState == HAL_UART_STATE_READY)
                tx_active_len[port] = 0U;
            else
                continue;
        }
        size_t length = 0U;
        if (!serial_v2_session_pop_tx(&v2_session, port, tx_active[port],
                                      sizeof(tx_active[port]), &length))
            continue;
        tx_active_len[port] = (uint16_t)length;
        if (HAL_UART_Transmit_DMA(uart, tx_active[port],
                                  tx_active_len[port]) != HAL_OK) {
            tx_active_len[port] = 0U;
            serial_v2_session_note_tx_drop(&v2_session);
        }
    }
}

void serial_v2_tick(uint32_t now_ms)
{
    serial_v2_snapshot_t snapshot;
    uint32_t external_faults = 0U;
    uint32_t battery_mv = car_battery_mv();
    proto_v2_drive_state_t feedback = {0};
    int64_t position_urad[WHEEL_COUNT];
    int32_t velocity_urad_s[WHEEL_COUNT];
    uint32_t encoder_epoch;
    uint64_t sample_us;
    uint8_t calibration_ready = car_wheel_calibration_ready();
    uint8_t imu_ready =
        (uint8_t)(car_control_ready() != 0U && imu_ok() &&
                  imu_sample_fresh(now_ms));
    uint8_t estop_clear_allowed = serial_estop_clear_allowed(battery_mv);

    car_encoder_snapshot(position_urad, velocity_urad_s,
                         &feedback.feedback_valid_mask, &encoder_epoch,
                         &sample_us);
    feedback.encoder_epoch = encoder_epoch;
    feedback.sample_time_us = sample_us;
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i) {
        feedback.wheel_position_urad[i] = position_urad[i];
        feedback.wheel_velocity_urad_s[i] = velocity_urad_s[i];
    }
    uint8_t feedback_fresh = (uint8_t)(
        calibration_ready != 0U &&
        feedback.feedback_valid_mask == 0x0FU &&
        wheel_control_sample_fresh_from_clock(
            HAL_GetTick, (uint32_t)(sample_us / 1000ULL),
            (uint8_t)(sample_us != 0U)));

    /* [AI] 轮半径/方向签核和有效新鲜编码器采样属于 Task 3，当前必须闭锁。 */
    serial_v2_session_set_drive_prerequisites(
        &v2_session, imu_ready, calibration_ready, feedback_fresh);
    serial_v2_session_set_estop_clear_allowed(
        &v2_session, estop_clear_allowed);
    serial_v2_session_set_drive_feedback(&v2_session, &feedback);
    serial_v2_session_set_arm_commissioned(&v2_session, 0U);
    serial_v2_session_snapshot(&v2_session, &snapshot);
    if (snapshot.safety_latch_generation != safety_latch_generation) {
        safety_latch_generation = snapshot.safety_latch_generation;
        car_estop_request_fast();
    }
    if (snapshot.safety_clear_generation != safety_clear_generation) {
        safety_clear_generation = snapshot.safety_clear_generation;
        if (estop_clear_allowed == 0U || car_estop_clear() == 0U)
            serial_v2_session_reject_drive_clear(&v2_session);
    }
    if (imu_ready == 0U || calibration_ready == 0U || feedback_fresh == 0U)
        external_faults |= 1UL << 5;
    if ((car_flags() & 2U) != 0U)
        external_faults |= 1UL << 0;
    if (car_mode() == 4U)
        external_faults |= 1UL << 5;
    if (battery_mv != 0U && battery_mv < CAR_BATTERY_UV_THRESHOLD_MV)
        external_faults |= 1UL << 4;
    serial_v2_session_set_external_faults(&v2_session, external_faults);
    serial_v2_session_set_diagnostics(
        &v2_session, battery_mv > 0xFFFFU ? 0xFFFFU : (uint16_t)battery_mv,
        0);
    serial_v2_session_tick(&v2_session, car_uptime_us(now_ms));
    serial_v2_session_snapshot(&v2_session, &snapshot);

    if (snapshot.handshake_complete) {
        car_wheel_command_urad(snapshot.wheel_target_urad_s,
                               snapshot.drive_armed, now_ms);
    }
}
