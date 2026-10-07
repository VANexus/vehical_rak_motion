#include "uart_v2_host_hal_stubs.h"
#include "app_car_safety.h"

#include <string.h>

CoreDebug_Type test_core_debug;
DWT_Type test_dwt;
uint32_t test_tick_ms;
uint32_t test_car_battery_mv = 12000U;
uint8_t test_car_mode;
uint32_t test_car_flags;
static DMA_HandleTypeDef test_dma[2];
UART_HandleTypeDef huart1 = {
    .Instance = USART1, .hdmarx = &test_dma[0],
    .RxState = HAL_UART_STATE_READY, .gState = HAL_UART_STATE_READY
};
UART_HandleTypeDef huart3 = {
    .Instance = USART3, .hdmarx = &test_dma[1],
    .RxState = HAL_UART_STATE_READY, .gState = HAL_UART_STATE_READY
};
uint8_t *test_rx_buffer[2];
size_t test_rx_capacity[2];
uint8_t test_tx_frames[32][TEST_V2_MAX_WIRE];
size_t test_tx_lengths[32];
size_t test_tx_count;
volatile uint8_t imu_dbg_state;
uint8_t test_imu_fresh;
uint8_t test_wheel_calibration_ready;
uint8_t test_encoder_valid_mask;
int64_t test_encoder_position_urad[4];
int32_t test_encoder_velocity_urad_s[4];
uint32_t test_encoder_epoch;
uint64_t test_encoder_sample_us;
uint8_t test_car_wheel_command_enable;
uint32_t test_car_wheel_command_count;
uint32_t test_car_estop_clear_count;
uint32_t test_car_estop_request_count;
uint8_t test_car_enable_switch;
volatile uint32_t test_motor_command;
uint8_t test_estop_latched = 1U;
uint8_t test_remote_running;
uint8_t test_bench_running;
uint8_t test_bench_stop_requested;
uint8_t test_calibrating;
volatile int32_t test_wheel_targets[4];
int32_t test_wheel_ramps[4];
volatile int32_t test_duty_q[4];

uint32_t HAL_GetTick(void)
{
    return test_tick_ms;
}

uint64_t car_uptime_us(uint32_t tick_ms)
{
    return (uint64_t)tick_ms * 1000ULL;
}

uint8_t car_estop_clear(void)
{
    test_car_estop_clear_count++;
    car_safety_clear_pending(
        &test_motor_command, &test_estop_latched, &test_remote_running,
        &test_bench_running, &test_bench_stop_requested, &test_calibrating,
        test_wheel_targets, test_wheel_ramps, test_duty_q, 4U);
    test_car_flags &= ~(1UL << 1);
    test_car_wheel_command_enable = 0U;
    return 1U;
}

void car_estop_request_fast(void)
{
    test_car_estop_request_count++;
}

uint8_t car_estop_clear_allowed(void)
{
    return (uint8_t)(test_car_enable_switch == 0U && test_car_mode != 4U);
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                               uint8_t *data, uint16_t size)
{
    unsigned port = uart->Instance == USART1 ? 0U : 1U;
    test_rx_buffer[port] = data;
    test_rx_capacity[port] = size;
    uart->hdmarx->counter = size;
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,
                                        uint8_t *data, uint16_t size)
{
    (void)uart;
    if (test_tx_count < 32U && size <= TEST_V2_MAX_WIRE) {
        memcpy(test_tx_frames[test_tx_count], data, size);
        test_tx_lengths[test_tx_count++] = size;
    }
    return HAL_OK;
}

__attribute__((weak))
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    (void)uart;
    (void)size;
}

uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart)
{
    return uart->RxEventType;
}

void test_dma_write(unsigned port, const uint8_t *data, size_t length,
                    uint32_t event_type)
{
    UART_HandleTypeDef *uart = port == 0U ? &huart1 : &huart3;
    size_t position = test_rx_capacity[port] - uart->hdmarx->counter;
    for (size_t i = 0U; i < length; ++i) {
        test_rx_buffer[port][position] = data[i];
        position = (position + 1U) % test_rx_capacity[port];
    }
    uart->hdmarx->counter =
        (uint32_t)(test_rx_capacity[port] -
                   (position % test_rx_capacity[port]));
    uart->RxEventType = event_type;
    uint16_t size = event_type == HAL_UART_RXEVENT_HT
        ? (uint16_t)(test_rx_capacity[port] / 2U)
        : event_type == HAL_UART_RXEVENT_TC
            ? (uint16_t)test_rx_capacity[port] : (uint16_t)position;
    HAL_UARTEx_RxEventCallback(uart, size);
}

void test_hal_reset_tx(void)
{
    test_tx_count = 0U;
    memset(test_tx_lengths, 0, sizeof(test_tx_lengths));
}

uint32_t car_battery_mv(void)
{
    return test_car_battery_mv;
}

uint8_t car_control_ready(void)
{
    return 1U;
}

uint32_t car_flags(void)
{
    return test_car_flags;
}

uint8_t car_mode(void)
{
    return test_car_mode;
}

void car_wheel_command_urad(const int32_t wheel_urad_s[4], uint8_t enable,
                            uint32_t now)
{
    (void)wheel_urad_s;
    (void)now;
    test_car_wheel_command_enable = enable;
    test_car_wheel_command_count++;
}

uint8_t car_wheel_calibration_ready(void)
{
    return test_wheel_calibration_ready;
}

void car_encoder_snapshot(int64_t position_urad[4],
                          int32_t velocity_urad_s[4],
                          uint8_t *valid_mask, uint32_t *epoch,
                          uint64_t *sample_us)
{
    memcpy(position_urad, test_encoder_position_urad,
           sizeof(test_encoder_position_urad));
    memcpy(velocity_urad_s, test_encoder_velocity_urad_s,
           sizeof(test_encoder_velocity_urad_s));
    *valid_mask = test_encoder_valid_mask;
    *epoch = test_encoder_epoch;
    *sample_us = test_encoder_sample_us;
}

int imu_ok(void)
{
    return 1;
}

int imu_sample_fresh(uint32_t now_ms)
{
    (void)now_ms;
    return test_imu_fresh != 0U;
}
