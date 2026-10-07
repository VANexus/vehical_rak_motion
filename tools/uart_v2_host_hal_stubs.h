#ifndef UART_V2_HOST_HAL_STUBS_H
#define UART_V2_HOST_HAL_STUBS_H

#define APP_CAR_H
#define APP_IMU_H
#define __MAIN_H
#define __USART_H__
#define __STM32F4xx_H
#define CAR_BATTERY_UV_THRESHOLD_MV 10500U

#include <stddef.h>
#include <stdint.h>

#define TEST_V2_MAX_WIRE 138U
typedef struct {
    uint32_t DEMCR;
} CoreDebug_Type;

typedef struct {
    uint32_t CTRL;
    uint32_t CYCCNT;
} DWT_Type;

extern CoreDebug_Type test_core_debug;
extern DWT_Type test_dwt;
extern uint32_t test_tick_ms;
extern uint32_t test_car_battery_mv;
extern uint8_t test_car_mode;
extern uint32_t test_car_flags;

#define CoreDebug (&test_core_debug)
#define DWT (&test_dwt)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1UL

typedef struct {
    uint32_t counter;
} DMA_HandleTypeDef;

typedef struct {
    void *Instance;
    DMA_HandleTypeDef *hdmarx;
    uint32_t RxState;
    uint32_t gState;
    uint32_t RxEventType;
} UART_HandleTypeDef;

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_RX 1U
#define HAL_UART_STATE_BUSY_TX 2U
#define HAL_UART_RXEVENT_TC 0U
#define HAL_UART_RXEVENT_HT 1U
#define HAL_UART_RXEVENT_IDLE 2U
#define __HAL_DMA_GET_COUNTER(hdma) ((hdma)->counter)
#define USART1 ((void *)(uintptr_t)1U)
#define USART3 ((void *)(uintptr_t)3U)

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern uint8_t *test_rx_buffer[2];
extern size_t test_rx_capacity[2];
extern uint8_t test_tx_frames[32][TEST_V2_MAX_WIRE];
extern size_t test_tx_lengths[32];
extern size_t test_tx_count;
extern uint8_t test_imu_fresh;
extern uint8_t test_wheel_calibration_ready;
extern uint8_t test_encoder_valid_mask;
extern int64_t test_encoder_position_urad[4];
extern int32_t test_encoder_velocity_urad_s[4];
extern uint32_t test_encoder_epoch;
extern uint64_t test_encoder_sample_us;
extern uint8_t test_car_wheel_command_enable;
extern uint32_t test_car_wheel_command_count;
extern uint32_t test_car_estop_clear_count;
extern uint32_t test_car_estop_request_count;
extern uint8_t test_car_enable_switch;
extern volatile uint32_t test_motor_command;
extern uint8_t test_estop_latched;
extern uint8_t test_remote_running;
extern uint8_t test_bench_running;
extern uint8_t test_bench_stop_requested;
extern uint8_t test_calibrating;
extern volatile int32_t test_wheel_targets[4];
extern int32_t test_wheel_ramps[4];
extern volatile int32_t test_duty_q[4];

uint32_t HAL_GetTick(void);
uint64_t car_uptime_us(uint32_t tick_ms);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                               uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,
                                        uint8_t *data, uint16_t size);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart);
void test_dma_write(unsigned port, const uint8_t *data, size_t length,
                    uint32_t event_type);
void test_hal_reset_tx(void);

uint32_t car_battery_mv(void);
uint8_t car_control_ready(void);
uint8_t car_estop_clear_allowed(void);
uint32_t car_flags(void);
uint8_t car_mode(void);
void car_wheel_command_urad(const int32_t wheel_urad_s[4], uint8_t enable,
                            uint32_t now);
uint8_t car_estop_clear(void);
void car_estop_request_fast(void);
uint8_t car_wheel_calibration_ready(void);
void car_encoder_snapshot(int64_t position_urad[4],
                          int32_t velocity_urad_s[4],
                          uint8_t *valid_mask, uint32_t *epoch,
                          uint64_t *sample_us);
int imu_ok(void);
int imu_sample_fresh(uint32_t now_ms);
extern volatile uint8_t imu_dbg_state;

#endif
