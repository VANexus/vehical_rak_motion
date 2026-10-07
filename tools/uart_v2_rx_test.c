#include "app_protocol_v2.h"
#include "app_car_safety.h"
#include "app_serial.h"
#include "uart_v2_host_hal_stubs.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned test_rx_start_attempts;
static unsigned test_rx_start_failures;

HAL_StatusTypeDef __real_HAL_UARTEx_ReceiveToIdle_DMA(
    UART_HandleTypeDef *uart, uint8_t *data, uint16_t size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);

HAL_StatusTypeDef __wrap_HAL_UARTEx_ReceiveToIdle_DMA(
    UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    test_rx_start_attempts++;
    if (test_rx_start_failures != 0U) {
        test_rx_start_failures--;
        return HAL_ERROR;
    }
    return __real_HAL_UARTEx_ReceiveToIdle_DMA(uart, data, size);
}

static size_t encode_hello(uint8_t *wire, size_t capacity)
{
    proto_v2_hello_t hello = {
        .host_build_id = 0x10203040U,
        .requested_features = PROTO_V2_REQUIRED_FEATURES
    };
    proto_v2_frame_t frame = {
        .version = PROTO_V2_VERSION,
        .type = PROTO_V2_TYPE_HELLO,
        .sequence = 1U,
        .payload_length = 8U
    };
    size_t length = 0U;
    assert(proto_v2_pack_hello(&hello, frame.payload, sizeof(frame.payload),
                               &length));
    frame.payload_length = (uint16_t)length;
    assert(proto_v2_encode(&frame, wire, capacity, &length));
    return length;
}

static size_t encode_drive_command(uint16_t sequence, uint8_t *wire,
                                  size_t capacity)
{
    proto_v2_drive_command_t command = {
        .mode = 1U,
        .lease_ms = 100U,
        .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {500000, 500000, 500000, 500000}
    };
    proto_v2_frame_t frame = {
        .version = PROTO_V2_VERSION,
        .type = PROTO_V2_TYPE_DRIVE_COMMAND,
        .sequence = sequence
    };
    size_t payload_length = 0U;
    assert(proto_v2_pack_drive(&command, frame.payload,
                               sizeof(frame.payload), &payload_length));
    frame.payload_length = (uint16_t)payload_length;
    size_t wire_length = 0U;
    assert(proto_v2_encode(&frame, wire, capacity, &wire_length));
    return wire_length;
}

static size_t encode_disarmed_drive_command(uint16_t sequence, uint8_t *wire,
                                           size_t capacity)
{
    proto_v2_drive_command_t command = {
        .mode = 0U,
        .lease_ms = 100U,
        .valid_mask = 0U,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    proto_v2_frame_t frame = {
        .version = PROTO_V2_VERSION,
        .type = PROTO_V2_TYPE_DRIVE_COMMAND,
        .sequence = sequence
    };
    size_t payload_length = 0U;
    assert(proto_v2_pack_drive(&command, frame.payload,
                               sizeof(frame.payload), &payload_length));
    frame.payload_length = (uint16_t)payload_length;
    size_t wire_length = 0U;
    assert(proto_v2_encode(&frame, wire, capacity, &wire_length));
    return wire_length;
}

static size_t encode_safety_command(uint16_t sequence, uint8_t operation,
                                    uint8_t subsystem, uint8_t *wire,
                                    size_t capacity)
{
    proto_v2_safety_command_t command = {
        .operation = operation,
        .subsystem_mask = subsystem
    };
    proto_v2_frame_t frame = {
        .version = PROTO_V2_VERSION,
        .type = PROTO_V2_TYPE_SAFETY_COMMAND,
        .sequence = sequence
    };
    size_t payload_length = 0U;
    assert(proto_v2_pack_safety_command(&command, frame.payload,
                                        sizeof(frame.payload),
                                        &payload_length));
    frame.payload_length = (uint16_t)payload_length;
    size_t wire_length = 0U;
    assert(proto_v2_encode(&frame, wire, capacity, &wire_length));
    return wire_length;
}

static void feed_uart_frame_on(unsigned port, const uint8_t *wire,
                               size_t length)
{
    test_dma_write(port, wire, length, HAL_UART_RXEVENT_IDLE);
    serial_rx_service();
}

static void feed_uart_frame(const uint8_t *wire, size_t length)
{
    feed_uart_frame_on(0U, wire, length);
}

static uint16_t read_rx_overrun_count(void)
{
    uint16_t rx_overrun_count = 0U;
    int found_diagnostics = 0;

    serial_v2_tick(test_tick_ms);
    for (unsigned i = 0U; i < 4U; ++i)
        serial_poll_tx();
    for (size_t i = 0U; i < test_tx_count; ++i) {
        proto_v2_frame_t frame;
        if (!proto_v2_decode(test_tx_frames[i], test_tx_lengths[i], &frame))
            continue;
        if (frame.type == PROTO_V2_TYPE_DIAGNOSTICS) {
            proto_v2_diagnostics_t diagnostics;
            assert(proto_v2_unpack_diagnostics(frame.payload,
                                               frame.payload_length,
                                               &diagnostics));
            rx_overrun_count = diagnostics.rx_overrun_count;
            found_diagnostics = 1;
        }
    }
    assert(found_diagnostics);
    return rx_overrun_count;
}

static void test_stale_rx_event_sizes_do_not_advance_producer(void)
{
    uint8_t hello[TEST_V2_MAX_WIRE];
    uint8_t padding[512] = {0};
    uint8_t lap[512] = {0};
    size_t hello_length = encode_hello(hello, sizeof(hello));

    test_tick_ms = 0U;
    serial_init();
    serial_rx_service();
    test_dma_write(0U, hello, hello_length, HAL_UART_RXEVENT_IDLE);
    serial_rx_service();
    serial_poll_tx();
    test_hal_reset_tx();

    test_dma_write(0U, padding, 32U, HAL_UART_RXEVENT_IDLE);
    serial_rx_service();
    huart1.RxEventType = HAL_UART_RXEVENT_HT;
    HAL_UARTEx_RxEventCallback(&huart1, 256U);
    huart1.RxEventType = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart1, (uint16_t)hello_length);
    serial_rx_service();

    test_tick_ms = 1000U;
    assert(read_rx_overrun_count() == 0U);
    test_hal_reset_tx();

    size_t position = test_rx_capacity[0U] - huart1.hdmarx->counter;
    size_t to_boundary = test_rx_capacity[0U] - position;
    test_dma_write(0U, padding, to_boundary, HAL_UART_RXEVENT_TC);
    serial_rx_service();
    test_dma_write(0U, lap, sizeof(lap), HAL_UART_RXEVENT_TC);
    serial_rx_service();
    huart1.RxEventType = HAL_UART_RXEVENT_HT;
    HAL_UARTEx_RxEventCallback(&huart1, 256U);
    huart1.RxEventType = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart1, 0U);
    serial_rx_service();

    test_tick_ms = 2000U;
    assert(read_rx_overrun_count() == 1U);
}

static void test_failed_rx_rearms_are_retried_from_service(void)
{
    uint8_t hello[TEST_V2_MAX_WIRE];
    size_t hello_length = encode_hello(hello, sizeof(hello));
    unsigned attempts_before = test_rx_start_attempts;

    test_tick_ms = 0U;
    serial_init();
    assert(test_rx_start_attempts == attempts_before);
    assert(huart1.RxState == HAL_UART_STATE_READY);
    assert(huart3.RxState == HAL_UART_STATE_READY);
    serial_rx_service();
    assert(test_rx_start_attempts == attempts_before + 2U);
    assert(huart1.RxState == HAL_UART_STATE_BUSY_RX);

    test_dma_write(1U, hello, hello_length, HAL_UART_RXEVENT_IDLE);
    serial_rx_service();
    serial_poll_tx();
    test_hal_reset_tx();

    serial_rx_service();
    assert(test_rx_start_attempts == attempts_before + 2U);

    huart1.RxState = HAL_UART_STATE_READY;
    HAL_UART_ErrorCallback(&huart1);
    assert(test_rx_start_attempts == attempts_before + 2U);
    assert(huart1.RxState == HAL_UART_STATE_READY);

    serial_rx_service();
    assert(test_rx_start_attempts == attempts_before + 3U);
    assert(huart1.RxState == HAL_UART_STATE_BUSY_RX);

    huart1.RxState = HAL_UART_STATE_READY;
    test_rx_start_failures = 1U;
    HAL_UART_ErrorCallback(&huart1);
    assert(test_rx_start_attempts == attempts_before + 3U);
    assert(huart1.RxState == HAL_UART_STATE_READY);

    serial_rx_service();
    assert(test_rx_start_attempts == attempts_before + 4U);
    assert(huart1.RxState == HAL_UART_STATE_READY);

    serial_rx_service();
    assert(test_rx_start_attempts == attempts_before + 5U);
    assert(huart1.RxState == HAL_UART_STATE_BUSY_RX);

    test_tick_ms = 1000U;
    assert(read_rx_overrun_count() >= 2U);
}

static void test_full_dma_lap_with_unchanged_ndtr_is_reported(void)
{
    uint8_t hello[TEST_V2_MAX_WIRE];
    uint8_t lap[512] = {0};
    uint8_t padding[512] = {0};
    size_t hello_length = encode_hello(hello, sizeof(hello));
    proto_v2_diagnostics_t diagnostics;
    int found_diagnostics = 0;

    serial_init();
    serial_rx_service();
    test_dma_write(0U, hello, hello_length, HAL_UART_RXEVENT_IDLE);
    serial_rx_service();
    serial_poll_tx();
    test_hal_reset_tx();

    test_dma_write(0U, padding, sizeof(padding) - hello_length,
                   HAL_UART_RXEVENT_TC);
    serial_rx_service();
    test_dma_write(0U, lap, sizeof(lap), HAL_UART_RXEVENT_TC);
    serial_rx_service();
    test_tick_ms = 1000U;
    serial_v2_tick(test_tick_ms);
    for (unsigned i = 0U; i < 4U; ++i)
        serial_poll_tx();

    for (size_t i = 0U; i < test_tx_count; ++i) {
        proto_v2_frame_t frame;
        assert(proto_v2_decode(test_tx_frames[i], test_tx_lengths[i], &frame));
        if (frame.type == PROTO_V2_TYPE_DIAGNOSTICS) {
            assert(proto_v2_unpack_diagnostics(frame.payload,
                                               frame.payload_length,
                                               &diagnostics));
            found_diagnostics = 1;
        }
    }
    assert(found_diagnostics);
    assert(diagnostics.rx_overrun_count != 0U);
}

static void test_only_valid_drive_clear_reaches_car_estop_latch(void)
{
    uint8_t hello[TEST_V2_MAX_WIRE];
    uint8_t safety[TEST_V2_MAX_WIRE];
    size_t hello_length = encode_hello(hello, sizeof(hello));
    size_t safety_length;

    test_car_estop_clear_count = 0U;
    test_car_wheel_command_enable = 0U;
    test_car_wheel_command_count = 0U;
    test_car_battery_mv = 12000U;
    test_car_mode = 0U;
    test_car_flags = 1UL << 1;
    serial_init();
    serial_rx_service();
    feed_uart_frame(hello, hello_length);
    serial_v2_tick(1U);

    safety_length = encode_safety_command(
        2U, PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE,
        safety, sizeof(safety));
    feed_uart_frame_on(1U, safety, safety_length);
    serial_v2_tick(2U);
    assert(test_car_estop_clear_count == 0U);

    test_car_mode = 4U;
    feed_uart_frame(safety, safety_length);
    serial_v2_tick(3U);
    assert(test_car_estop_clear_count == 0U);
    assert((test_car_flags & (1UL << 1)) != 0U);

    test_car_mode = 0U;
    test_car_battery_mv = 9000U;
    safety_length = encode_safety_command(
        3U, PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE,
        safety, sizeof(safety));
    feed_uart_frame(safety, safety_length);
    serial_v2_tick(4U);
    assert(test_car_estop_clear_count == 0U);
    assert((test_car_flags & (1UL << 1)) != 0U);

    test_car_battery_mv = 12000U;
    test_car_enable_switch = 0U;
    test_motor_command = 3U;
    test_estop_latched = 1U;
    test_remote_running = 1U;
    test_bench_running = 1U;
    test_bench_stop_requested = 1U;
    test_calibrating = 1U;
    test_wheel_targets[0] = 100;
    test_wheel_targets[1] = -200;
    test_wheel_targets[2] = 300;
    test_wheel_targets[3] = -400;
    test_wheel_ramps[0] = 10;
    test_wheel_ramps[1] = -20;
    test_wheel_ramps[2] = 30;
    test_wheel_ramps[3] = -40;
    test_duty_q[0] = 1000;
    test_duty_q[1] = -2000;
    test_duty_q[2] = 3000;
    test_duty_q[3] = -4000;
    safety_length = encode_safety_command(
        4U, PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE,
        safety, sizeof(safety));
    feed_uart_frame(safety, safety_length);
    serial_v2_tick(5U);
    assert(test_car_estop_clear_count == 1U);
    assert((test_car_flags & (1UL << 1)) == 0U);
    assert(test_car_wheel_command_enable == 0U);
    car_motor_output_t output = car_motor_output_gate(
        test_estop_latched, test_calibrating, 0U, 1U, 1, 0);
    assert(output.action == CAR_MOTOR_OUTPUT_BRAKE);
    assert(output.duty_percent == 0U);
    assert(test_motor_command == 0U);
    assert(test_estop_latched == 0U);
    assert(test_remote_running == 0U);
    assert(test_bench_running == 0U);
    assert(test_bench_stop_requested == 0U);
    assert(test_calibrating == 0U);
    for (unsigned i = 0U; i < 4U; ++i) {
        assert(test_wheel_targets[i] == 0);
        assert(test_wheel_ramps[i] == 0);
        assert(test_duty_q[i] == 0);
    }

    feed_uart_frame(safety, safety_length);
    serial_v2_tick(6U);
    assert(test_car_estop_clear_count == 1U);

    safety_length = encode_safety_command(
        5U, PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_ARM,
        safety, sizeof(safety));
    feed_uart_frame(safety, safety_length);
    serial_v2_tick(7U);
    assert(test_car_estop_clear_count == 1U);
    assert(test_car_wheel_command_enable == 0U);
}

static void test_only_valid_owner_drive_latch_reaches_fast_estop(void)
{
    uint8_t hello[TEST_V2_MAX_WIRE];
    uint8_t safety[TEST_V2_MAX_WIRE];
    size_t hello_length = encode_hello(hello, sizeof(hello));
    size_t safety_length;

    test_car_estop_request_count = 0U;
    test_car_flags = 0U;
    serial_init();
    serial_rx_service();
    feed_uart_frame(hello, hello_length);
    serial_v2_tick(1U);

    safety_length = encode_safety_command(
        2U, PROTO_V2_SAFETY_LATCH_INHIBIT, PROTO_V2_SUBSYSTEM_ARM,
        safety, sizeof(safety));
    feed_uart_frame(safety, safety_length);
    serial_v2_tick(2U);
    assert(test_car_estop_request_count == 0U);

    safety_length = encode_safety_command(
        3U, PROTO_V2_SAFETY_LATCH_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE,
        safety, sizeof(safety));
    safety[safety_length - 2U] ^= 0x01U;
    feed_uart_frame(safety, safety_length);
    serial_v2_tick(3U);
    assert(test_car_estop_request_count == 0U);

    safety_length = encode_safety_command(
        3U, PROTO_V2_SAFETY_LATCH_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE,
        safety, sizeof(safety));
    feed_uart_frame_on(1U, safety, safety_length);
    serial_v2_tick(4U);
    assert(test_car_estop_request_count == 0U);

    feed_uart_frame(safety, safety_length);
    serial_v2_tick(5U);
    assert(test_car_estop_request_count == 1U);

    feed_uart_frame(safety, safety_length);
    serial_v2_tick(6U);
    assert(test_car_estop_request_count == 1U);

    safety_length = encode_safety_command(
        4U, PROTO_V2_SAFETY_STOP, PROTO_V2_SUBSYSTEM_DRIVE,
        safety, sizeof(safety));
    feed_uart_frame(safety, safety_length);
    serial_v2_tick(7U);
    assert(test_car_estop_request_count == 1U);
}

static void test_pd3_on_clear_is_rejected(void)
{
    uint8_t hello[TEST_V2_MAX_WIRE];
    uint8_t safety[TEST_V2_MAX_WIRE];
    size_t hello_length = encode_hello(hello, sizeof(hello));
    size_t safety_length = encode_safety_command(
        2U, PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE,
        safety, sizeof(safety));

    test_car_estop_clear_count = 0U;
    test_car_enable_switch = 1U;
    test_car_battery_mv = 12000U;
    test_car_mode = 0U;
    test_car_flags = 1UL << 1;
    serial_init();
    serial_rx_service();
    feed_uart_frame(hello, hello_length);
    serial_v2_tick(1U);
    feed_uart_frame(safety, safety_length);
    serial_v2_tick(2U);

    assert(test_car_estop_clear_count == 0U);
    assert((test_car_flags & (1UL << 1)) != 0U);
}

static void test_clear_and_armed_drive_in_one_rx_batch_stays_disarmed(void)
{
    uint8_t hello[TEST_V2_MAX_WIRE];
    uint8_t clear[TEST_V2_MAX_WIRE];
    uint8_t armed[TEST_V2_MAX_WIRE];
    uint8_t disarmed[TEST_V2_MAX_WIRE];
    uint8_t batch[TEST_V2_MAX_WIRE * 2U];
    size_t hello_length = encode_hello(hello, sizeof(hello));
    size_t clear_length = encode_safety_command(
        2U, PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE,
        clear, sizeof(clear));
    size_t armed_length = encode_drive_command(3U, armed, sizeof(armed));
    size_t disarmed_length;

    test_tick_ms = 100U;
    test_imu_fresh = 1U;
    test_wheel_calibration_ready = 1U;
    test_encoder_valid_mask = 0x0FU;
    test_encoder_sample_us = 100000U;
    test_car_enable_switch = 0U;
    test_car_battery_mv = 12000U;
    test_car_mode = 0U;
    test_car_flags = 1UL << 1;
    test_car_estop_clear_count = 0U;
    test_car_wheel_command_enable = 0U;

    serial_init();
    serial_rx_service();
    feed_uart_frame(hello, hello_length);
    serial_v2_tick(100U);
    test_car_flags = 0U;
    serial_v2_tick(101U);

    memcpy(batch, clear, clear_length);
    memcpy(batch + clear_length, armed, armed_length);
    test_dma_write(0U, batch, clear_length + armed_length,
                   HAL_UART_RXEVENT_IDLE);
    serial_rx_service();
    serial_v2_tick(102U);
    assert(test_car_estop_clear_count == 1U);
    assert(test_car_wheel_command_enable == 0U);

    disarmed_length = encode_disarmed_drive_command(
        4U, disarmed, sizeof(disarmed));
    feed_uart_frame(disarmed, disarmed_length);
    serial_v2_tick(103U);
    assert(test_car_wheel_command_enable == 0U);

    armed_length = encode_drive_command(5U, armed, sizeof(armed));
    feed_uart_frame(armed, armed_length);
    serial_v2_tick(104U);
    assert(test_car_wheel_command_enable == 1U);
}

static void test_drive_gate_revokes_on_stale_imu_without_auto_rearm(void)
{
    uint8_t hello[TEST_V2_MAX_WIRE];
    uint8_t drive[TEST_V2_MAX_WIRE];
    size_t hello_length = encode_hello(hello, sizeof(hello));
    size_t drive_length = encode_drive_command(2U, drive, sizeof(drive));

    test_tick_ms = 100U;
    test_imu_fresh = 1U;
    test_wheel_calibration_ready = 0U;
    test_encoder_valid_mask = 0x0FU;
    test_encoder_position_urad[0] = 101;
    test_encoder_position_urad[1] = -202;
    test_encoder_position_urad[2] = 303;
    test_encoder_position_urad[3] = -404;
    test_encoder_velocity_urad_s[0] = 1001;
    test_encoder_velocity_urad_s[1] = -2002;
    test_encoder_velocity_urad_s[2] = 3003;
    test_encoder_velocity_urad_s[3] = -4004;
    test_encoder_epoch = 7U;
    test_encoder_sample_us = 100000U;
    test_car_wheel_command_enable = 0U;
    test_car_wheel_command_count = 0U;
    test_hal_reset_tx();

    serial_init();
    serial_rx_service();
    feed_uart_frame(hello, hello_length);
    serial_v2_tick(100U);
    feed_uart_frame(drive, drive_length);
    serial_v2_tick(102U);
    assert(test_car_wheel_command_enable == 0U);

    test_wheel_calibration_ready = 1U;
    serial_v2_tick(103U);
    assert(test_car_wheel_command_enable == 0U);

    drive_length = encode_drive_command(3U, drive, sizeof(drive));
    feed_uart_frame(drive, drive_length);
    serial_v2_tick(105U);
    assert(test_car_wheel_command_enable == 1U);

    test_imu_fresh = 0U;
    serial_v2_tick(110U);
    assert(test_car_wheel_command_enable == 0U);
    for (unsigned i = 0U; i < 8U; ++i)
        serial_poll_tx();

    int drive_state_found = 0;
    for (size_t i = 0U; i < test_tx_count; ++i) {
        proto_v2_frame_t frame;
        if (!proto_v2_decode(test_tx_frames[i], test_tx_lengths[i], &frame) ||
            frame.type != PROTO_V2_TYPE_DRIVE_STATE)
            continue;
        proto_v2_drive_state_t state;
        assert(proto_v2_unpack_drive_state(frame.payload, frame.payload_length,
                                           &state));
        assert(state.feedback_valid_mask == 0x0FU);
        assert(state.encoder_epoch == 7U);
        assert(state.sample_time_us == 100000U);
        assert(state.wheel_position_urad[0] == 101);
        assert(state.wheel_position_urad[3] == -404);
        assert(state.wheel_velocity_urad_s[1] == -2002);
        assert(state.wheel_velocity_urad_s[2] == 3003);
        assert((state.fault_bitmap & (1UL << 5)) != 0U);
        drive_state_found = 1;
    }
    assert(drive_state_found);

    test_imu_fresh = 1U;
    serial_v2_tick(111U);
    assert(test_car_wheel_command_enable == 0U);

    drive_length = encode_drive_command(4U, drive, sizeof(drive));
    feed_uart_frame(drive, drive_length);
    serial_v2_tick(112U);
    assert(test_car_wheel_command_enable == 1U);
}

int main(void)
{
    test_failed_rx_rearms_are_retried_from_service();
    test_stale_rx_event_sizes_do_not_advance_producer();
    test_full_dma_lap_with_unchanged_ndtr_is_reported();
    test_pd3_on_clear_is_rejected();
    test_only_valid_drive_clear_reaches_car_estop_latch();
    test_only_valid_owner_drive_latch_reaches_fast_estop();
    test_clear_and_armed_drive_in_one_rx_batch_stays_disarmed();
    test_drive_gate_revokes_on_stale_imu_without_auto_rearm();
    puts("UART v2 RX 测试通过");
    return 0;
}
