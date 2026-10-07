#ifndef APP_CAR_SAFETY_H
#define APP_CAR_SAFETY_H

#include <stdint.h>

typedef enum {
    CAR_MOTOR_OUTPUT_BRAKE = 0,
    CAR_MOTOR_OUTPUT_DRIVE
} car_motor_output_action_t;

typedef struct {
    car_motor_output_action_t action;
    uint8_t reset_controller;
    int8_t direction;
    uint8_t duty_percent;
} car_motor_output_t;

static inline uint8_t car_control_inputs_allowed(uint8_t estop_latched,
                                                  uint8_t estop_pending)
{
    return (uint8_t)(estop_latched == 0U && estop_pending == 0U);
}

static inline uint8_t car_safety_clear_allowed(uint8_t pd3_off,
                                                uint8_t internal_faults_clear,
                                                uint8_t estop_pending)
{
    return (uint8_t)(pd3_off != 0U && internal_faults_clear != 0U &&
                     estop_pending == 0U);
}

static inline void car_safety_clear_pending(
    volatile uint32_t *motor_command, volatile uint8_t *estop_latched,
    uint8_t *remote_running, uint8_t *bench_running,
    uint8_t *bench_stop_requested, uint8_t *calibrating,
    volatile int32_t *targets, int32_t *ramps, volatile int32_t *duty_q,
    uint8_t motor_count)
{
    *motor_command = 0U;
    *estop_latched = 0U;
    *remote_running = 0U;
    *bench_running = 0U;
    *bench_stop_requested = 0U;
    *calibrating = 0U;
    for (uint8_t i = 0U; i < motor_count; ++i) {
        targets[i] = 0;
        ramps[i] = 0;
        duty_q[i] = 0;
    }
}

static inline car_motor_output_t car_motor_output_gate(
    uint8_t estop_latched, uint8_t calibrating, uint8_t drive_phase,
    uint8_t encoder_uninitialized, int8_t forward_direction,
    int32_t duty_q)
{
    car_motor_output_t output = {
        .action = CAR_MOTOR_OUTPUT_BRAKE,
        .reset_controller = 0U,
        .direction = 0,
        .duty_percent = 0U
    };

    if (estop_latched != 0U) {
        output.reset_controller = 1U;
        return output;
    }
    if (calibrating != 0U) {
        output.action = CAR_MOTOR_OUTPUT_DRIVE;
        output.direction = forward_direction;
        output.duty_percent = 55U;
        return output;
    }
    if (drive_phase == 0U) return output;
    if (encoder_uninitialized != 0U) {
        output.action = CAR_MOTOR_OUTPUT_DRIVE;
        output.direction = forward_direction;
        output.duty_percent = 75U;
        return output;
    }

    output.action = CAR_MOTOR_OUTPUT_DRIVE;
    output.direction = forward_direction;
    if (duty_q < 0) {
        output.direction = (int8_t)-output.direction;
        duty_q = -duty_q;
    }
    uint32_t duty_percent = (uint32_t)duty_q / 100U;
    if (duty_percent > 85U) duty_percent = 85U;
    output.duty_percent = (uint8_t)duty_percent;
    return output;
}

#endif
