#include "app_car_safety.h"

#include <assert.h>
#include <stdio.h>

static void test_clear_during_calibration_does_not_restore_motor_output(void)
{
    volatile uint32_t motor_command = 3U;
    uint8_t estop_latched = 1U;
    uint8_t remote_running = 1U;
    uint8_t bench_running = 1U;
    uint8_t bench_stop_requested = 0U;
    uint8_t calibrating = 1U;
    volatile int32_t targets[4] = {100, 200, 300, 400};
    int32_t ramps[4] = {10, 20, 30, 40};
    volatile int32_t duty_q[4] = {1000, 2000, 3000, 4000};
    car_motor_output_t output = car_motor_output_gate(
        estop_latched, calibrating, 1U, 1U, 1, 7500);

    assert(output.action == CAR_MOTOR_OUTPUT_BRAKE);
    assert(output.duty_percent == 0U);

    car_safety_clear_pending(
        &motor_command, &estop_latched, &remote_running, &bench_running,
        &bench_stop_requested, &calibrating, targets, ramps, duty_q, 4U);

    assert(motor_command == 0U);
    assert(estop_latched == 0U);
    assert(remote_running == 0U);
    assert(bench_running == 0U);
    assert(bench_stop_requested == 0U);
    assert(calibrating == 0U);
    for (unsigned i = 0U; i < 4U; ++i) {
        assert(targets[i] == 0);
        assert(ramps[i] == 0);
        assert(duty_q[i] == 0);
    }

    output = car_motor_output_gate(estop_latched, calibrating, 0U, 1U,
                                   1, 0);
    assert(output.action == CAR_MOTOR_OUTPUT_BRAKE);
    assert(output.duty_percent == 0U);
}

int main(void)
{
    test_clear_during_calibration_does_not_restore_motor_output();
    puts("急停清锁竞争 host 测试通过");
    return 0;
}
