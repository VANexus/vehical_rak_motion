#include "app_car_safety.h"

#include <assert.h>
#include <stdio.h>

static void test_latched_inputs_are_rejected(void)
{
    assert(car_control_inputs_allowed(1U, 0U) == 0U);
    assert(car_control_inputs_allowed(0U, 1U) == 0U);
    assert(car_control_inputs_allowed(0U, 0U) != 0U);
}

static void test_estop_overrides_calibration_drive_and_kick(void)
{
    car_motor_output_t output = car_motor_output_gate(
        1U, 1U, 1U, 1U, 1, 8500);

    assert(output.action == CAR_MOTOR_OUTPUT_BRAKE);
    assert(output.reset_controller != 0U);
    assert(output.direction == 0);
    assert(output.duty_percent == 0U);

    output = car_motor_output_gate(1U, 0U, 1U, 1U, -1, 8500);
    assert(output.action == CAR_MOTOR_OUTPUT_BRAKE);
    assert(output.reset_controller != 0U);
    assert(output.duty_percent == 0U);
}

static void test_unlatched_modes_keep_existing_output_selection(void)
{
    car_motor_output_t output = car_motor_output_gate(
        0U, 1U, 0U, 0U, -1, 0);
    assert(output.action == CAR_MOTOR_OUTPUT_DRIVE);
    assert(output.direction == -1);
    assert(output.duty_percent == 55U);

    output = car_motor_output_gate(0U, 0U, 1U, 1U, 1, 0);
    assert(output.action == CAR_MOTOR_OUTPUT_DRIVE);
    assert(output.duty_percent == 75U);

    output = car_motor_output_gate(0U, 0U, 1U, 0U, 1, -4200);
    assert(output.action == CAR_MOTOR_OUTPUT_DRIVE);
    assert(output.direction == -1);
    assert(output.duty_percent == 42U);
}

static void test_clear_does_not_rearm_or_drive(void)
{
    car_motor_output_t output = car_motor_output_gate(
        0U, 0U, 0U, 0U, 1, 4200);
    assert(output.action == CAR_MOTOR_OUTPUT_BRAKE);
    assert(output.reset_controller == 0U);
    assert(output.duty_percent == 0U);
}

static void test_clear_requires_pd3_off_no_internal_fault_and_no_pending_estop(void)
{
    assert(car_safety_clear_allowed(1U, 1U, 0U) != 0U);
    assert(car_safety_clear_allowed(0U, 1U, 0U) == 0U);
    assert(car_safety_clear_allowed(1U, 0U, 0U) == 0U);
    assert(car_safety_clear_allowed(1U, 1U, 1U) == 0U);
}

int main(void)
{
    test_latched_inputs_are_rejected();
    test_estop_overrides_calibration_drive_and_kick();
    test_unlatched_modes_keep_existing_output_selection();
    test_clear_does_not_rearm_or_drive();
    test_clear_requires_pd3_off_no_internal_fault_and_no_pending_estop();
    puts("电机安全门策略 host 测试通过");
    return 0;
}
