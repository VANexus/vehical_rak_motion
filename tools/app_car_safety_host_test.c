#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char *read_file(const char *path)
{
    FILE *file = fopen(path, "rb");
    assert(file != NULL);
    assert(fseek(file, 0L, SEEK_END) == 0);
    long length = ftell(file);
    assert(length >= 0L);
    assert(fseek(file, 0L, SEEK_SET) == 0);
    char *contents = malloc((size_t)length + 1U);
    assert(contents != NULL);
    assert(fread(contents, 1U, (size_t)length, file) == (size_t)length);
    contents[length] = '\0';
    assert(fclose(file) == 0);
    return contents;
}

static const char *function_body(const char *source, const char *name)
{
    const char *start = strstr(source, name);
    assert(start != NULL);
    start = strchr(start, '{');
    assert(start != NULL);
    return start;
}

static void test_supervisor_checks_safety_before_accepting_commands(void)
{
    char *source = read_file("Core/Src/app_car.c");
    const char *step = function_body(source, "void car_control_step(");
    const char *gate = strstr(step, "car_control_inputs_allowed(");
    const char *button = strstr(step, "button_pressed_event(now)");
    const char *blocked = strstr(step, "if (controls_allowed == 0U)");
    const char *short_press = strstr(step, "if (bev == 1)");
    const char *long_press = strstr(step, "else if (bev == 2)");
    const char *motor_command = strstr(step, "if (motor_command != 0U)");

    assert(gate != NULL);
    assert(button != NULL && gate < button);
    assert(blocked != NULL && button < blocked);
    assert(strstr(blocked, "motor_command = 0U;") != NULL);
    assert(strstr(blocked, "brake_all();") != NULL);
    assert(strstr(blocked, "return;") != NULL);
    assert(short_press != NULL && gate < short_press);
    assert(long_press != NULL && gate < long_press);
    assert(motor_command != NULL && gate < motor_command);
    free(source);
}

static void test_isr_routes_every_motor_output_through_safety_gate(void)
{
    char *source = read_file("Core/Src/app_car.c");
    const char *isr = function_body(source, "void car_control_isr(void)");
    const char *fast_request = strstr(isr, "if (car_estop_fast_req != 0U)");
    const char *latch = strstr(isr, "estop_latched = 1U;");
    const char *gate = strstr(isr, "car_motor_output_gate(");
    const char *brake = strstr(isr, "motor_brake(i)");
    const char *drive = strstr(isr, "motor_drive(i,");

    assert(fast_request != NULL);
    assert(latch != NULL && fast_request < latch);
    assert(gate != NULL && latch < gate);
    assert(gate != NULL);
    assert(brake != NULL && gate < brake);
    assert(drive != NULL && gate < drive);
    free(source);
}

static void test_only_accepted_usb_clear_reaches_car_latch(void)
{
    char *source = read_file("Core/Src/app_serial.c");
    const char *generation =
        strstr(source, "snapshot.safety_clear_generation !=");
    const char *clear = strstr(source, "car_estop_clear() == 0U");
    const char *reject = strstr(source, "serial_v2_session_reject_drive_clear(");

    assert(generation != NULL);
    assert(clear != NULL && generation < clear);
    assert(reject != NULL && clear < reject);
    free(source);
}

int main(void)
{
    test_supervisor_checks_safety_before_accepting_commands();
    test_isr_routes_every_motor_output_through_safety_gate();
    test_only_accepted_usb_clear_reaches_car_latch();
    puts("电机安全门 host 接线测试通过");
    return 0;
}
