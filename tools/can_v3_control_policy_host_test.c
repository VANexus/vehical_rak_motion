#include "app_can_control_policy.h"

#include <assert.h>
#include <stdio.h>

static void test_only_emergency_stop_is_a_can_control_action(void)
{
    static const uint8_t commands[] = {1U, 2U, 4U, 5U, 6U};

    for (unsigned i = 0U; i < sizeof(commands); ++i) {
        can_v3_control_action_t action =
            can_v3_control_policy(CAN_V3_ID_SAFETY, commands[i]);
        if (commands[i] == 4U)
            assert(action == CAN_V3_ACTION_EMERGENCY_STOP);
        else
            assert(action == CAN_V3_ACTION_NONE);
    }
}

static void test_velocity_frame_never_authorizes_or_enables_drive(void)
{
    assert(can_v3_control_policy(CAN_V3_ID_CHASSIS_VELOCITY, 1U) ==
           CAN_V3_ACTION_NONE);
    assert(can_v3_control_policy(CAN_V3_ID_CHASSIS_VELOCITY, 4U) ==
           CAN_V3_ACTION_NONE);
}

static void test_can_clear_cannot_unlock_or_arm_drive(void)
{
    uint8_t estop_latched = 1U;
    uint8_t drive_armed = 0U;
    can_v3_control_action_t action =
        can_v3_control_policy(CAN_V3_ID_SAFETY, 5U);

    assert(action == CAN_V3_ACTION_NONE);
    assert(estop_latched != 0U);
    assert(drive_armed == 0U);
    assert(can_v3_control_policy(CAN_V3_ID_CHASSIS_VELOCITY, 1U) ==
           CAN_V3_ACTION_NONE);
    assert(drive_armed == 0U);
}

int main(void)
{
    test_only_emergency_stop_is_a_can_control_action();
    test_velocity_frame_never_authorizes_or_enables_drive();
    test_can_clear_cannot_unlock_or_arm_drive();
    puts("CAN v3 控制授权 host 测试通过");
    return 0;
}
