#include "app_can_control_policy.h"

can_v3_control_action_t can_v3_control_policy(uint16_t can_id,
                                               uint8_t command)
{
    if (can_id == CAN_V3_ID_SAFETY && command == 4U)
        return CAN_V3_ACTION_EMERGENCY_STOP;
    return CAN_V3_ACTION_NONE;
}
