#ifndef APP_CAN_CONTROL_POLICY_H
#define APP_CAN_CONTROL_POLICY_H

#include <stdint.h>

#define CAN_V3_ID_SAFETY 0x010U
#define CAN_V3_ID_CHASSIS_VELOCITY 0x100U

typedef enum {
    CAN_V3_ACTION_NONE = 0,
    CAN_V3_ACTION_EMERGENCY_STOP = 1
} can_v3_control_action_t;

can_v3_control_action_t can_v3_control_policy(uint16_t can_id,
                                               uint8_t command);

#endif
