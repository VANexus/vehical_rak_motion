#ifndef APP_PROTOCOL_V2_H
#define APP_PROTOCOL_V2_H

#include <stddef.h>
#include <stdint.h>

#define PROTO_V2_VERSION       2U
#define PROTO_V2_MAX_PAYLOAD   128U
#define PROTO_V2_MAX_RAW       (6U + PROTO_V2_MAX_PAYLOAD + 2U)
#define PROTO_V2_MAX_WIRE      (PROTO_V2_MAX_RAW + 2U)

#define PROTO_V2_TYPE_HELLO             0x01U
#define PROTO_V2_TYPE_HELLO_ACK         0x02U
#define PROTO_V2_TYPE_TIME_SYNC_REQUEST 0x03U
#define PROTO_V2_TYPE_TIME_SYNC_RESPONSE 0x04U
#define PROTO_V2_TYPE_DRIVE_COMMAND     0x10U
#define PROTO_V2_TYPE_ARM_COMMAND       0x11U
#define PROTO_V2_TYPE_SAFETY_COMMAND    0x12U
#define PROTO_V2_TYPE_DRIVE_STATE       0x20U
#define PROTO_V2_TYPE_ARM_STATE         0x21U
#define PROTO_V2_TYPE_DIAGNOSTICS       0x22U
#define PROTO_V2_TYPE_FAULT_EVENT       0x23U
#define PROTO_V2_FEATURE_GRIPPER_SERVO  (1UL << 4)

typedef struct {
    uint8_t version;
    uint8_t type;
    uint16_t sequence;
    uint16_t payload_length;
    uint8_t payload[PROTO_V2_MAX_PAYLOAD];
} proto_v2_frame_t;

typedef struct {
    uint8_t mode;
    uint16_t lease_ms;
    uint8_t valid_mask;
    int32_t wheel_velocity_urad_s[4];
} proto_v2_drive_command_t;

typedef struct {
    uint8_t mode;
    uint16_t lease_ms;
    uint8_t axis_count;
    uint8_t valid_mask;
    int32_t position_urad[7];
} proto_v2_arm_command_t;

uint16_t proto_v2_crc16(const uint8_t *data, size_t length);
int proto_v2_encode(const proto_v2_frame_t *frame, uint8_t *wire,
                    size_t wire_capacity, size_t *wire_length);
int proto_v2_decode(const uint8_t *wire, size_t wire_length,
                    proto_v2_frame_t *frame);
int proto_v2_pack_drive(const proto_v2_drive_command_t *command,
                        uint8_t *payload, size_t capacity, size_t *length);
int proto_v2_unpack_drive(const uint8_t *payload, size_t length,
                          proto_v2_drive_command_t *command);
int proto_v2_pack_arm(const proto_v2_arm_command_t *command,
                      uint32_t accepted_features, uint8_t *payload,
                      size_t capacity, size_t *length);
int proto_v2_unpack_arm(const uint8_t *payload, size_t length,
                        uint32_t accepted_features, proto_v2_arm_command_t *command);

#endif
