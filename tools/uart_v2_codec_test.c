#include "app_protocol_v2.h"

#include <assert.h>
#include <string.h>

int main(void)
{
    static const uint8_t crc_input[] = "123456789";
    static const uint8_t expected_drive_payload[20] = {
        0x01, 0x64, 0x00, 0x0F, 0x40, 0x42, 0x0F, 0x00, 0x80, 0x7B,
        0xE1, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x20, 0xA1, 0x07, 0x00
    };
    static const uint8_t expected_drive_wire[] = {
        0x06, 0x02, 0x10, 0x34, 0x12, 0x14, 0x03, 0x01, 0x64, 0x05,
        0x0F, 0x40, 0x42, 0x0F, 0x05, 0x80, 0x7B, 0xE1, 0xFF, 0x01,
        0x01, 0x01, 0x04, 0x20, 0xA1, 0x07, 0x03, 0xD7, 0x65, 0x00
    };
    static const uint8_t expected_arm_payload[29] = {
        0x01, 0x64, 0x00, 0x06, 0x3F, 0xA0, 0x86, 0x01, 0x00,
        0xC0, 0xF2, 0xFC, 0xFF, 0xE0, 0x93, 0x04, 0x00,
        0x80, 0xE5, 0xF9, 0xFF, 0x20, 0xA1, 0x07, 0x00,
        0x40, 0xD8, 0xF6, 0xFF
    };
    proto_v2_drive_command_t drive = {
        .mode = 1U,
        .lease_ms = 100U,
        .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {1000000, -2000000, 0, 500000}
    };
    proto_v2_frame_t frame = {
        .version = PROTO_V2_VERSION,
        .type = PROTO_V2_TYPE_DRIVE_COMMAND,
        .sequence = 0x1234U
    };
    uint8_t payload[PROTO_V2_MAX_PAYLOAD];
    uint8_t wire[PROTO_V2_MAX_WIRE];
    size_t payload_length = 0U, wire_length = 0U;
    proto_v2_frame_t decoded;
    proto_v2_drive_command_t decoded_drive;
    proto_v2_arm_command_t arm = {
        .mode = 1U,
        .lease_ms = 100U,
        .axis_count = 6U,
        .valid_mask = 0x3FU,
        .position_urad = {100000, -200000, 300000, -400000, 500000, -600000, 0}
    };
    proto_v2_arm_command_t decoded_arm;

    assert(proto_v2_crc16(crc_input, sizeof(crc_input) - 1U) == 0x29B1U);
    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &payload_length));
    assert(payload_length == sizeof(expected_drive_payload));
    assert(memcmp(payload, expected_drive_payload, payload_length) == 0);
    frame.payload_length = (uint16_t)payload_length;
    memcpy(frame.payload, payload, payload_length);
    assert(proto_v2_encode(&frame, wire, sizeof(wire), &wire_length));
    assert(wire_length == sizeof(expected_drive_wire));
    assert(memcmp(wire, expected_drive_wire, wire_length) == 0);
    assert(proto_v2_decode(wire, wire_length, &decoded));
    assert(decoded.version == PROTO_V2_VERSION);
    assert(decoded.type == PROTO_V2_TYPE_DRIVE_COMMAND);
    assert(decoded.sequence == 0x1234U);
    assert(proto_v2_unpack_drive(decoded.payload, decoded.payload_length, &decoded_drive));
    assert(decoded_drive.mode == drive.mode);
    assert(decoded_drive.lease_ms == drive.lease_ms);
    assert(decoded_drive.valid_mask == drive.valid_mask);
    assert(memcmp(decoded_drive.wheel_velocity_urad_s,
                  drive.wheel_velocity_urad_s,
                  sizeof(drive.wheel_velocity_urad_s)) == 0);

    wire[wire_length - 2U] ^= 1U;
    assert(!proto_v2_decode(wire, wire_length, &decoded));

    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &payload_length));
    assert(payload_length == sizeof(expected_arm_payload));
    assert(memcmp(payload, expected_arm_payload, payload_length) == 0);
    assert(proto_v2_unpack_arm(payload, payload_length, 0U, &decoded_arm));
    assert(decoded_arm.axis_count == 6U);
    assert(memcmp(decoded_arm.position_urad, arm.position_urad,
                  6U * sizeof(arm.position_urad[0])) == 0);

    arm.axis_count = 7U;
    arm.valid_mask = 0x7FU;
    arm.position_urad[6] = 30000;
    assert(!proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &payload_length));
    assert(proto_v2_pack_arm(&arm, PROTO_V2_FEATURE_GRIPPER_SERVO,
                             payload, sizeof(payload), &payload_length));
    assert(payload_length == 33U);
    assert(!proto_v2_unpack_arm(payload, payload_length, 0U, &decoded_arm));
    assert(proto_v2_unpack_arm(payload, payload_length,
                               PROTO_V2_FEATURE_GRIPPER_SERVO, &decoded_arm));
    assert(decoded_arm.axis_count == 7U);
    assert(decoded_arm.position_urad[6] == 30000);
    return 0;
}
