#include "app_protocol_v2.h"

#include <string.h>

#define V2_MODE_DISARMED 0U
#define V2_MODE_ARMED    1U
#define V2_LEASE_MIN_MS  10U
#define V2_LEASE_MAX_MS  200U

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t read_i32(const uint8_t *p)
{
    uint32_t raw = read_u32(p);
    int64_t value = (raw <= 0x7FFFFFFFUL) ? (int64_t)raw
                                           : (int64_t)raw - 0x100000000LL;
    return (int32_t)value;
}

static void write_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void write_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

uint16_t proto_v2_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0U; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static int cobs_encode(const uint8_t *input, size_t input_length,
                       uint8_t *output, size_t capacity, size_t *output_length)
{
    size_t read_index = 0U, write_index = 1U, code_index = 0U;
    uint8_t code = 1U;
    if (capacity == 0U) return 0;

    while (read_index < input_length) {
        uint8_t value = input[read_index++];
        if (value == 0U) {
            output[code_index] = code;
            if (write_index >= capacity) return 0;
            code_index = write_index++;
            code = 1U;
        } else {
            if (write_index >= capacity) return 0;
            output[write_index++] = value;
            if (++code == 0xFFU) {
                output[code_index] = code;
                if (write_index >= capacity) return 0;
                code_index = write_index++;
                code = 1U;
            }
        }
    }
    output[code_index] = code;
    *output_length = write_index;
    return 1;
}

static int cobs_decode(const uint8_t *input, size_t input_length,
                       uint8_t *output, size_t capacity, size_t *output_length)
{
    size_t read_index = 0U, write_index = 0U;
    if (input_length == 0U) return 0;
    while (read_index < input_length) {
        uint8_t code = input[read_index++];
        size_t copy_length;
        if (code == 0U) return 0;
        copy_length = (size_t)code - 1U;
        if (read_index + copy_length > input_length ||
            write_index + copy_length > capacity) return 0;
        memcpy(&output[write_index], &input[read_index], copy_length);
        read_index += copy_length;
        write_index += copy_length;
        if (code != 0xFFU && read_index < input_length) {
            if (write_index >= capacity) return 0;
            output[write_index++] = 0U;
        }
    }
    *output_length = write_index;
    return 1;
}

int proto_v2_encode(const proto_v2_frame_t *frame, uint8_t *wire,
                    size_t wire_capacity, size_t *wire_length)
{
    uint8_t raw[PROTO_V2_MAX_RAW];
    size_t raw_length, encoded_length;
    uint16_t crc;

    if (frame == NULL || wire == NULL || wire_length == NULL ||
        frame->version != PROTO_V2_VERSION ||
        frame->payload_length > PROTO_V2_MAX_PAYLOAD ||
        wire_capacity < 2U) return 0;

    raw[0] = frame->version;
    raw[1] = frame->type;
    write_u16(&raw[2], frame->sequence);
    write_u16(&raw[4], frame->payload_length);
    memcpy(&raw[6], frame->payload, frame->payload_length);
    raw_length = 6U + frame->payload_length;
    crc = proto_v2_crc16(raw, raw_length);
    write_u16(&raw[raw_length], crc);
    raw_length += 2U;

    if (!cobs_encode(raw, raw_length, wire, wire_capacity - 1U, &encoded_length))
        return 0;
    wire[encoded_length++] = 0U;
    *wire_length = encoded_length;
    return 1;
}

int proto_v2_decode(const uint8_t *wire, size_t wire_length,
                    proto_v2_frame_t *frame)
{
    uint8_t raw[PROTO_V2_MAX_RAW];
    size_t raw_length;
    uint16_t payload_length;

    if (wire == NULL || frame == NULL || wire_length < 2U ||
        wire[wire_length - 1U] != 0U ||
        !cobs_decode(wire, wire_length - 1U, raw, sizeof(raw), &raw_length) ||
        raw_length < 8U || raw[0] != PROTO_V2_VERSION) return 0;

    payload_length = read_u16(&raw[4]);
    if (payload_length > PROTO_V2_MAX_PAYLOAD ||
        raw_length != 8U + payload_length ||
        read_u16(&raw[6U + payload_length]) != proto_v2_crc16(raw, 6U + payload_length))
        return 0;

    frame->version = raw[0];
    frame->type = raw[1];
    frame->sequence = read_u16(&raw[2]);
    frame->payload_length = payload_length;
    memcpy(frame->payload, &raw[6], payload_length);
    return 1;
}

int proto_v2_pack_drive(const proto_v2_drive_command_t *command,
                        uint8_t *payload, size_t capacity, size_t *length)
{
    if (command == NULL || payload == NULL || length == NULL || capacity < 20U ||
        command->mode > V2_MODE_ARMED ||
        command->lease_ms < V2_LEASE_MIN_MS || command->lease_ms > V2_LEASE_MAX_MS ||
        (command->mode == V2_MODE_ARMED && command->valid_mask != 0x0FU) ||
        (command->mode == V2_MODE_DISARMED &&
         (command->valid_mask != 0U || command->wheel_velocity_urad_s[0] != 0 ||
          command->wheel_velocity_urad_s[1] != 0 ||
          command->wheel_velocity_urad_s[2] != 0 ||
          command->wheel_velocity_urad_s[3] != 0))) return 0;
    payload[0] = command->mode;
    write_u16(&payload[1], command->lease_ms);
    payload[3] = command->valid_mask;
    for (unsigned i = 0U; i < 4U; ++i)
        write_u32(&payload[4U + i * 4U], (uint32_t)command->wheel_velocity_urad_s[i]);
    *length = 20U;
    return 1;
}

int proto_v2_unpack_drive(const uint8_t *payload, size_t length,
                          proto_v2_drive_command_t *command)
{
    if (payload == NULL || command == NULL || length != 20U) return 0;
    command->mode = payload[0];
    command->lease_ms = read_u16(&payload[1]);
    command->valid_mask = payload[3];
    for (unsigned i = 0U; i < 4U; ++i)
        command->wheel_velocity_urad_s[i] = read_i32(&payload[4U + i * 4U]);
    {
        uint8_t check[20];
        size_t check_length;
        return proto_v2_pack_drive(command, check, sizeof(check), &check_length);
    }
}

int proto_v2_pack_arm(const proto_v2_arm_command_t *command,
                      uint32_t accepted_features, uint8_t *payload,
                      size_t capacity, size_t *length)
{
    size_t expected;
    if (command == NULL || payload == NULL || length == NULL ||
        (command->axis_count != 6U && command->axis_count != 7U)) return 0;
    if (command->axis_count == 7U &&
        (accepted_features & PROTO_V2_FEATURE_GRIPPER_SERVO) == 0U) return 0;
    expected = 5U + (size_t)command->axis_count * 4U;
    if (capacity < expected || command->mode > V2_MODE_ARMED ||
        command->lease_ms < V2_LEASE_MIN_MS || command->lease_ms > V2_LEASE_MAX_MS ||
        (command->mode == V2_MODE_ARMED &&
         command->valid_mask != (uint8_t)((1U << command->axis_count) - 1U)) ||
        (command->mode == V2_MODE_DISARMED && command->valid_mask != 0U)) return 0;
    payload[0] = command->mode;
    write_u16(&payload[1], command->lease_ms);
    payload[3] = command->axis_count;
    payload[4] = command->valid_mask;
    for (unsigned i = 0U; i < command->axis_count; ++i) {
        if (command->mode == V2_MODE_DISARMED && command->position_urad[i] != 0)
            return 0;
        write_u32(&payload[5U + i * 4U], (uint32_t)command->position_urad[i]);
    }
    *length = expected;
    return 1;
}

int proto_v2_unpack_arm(const uint8_t *payload, size_t length,
                        uint32_t accepted_features, proto_v2_arm_command_t *command)
{
    size_t expected;
    if (payload == NULL || command == NULL || length < 5U ||
        (payload[3] != 6U && payload[3] != 7U)) return 0;
    if (payload[3] == 7U &&
        (accepted_features & PROTO_V2_FEATURE_GRIPPER_SERVO) == 0U) return 0;
    expected = 5U + (size_t)payload[3] * 4U;
    if (length != expected) return 0;
    command->mode = payload[0];
    command->lease_ms = read_u16(&payload[1]);
    command->axis_count = payload[3];
    command->valid_mask = payload[4];
    for (unsigned i = 0U; i < command->axis_count; ++i)
        command->position_urad[i] = read_i32(&payload[5U + i * 4U]);
    {
        uint8_t check[33];
        size_t check_length;
        return proto_v2_pack_arm(command, accepted_features, check, sizeof(check),
                                 &check_length);
    }
}
