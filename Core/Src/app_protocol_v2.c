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

static uint64_t read_u64(const uint8_t *p)
{
    uint64_t value = 0U;
    for (unsigned i = 0U; i < 8U; ++i)
        value |= (uint64_t)p[i] << (i * 8U);
    return value;
}

static int64_t read_i64(const uint8_t *p)
{
    uint64_t raw = read_u64(p);
    if (raw <= 0x7FFFFFFFFFFFFFFFULL) return (int64_t)raw;
    return -1 - (int64_t)(~raw);
}

static int16_t read_i16(const uint8_t *p)
{
    uint16_t raw = read_u16(p);
    int32_t value = raw <= 0x7FFFU ? (int32_t)raw
                                   : (int32_t)raw - 0x10000L;
    return (int16_t)value;
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

static void write_u64(uint8_t *p, uint64_t value)
{
    for (unsigned i = 0U; i < 8U; ++i)
        p[i] = (uint8_t)(value >> (i * 8U));
}

static int known_message_type(uint8_t type)
{
    switch (type) {
    case PROTO_V2_TYPE_HELLO:
    case PROTO_V2_TYPE_HELLO_ACK:
    case PROTO_V2_TYPE_TIME_SYNC_REQUEST:
    case PROTO_V2_TYPE_TIME_SYNC_RESPONSE:
    case PROTO_V2_TYPE_DRIVE_COMMAND:
    case PROTO_V2_TYPE_ARM_COMMAND:
    case PROTO_V2_TYPE_SAFETY_COMMAND:
    case PROTO_V2_TYPE_DRIVE_STATE:
    case PROTO_V2_TYPE_ARM_STATE:
    case PROTO_V2_TYPE_DIAGNOSTICS:
    case PROTO_V2_TYPE_FAULT_EVENT:
        return 1;
    default:
        return 0;
    }
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

int proto_v2_required_features_supported(uint32_t accepted_features)
{
    return (accepted_features & PROTO_V2_REQUIRED_FEATURES) ==
           PROTO_V2_REQUIRED_FEATURES;
}

int proto_v2_sequence_is_newer(uint16_t candidate, uint16_t previous)
{
    uint16_t delta = (uint16_t)(candidate - previous);
    return delta != 0U && delta <= 0x7FFFU;
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
        !known_message_type(frame->type) ||
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
    if (!known_message_type(raw[1])) return 0;

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
    proto_v2_drive_command_t parsed;
    uint8_t check[20];
    size_t check_length;
    if (payload == NULL || command == NULL || length != 20U) return 0;
    parsed.mode = payload[0];
    parsed.lease_ms = read_u16(&payload[1]);
    parsed.valid_mask = payload[3];
    for (unsigned i = 0U; i < 4U; ++i)
        parsed.wheel_velocity_urad_s[i] = read_i32(&payload[4U + i * 4U]);
    if (!proto_v2_pack_drive(&parsed, check, sizeof(check), &check_length)) return 0;
    *command = parsed;
    return 1;
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
    if (command->mode == V2_MODE_DISARMED) {
        for (unsigned i = 0U; i < command->axis_count; ++i)
            if (command->position_urad[i] != 0) return 0;
    }
    payload[0] = command->mode;
    write_u16(&payload[1], command->lease_ms);
    payload[3] = command->axis_count;
    payload[4] = command->valid_mask;
    for (unsigned i = 0U; i < command->axis_count; ++i) {
        write_u32(&payload[5U + i * 4U], (uint32_t)command->position_urad[i]);
    }
    *length = expected;
    return 1;
}

int proto_v2_unpack_arm(const uint8_t *payload, size_t length,
                        uint32_t accepted_features, proto_v2_arm_command_t *command)
{
    proto_v2_arm_command_t parsed = {0};
    uint8_t check[33];
    size_t check_length;
    size_t expected;
    if (payload == NULL || command == NULL || length < 5U ||
        (payload[3] != 6U && payload[3] != 7U)) return 0;
    if (payload[3] == 7U &&
        (accepted_features & PROTO_V2_FEATURE_GRIPPER_SERVO) == 0U) return 0;
    expected = 5U + (size_t)payload[3] * 4U;
    if (length != expected) return 0;
    parsed.mode = payload[0];
    parsed.lease_ms = read_u16(&payload[1]);
    parsed.axis_count = payload[3];
    parsed.valid_mask = payload[4];
    for (unsigned i = 0U; i < parsed.axis_count; ++i)
        parsed.position_urad[i] = read_i32(&payload[5U + i * 4U]);
    if (!proto_v2_pack_arm(&parsed, accepted_features, check, sizeof(check),
                           &check_length)) return 0;
    *command = parsed;
    return 1;
}

int proto_v2_pack_hello(const proto_v2_hello_t *hello, uint8_t *payload,
                        size_t capacity, size_t *length)
{
    if (hello == NULL || payload == NULL || length == NULL || capacity < 8U ||
        (hello->requested_features & ~PROTO_V2_KNOWN_FEATURES) != 0U) return 0;
    write_u32(&payload[0], hello->host_build_id);
    write_u32(&payload[4], hello->requested_features);
    *length = 8U;
    return 1;
}

int proto_v2_unpack_hello(const uint8_t *payload, size_t length,
                          proto_v2_hello_t *hello)
{
    proto_v2_hello_t parsed;
    if (payload == NULL || hello == NULL || length != 8U) return 0;
    parsed.host_build_id = read_u32(&payload[0]);
    parsed.requested_features = read_u32(&payload[4]);
    if ((parsed.requested_features & ~PROTO_V2_KNOWN_FEATURES) != 0U) return 0;
    *hello = parsed;
    return 1;
}

int proto_v2_pack_hello_ack(const proto_v2_hello_ack_t *ack, uint8_t *payload,
                            size_t capacity, size_t *length)
{
    if (ack == NULL || payload == NULL || length == NULL || capacity < 14U ||
        (ack->accepted_features & ~PROTO_V2_KNOWN_FEATURES) != 0U ||
        !proto_v2_required_features_supported(ack->accepted_features)) return 0;
    write_u16(&payload[0], ack->board_id);
    write_u32(&payload[2], ack->firmware_version);
    write_u32(&payload[6], ack->boot_id);
    write_u32(&payload[10], ack->accepted_features);
    *length = 14U;
    return 1;
}

int proto_v2_unpack_hello_ack(const uint8_t *payload, size_t length,
                              proto_v2_hello_ack_t *ack)
{
    proto_v2_hello_ack_t parsed;
    if (payload == NULL || ack == NULL || length != 14U) return 0;
    parsed.board_id = read_u16(&payload[0]);
    parsed.firmware_version = read_u32(&payload[2]);
    parsed.boot_id = read_u32(&payload[6]);
    parsed.accepted_features = read_u32(&payload[10]);
    if ((parsed.accepted_features & ~PROTO_V2_KNOWN_FEATURES) != 0U ||
        !proto_v2_required_features_supported(parsed.accepted_features)) return 0;
    *ack = parsed;
    return 1;
}

int proto_v2_pack_time_sync_request(uint64_t host_send_monotonic_ns,
                                    uint8_t *payload, size_t capacity,
                                    size_t *length)
{
    if (payload == NULL || length == NULL || capacity < 8U) return 0;
    write_u64(payload, host_send_monotonic_ns);
    *length = 8U;
    return 1;
}

int proto_v2_unpack_time_sync_request(const uint8_t *payload, size_t length,
                                      uint64_t *host_send_monotonic_ns)
{
    uint64_t parsed;
    if (payload == NULL || host_send_monotonic_ns == NULL || length != 8U) return 0;
    parsed = read_u64(payload);
    *host_send_monotonic_ns = parsed;
    return 1;
}

int proto_v2_pack_time_sync_response(const proto_v2_time_sync_response_t *response,
                                     uint8_t *payload, size_t capacity,
                                     size_t *length)
{
    if (response == NULL || payload == NULL || length == NULL || capacity < 24U)
        return 0;
    write_u64(&payload[0], response->host_send_monotonic_ns);
    write_u64(&payload[8], response->device_rx_uptime_us);
    write_u64(&payload[16], response->device_tx_uptime_us);
    *length = 24U;
    return 1;
}

int proto_v2_unpack_time_sync_response(const uint8_t *payload, size_t length,
                                       proto_v2_time_sync_response_t *response)
{
    proto_v2_time_sync_response_t parsed;
    if (payload == NULL || response == NULL || length != 24U) return 0;
    parsed.host_send_monotonic_ns = read_u64(&payload[0]);
    parsed.device_rx_uptime_us = read_u64(&payload[8]);
    parsed.device_tx_uptime_us = read_u64(&payload[16]);
    *response = parsed;
    return 1;
}

static int valid_safety_command(const proto_v2_safety_command_t *command)
{
    return command != NULL &&
           command->operation >= PROTO_V2_SAFETY_STOP &&
           command->operation <= PROTO_V2_SAFETY_CLEAR_INHIBIT &&
           command->subsystem_mask != 0U &&
           (command->subsystem_mask &
            (uint8_t)~(PROTO_V2_SUBSYSTEM_DRIVE | PROTO_V2_SUBSYSTEM_ARM)) == 0U;
}

int proto_v2_pack_safety_command(const proto_v2_safety_command_t *command,
                                 uint8_t *payload, size_t capacity,
                                 size_t *length)
{
    if (!valid_safety_command(command) || payload == NULL || length == NULL ||
        capacity < 2U) return 0;
    payload[0] = command->operation;
    payload[1] = command->subsystem_mask;
    *length = 2U;
    return 1;
}

int proto_v2_unpack_safety_command(const uint8_t *payload, size_t length,
                                   proto_v2_safety_command_t *command)
{
    proto_v2_safety_command_t parsed;
    if (payload == NULL || command == NULL || length != 2U) return 0;
    parsed.operation = payload[0];
    parsed.subsystem_mask = payload[1];
    if (!valid_safety_command(&parsed)) return 0;
    *command = parsed;
    return 1;
}

static int valid_state_mode(uint8_t mode)
{
    return mode == V2_MODE_DISARMED || mode == V2_MODE_ARMED;
}

int proto_v2_pack_drive_state(const proto_v2_drive_state_t *state,
                              uint8_t *payload, size_t capacity,
                              size_t *length)
{
    uint8_t encoded[68];
    if (state == NULL || payload == NULL || length == NULL || capacity < sizeof(encoded) ||
        !valid_state_mode(state->mode) ||
        (state->feedback_valid_mask & 0xF0U) != 0U ||
        (state->fault_bitmap & ~PROTO_V2_KNOWN_FAULTS) != 0U) return 0;
    write_u16(&encoded[0], state->echoed_command_sequence);
    encoded[2] = state->mode;
    encoded[3] = state->feedback_valid_mask;
    write_u32(&encoded[4], state->fault_bitmap);
    write_u32(&encoded[8], state->encoder_epoch);
    write_u64(&encoded[12], state->sample_time_us);
    for (unsigned i = 0U; i < 4U; ++i)
        write_u64(&encoded[20U + i * 8U], (uint64_t)state->wheel_position_urad[i]);
    for (unsigned i = 0U; i < 4U; ++i)
        write_u32(&encoded[52U + i * 4U], (uint32_t)state->wheel_velocity_urad_s[i]);
    memcpy(payload, encoded, sizeof(encoded));
    *length = sizeof(encoded);
    return 1;
}

int proto_v2_unpack_drive_state(const uint8_t *payload, size_t length,
                                proto_v2_drive_state_t *state)
{
    proto_v2_drive_state_t parsed = {0};
    uint8_t check[68];
    size_t check_length;
    if (payload == NULL || state == NULL || length != sizeof(check)) return 0;
    parsed.echoed_command_sequence = read_u16(&payload[0]);
    parsed.mode = payload[2];
    parsed.feedback_valid_mask = payload[3];
    parsed.fault_bitmap = read_u32(&payload[4]);
    parsed.encoder_epoch = read_u32(&payload[8]);
    parsed.sample_time_us = read_u64(&payload[12]);
    for (unsigned i = 0U; i < 4U; ++i)
        parsed.wheel_position_urad[i] = read_i64(&payload[20U + i * 8U]);
    for (unsigned i = 0U; i < 4U; ++i)
        parsed.wheel_velocity_urad_s[i] = read_i32(&payload[52U + i * 4U]);
    if (!proto_v2_pack_drive_state(&parsed, check, sizeof(check), &check_length)) return 0;
    *state = parsed;
    return 1;
}

static int valid_arm_state(const proto_v2_arm_state_t *state,
                           uint32_t accepted_features)
{
    uint8_t allowed_mask;
    if (state == NULL ||
        (accepted_features & ~PROTO_V2_KNOWN_FEATURES) != 0U ||
        (state->axis_count != 6U && state->axis_count != 7U) ||
        !valid_state_mode(state->mode) || state->measured_source > 3U ||
        (state->fault_bitmap & ~PROTO_V2_KNOWN_FAULTS) != 0U) return 0;
    if (state->axis_count == 7U &&
        (accepted_features & PROTO_V2_FEATURE_GRIPPER_SERVO) == 0U) return 0;
    allowed_mask = (uint8_t)((1U << state->axis_count) - 1U);
    if ((state->measured_valid_mask & (uint8_t)~allowed_mask) != 0U) return 0;
    if ((accepted_features & PROTO_V2_FEATURE_ARM_MEASURED_POSITION) == 0U &&
        (state->measured_source != 0U || state->measured_valid_mask != 0U)) return 0;
    if (state->measured_source == 0U) {
        if (state->measured_valid_mask != 0U) return 0;
        for (unsigned i = 0U; i < state->axis_count; ++i)
            if (state->measured_position_urad[i] != 0) return 0;
    }
    return 1;
}

int proto_v2_pack_arm_state(const proto_v2_arm_state_t *state,
                            uint32_t accepted_features, uint8_t *payload,
                            size_t capacity, size_t *length)
{
    uint8_t encoded[74];
    size_t encoded_length;
    if (state == NULL || payload == NULL || length == NULL ||
        !valid_arm_state(state, accepted_features)) return 0;
    encoded_length = 18U + (size_t)state->axis_count * 8U;
    if (capacity < encoded_length) return 0;
    write_u16(&encoded[0], state->echoed_command_sequence);
    encoded[2] = state->mode;
    encoded[3] = state->axis_count;
    encoded[4] = state->measured_valid_mask;
    encoded[5] = state->measured_source;
    write_u32(&encoded[6], state->fault_bitmap);
    write_u64(&encoded[10], state->sample_time_us);
    for (unsigned i = 0U; i < state->axis_count; ++i)
        write_u32(&encoded[18U + i * 4U],
                  (uint32_t)state->command_position_urad[i]);
    for (unsigned i = 0U; i < state->axis_count; ++i)
        write_u32(&encoded[18U + (size_t)state->axis_count * 4U + i * 4U],
                  (uint32_t)state->measured_position_urad[i]);
    memcpy(payload, encoded, encoded_length);
    *length = encoded_length;
    return 1;
}

int proto_v2_unpack_arm_state(const uint8_t *payload, size_t length,
                              uint32_t accepted_features,
                              proto_v2_arm_state_t *state)
{
    proto_v2_arm_state_t parsed = {0};
    uint8_t check[74];
    size_t check_length;
    size_t expected;
    if (payload == NULL || state == NULL || length < 18U ||
        (payload[3] != 6U && payload[3] != 7U)) return 0;
    expected = 18U + (size_t)payload[3] * 8U;
    if (length != expected) return 0;
    parsed.echoed_command_sequence = read_u16(&payload[0]);
    parsed.mode = payload[2];
    parsed.axis_count = payload[3];
    parsed.measured_valid_mask = payload[4];
    parsed.measured_source = payload[5];
    parsed.fault_bitmap = read_u32(&payload[6]);
    parsed.sample_time_us = read_u64(&payload[10]);
    for (unsigned i = 0U; i < parsed.axis_count; ++i)
        parsed.command_position_urad[i] = read_i32(&payload[18U + i * 4U]);
    for (unsigned i = 0U; i < parsed.axis_count; ++i)
        parsed.measured_position_urad[i] =
            read_i32(&payload[18U + (size_t)parsed.axis_count * 4U + i * 4U]);
    if (!valid_arm_state(&parsed, accepted_features) ||
        !proto_v2_pack_arm_state(&parsed, accepted_features, check, sizeof(check),
                                 &check_length)) return 0;
    *state = parsed;
    return 1;
}

int proto_v2_pack_diagnostics(const proto_v2_diagnostics_t *diagnostics,
                              uint8_t *payload, size_t capacity,
                              size_t *length)
{
    uint8_t encoded[36];
    const uint32_t counters[6] = {
        diagnostics != NULL ? diagnostics->crc_error_count : 0U,
        diagnostics != NULL ? diagnostics->sequence_error_count : 0U,
        diagnostics != NULL ? diagnostics->rx_overrun_count : 0U,
        diagnostics != NULL ? diagnostics->tx_drop_count : 0U,
        diagnostics != NULL ? diagnostics->control_deadline_miss_count : 0U,
        diagnostics != NULL ? diagnostics->reset_count : 0U
    };
    if (diagnostics == NULL || payload == NULL || length == NULL ||
        capacity < sizeof(encoded)) return 0;
    write_u64(&encoded[0], diagnostics->device_uptime_us);
    write_u16(&encoded[8], diagnostics->battery_mv);
    write_u16(&encoded[10], (uint16_t)diagnostics->board_temp_centi_c);
    for (unsigned i = 0U; i < 6U; ++i)
        write_u32(&encoded[12U + i * 4U], counters[i]);
    memcpy(payload, encoded, sizeof(encoded));
    *length = sizeof(encoded);
    return 1;
}

int proto_v2_unpack_diagnostics(const uint8_t *payload, size_t length,
                                proto_v2_diagnostics_t *diagnostics)
{
    proto_v2_diagnostics_t parsed;
    if (payload == NULL || diagnostics == NULL || length != 36U) return 0;
    parsed.device_uptime_us = read_u64(&payload[0]);
    parsed.battery_mv = read_u16(&payload[8]);
    parsed.board_temp_centi_c = read_i16(&payload[10]);
    parsed.crc_error_count = read_u32(&payload[12]);
    parsed.sequence_error_count = read_u32(&payload[16]);
    parsed.rx_overrun_count = read_u32(&payload[20]);
    parsed.tx_drop_count = read_u32(&payload[24]);
    parsed.control_deadline_miss_count = read_u32(&payload[28]);
    parsed.reset_count = read_u32(&payload[32]);
    *diagnostics = parsed;
    return 1;
}

int proto_v2_pack_fault_event(const proto_v2_fault_event_t *event,
                              uint8_t *payload, size_t capacity,
                              size_t *length)
{
    uint8_t encoded[7];
    if (event == NULL || payload == NULL || length == NULL || capacity < sizeof(encoded) ||
        (event->fault_bitmap & ~PROTO_V2_KNOWN_FAULTS) != 0U ||
        event->subsystem < 1U || event->subsystem > 3U) return 0;
    write_u32(&encoded[0], event->fault_bitmap);
    write_u16(&encoded[4], event->source_command_sequence);
    encoded[6] = event->subsystem;
    memcpy(payload, encoded, sizeof(encoded));
    *length = sizeof(encoded);
    return 1;
}

int proto_v2_unpack_fault_event(const uint8_t *payload, size_t length,
                                proto_v2_fault_event_t *event)
{
    proto_v2_fault_event_t parsed;
    if (payload == NULL || event == NULL || length != 7U) return 0;
    parsed.fault_bitmap = read_u32(&payload[0]);
    parsed.source_command_sequence = read_u16(&payload[4]);
    parsed.subsystem = payload[6];
    if ((parsed.fault_bitmap & ~PROTO_V2_KNOWN_FAULTS) != 0U ||
        parsed.subsystem < 1U || parsed.subsystem > 3U) return 0;
    *event = parsed;
    return 1;
}
