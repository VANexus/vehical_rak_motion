#include "app_protocol_v2.h"

#include <assert.h>
#include <string.h>

typedef struct {
    const char *wire_hex;
    uint8_t type;
    uint16_t sequence;
} golden_frame_t;

static int hex_value(char value)
{
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

static size_t parse_hex(const char *text, uint8_t *output, size_t capacity)
{
    size_t length = 0U;
    while (*text != '\0') {
        int high, low;
        while (*text == ' ') ++text;
        if (*text == '\0') break;
        high = hex_value(*text++);
        assert(high >= 0 && *text != '\0');
        low = hex_value(*text++);
        assert(low >= 0 && length < capacity);
        output[length++] = (uint8_t)((high << 4) | low);
        assert(*text == ' ' || *text == '\0');
    }
    return length;
}

static const golden_frame_t golden_frames[] = {
    {"04 02 01 01 02 08 06 78 56 34 12 07 01 01 03 48 8e 00", 0x01U, 0x0001U},
    {"04 02 02 02 02 0e 06 07 04 03 02 01 06 d4 c3 b2 a1 07 01 01 03 95 99 00", 0x02U, 0x0002U},
    {"04 02 03 03 02 08 0b 08 07 06 05 04 03 02 01 13 93 00", 0x03U, 0x0003U},
    {"04 02 04 04 02 18 1b 08 07 06 05 04 03 02 01 18 17 16 15 14 13 12 11 28 27 26 25 24 23 22 21 c1 89 00", 0x04U, 0x0004U},
    {"06 02 10 34 12 14 03 01 64 05 0f 40 42 0f 05 80 7b e1 ff 01 01 01 04 20 a1 07 03 d7 65 00", 0x10U, 0x1234U},
    {"04 02 11 11 02 1d 03 01 64 06 06 3f a0 86 01 08 c0 f2 fc ff e0 93 04 08 80 e5 f9 ff 20 a1 07 07 40 d8 f6 ff 63 48 00", 0x11U, 0x0011U},
    {"04 02 12 12 02 02 05 02 03 ad c3 00", 0x12U, 0x0012U},
    {"04 02 20 20 02 46 16 34 12 01 0f 04 03 02 01 44 33 22 11 08 07 06 05 04 03 02 01 01 01 01 01 01 01 01 0a fe ff ff ff ff ff ff ff 03 01 01 01 01 01 01 0a fc ff ff ff ff ff ff ff 05 01 01 07 06 fa ff ff ff 07 01 01 08 08 f8 ff ff ff 83 74 00", 0x20U, 0x0020U},
    {"04 02 21 21 02 42 03 34 12 02 06 01 01 01 01 01 "
     "09 08 07 06 05 04 03 02 01 01 01 01 01 01 01 01 "
     "01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 "
     "01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 "
     "01 01 01 01 01 01 01 01 03 cf 9b 00", 0x21U, 0x0021U},
    {"04 02 22 22 02 24 0e 08 07 06 05 04 03 02 01 18 2e 1e fb 01 01 01 02 02 01 01 02 03 01 01 02 04 01 01 02 05 01 01 02 06 01 01 03 6b 8c 00", 0x22U, 0x0022U},
    {"04 02 23 23 02 07 0a 78 56 34 12 bc 9a 03 9d 7f 00", 0x23U, 0x0023U}
};

static void test_golden_frames(void)
{
    for (size_t i = 0U; i < sizeof(golden_frames) / sizeof(golden_frames[0]); ++i) {
        uint8_t expected[PROTO_V2_MAX_WIRE];
        uint8_t encoded[PROTO_V2_MAX_WIRE];
        size_t expected_length = parse_hex(golden_frames[i].wire_hex, expected,
                                           sizeof(expected));
        size_t encoded_length = 0U;
        proto_v2_frame_t frame;
        assert(proto_v2_decode(expected, expected_length, &frame));
        assert(frame.version == PROTO_V2_VERSION);
        assert(frame.type == golden_frames[i].type);
        assert(frame.sequence == golden_frames[i].sequence);
        assert(proto_v2_encode(&frame, encoded, sizeof(encoded), &encoded_length));
        assert(encoded_length == expected_length);
        assert(memcmp(encoded, expected, expected_length) == 0);
    }
}

static void test_invalid_frames(void)
{
    static const uint8_t malformed_cobs_overrun[] = {0x05U, 0x11U, 0x22U, 0x00U};
    static const uint8_t malformed_cobs_zero_code[] = {0x00U, 0x00U};
    static const uint8_t wrong_version_raw[] = {
        0x01U, 0x01U, 0x01U, 0x00U, 0x00U, 0x00U, 0x55U, 0x97U
    };
    static const uint8_t wrong_version_wire[] = {
        0x04U, 0x01U, 0x01U, 0x01U, 0x01U, 0x01U, 0x03U, 0x55U, 0x97U, 0x00U
    };
    static const uint8_t length_mismatch_raw[] = {
        0x02U, 0x01U, 0x01U, 0x00U, 0x02U, 0x00U, 0xAAU, 0x1CU, 0x04U
    };
    static const uint8_t length_mismatch_wire[] = {
        0x04U, 0x02U, 0x01U, 0x01U, 0x02U, 0x02U,
        0x04U, 0xAAU, 0x1CU, 0x04U, 0x00U
    };
    proto_v2_frame_t decoded;

    assert(proto_v2_crc16(wrong_version_raw, 6U) == 0x9755U);
    assert(proto_v2_crc16(length_mismatch_raw, 7U) == 0x041CU);
    assert(!proto_v2_decode(malformed_cobs_overrun,
                            sizeof(malformed_cobs_overrun), &decoded));
    assert(!proto_v2_decode(malformed_cobs_zero_code,
                            sizeof(malformed_cobs_zero_code), &decoded));
    assert(!proto_v2_decode(wrong_version_wire, sizeof(wrong_version_wire),
                            &decoded));
    assert(!proto_v2_decode(length_mismatch_wire, sizeof(length_mismatch_wire),
                            &decoded));
}

static void test_command_boundaries(void)
{
    uint8_t payload[33];
    size_t payload_length = 0U;
    proto_v2_drive_command_t drive = {
        .mode = 1U, .lease_ms = 10U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    proto_v2_drive_command_t decoded_drive;
    proto_v2_arm_command_t arm = {
        .mode = 1U, .lease_ms = 10U, .axis_count = 6U, .valid_mask = 0x3FU,
        .position_urad = {0, 0, 0, 0, 0, 0, 0}
    };
    proto_v2_arm_command_t decoded_arm;

    drive.lease_ms = 200U;
    assert(proto_v2_pack_drive(&drive, payload, 20U, &payload_length));
    assert(proto_v2_unpack_drive(payload, payload_length, &decoded_drive));
    assert(decoded_drive.lease_ms == 200U);
    drive.lease_ms = 9U;
    assert(!proto_v2_pack_drive(&drive, payload, 20U, &payload_length));
    drive.lease_ms = 201U;
    assert(!proto_v2_pack_drive(&drive, payload, 20U, &payload_length));

    drive.lease_ms = 10U;
    assert(proto_v2_pack_drive(&drive, payload, 20U, &payload_length));
    payload[1] = 9U;
    assert(!proto_v2_unpack_drive(payload, payload_length, &decoded_drive));
    payload[1] = 201U;
    payload[2] = 0U;
    assert(!proto_v2_unpack_drive(payload, payload_length, &decoded_drive));

    arm.lease_ms = 200U;
    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &payload_length));
    assert(proto_v2_unpack_arm(payload, payload_length, 0U, &decoded_arm));
    assert(decoded_arm.lease_ms == 200U);
    arm.lease_ms = 9U;
    assert(!proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &payload_length));
    arm.lease_ms = 201U;
    assert(!proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &payload_length));

    arm.lease_ms = 10U;
    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &payload_length));
    payload[1] = 9U;
    assert(!proto_v2_unpack_arm(payload, payload_length, 0U, &decoded_arm));
    payload[1] = 201U;
    payload[2] = 0U;
    assert(!proto_v2_unpack_arm(payload, payload_length, 0U, &decoded_arm));

    drive.lease_ms = 100U;
    drive.mode = 0U;
    drive.valid_mask = 0U;
    drive.wheel_velocity_urad_s[2] = 1;
    assert(!proto_v2_pack_drive(&drive, payload, 20U, &payload_length));
    drive.mode = 2U;
    drive.valid_mask = 0x0FU;
    drive.wheel_velocity_urad_s[2] = 0;
    assert(!proto_v2_pack_drive(&drive, payload, 20U, &payload_length));

    drive.mode = 1U;
    drive.valid_mask = 0x07U;
    assert(!proto_v2_pack_drive(&drive, payload, 20U, &payload_length));
    drive.valid_mask = 0x0FU;
    assert(proto_v2_pack_drive(&drive, payload, 20U, &payload_length));
    payload[0] = 0U;
    payload[3] = 0U;
    payload[4] = 1U;
    assert(!proto_v2_unpack_drive(payload, payload_length, &decoded_drive));

    arm.mode = 0U;
    arm.valid_mask = 0U;
    arm.position_urad[5] = 1;
    assert(!proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &payload_length));
    arm.mode = 2U;
    arm.valid_mask = 0x3FU;
    arm.position_urad[5] = 0;
    assert(!proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &payload_length));
    arm.mode = 1U;
    arm.valid_mask = 0x1FU;
    assert(!proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &payload_length));
    arm.valid_mask = 0x3FU;
    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &payload_length));
    payload[0] = 0U;
    payload[4] = 0U;
    payload[5] = 1U;
    assert(!proto_v2_unpack_arm(payload, payload_length, 0U, &decoded_arm));
}

static void test_control_payload_codecs(void)
{
    static const struct {
        const char *expected;
        proto_v2_hello_t value;
    } hello = {
        "78 56 34 12 07 00 00 00",
        {0x12345678U, PROTO_V2_REQUIRED_FEATURES}
    };
    static const struct {
        const char *expected;
        proto_v2_hello_ack_t value;
    } hello_ack = {
        "07 04 03 02 01 00 d4 c3 b2 a1 07 00 00 00",
        {0x0407U, 0x00010203U, 0xA1B2C3D4U, PROTO_V2_REQUIRED_FEATURES}
    };
    static const struct {
        const char *expected;
        uint64_t value;
    } time_request = {
        "08 07 06 05 04 03 02 01", 0x0102030405060708ULL
    };
    static const struct {
        const char *expected;
        proto_v2_time_sync_response_t value;
    } time_response = {
        "08 07 06 05 04 03 02 01 18 17 16 15 14 13 12 11 "
        "28 27 26 25 24 23 22 21",
        {0x0102030405060708ULL, 0x1112131415161718ULL,
         0x2122232425262728ULL}
    };
    static const struct {
        const char *expected;
        proto_v2_safety_command_t value;
    } safety = {"02 03", {PROTO_V2_SAFETY_LATCH_INHIBIT, 0x03U}};
    uint8_t payload[PROTO_V2_MAX_PAYLOAD];
    uint8_t expected[PROTO_V2_MAX_PAYLOAD];
    size_t length = 0U;
    proto_v2_hello_t decoded_hello;
    proto_v2_hello_ack_t decoded_ack;
    proto_v2_time_sync_response_t decoded_response;
    proto_v2_safety_command_t decoded_safety;
    uint64_t decoded_request = 0U;

    length = parse_hex(hello.expected, expected, sizeof(expected));
    assert(proto_v2_pack_hello(&hello.value, payload, sizeof(payload), &length));
    assert(length == 8U && memcmp(payload, expected, length) == 0);
    assert(proto_v2_unpack_hello(payload, length, &decoded_hello));
    assert(memcmp(&decoded_hello, &hello.value, sizeof(decoded_hello)) == 0);

    length = parse_hex(hello_ack.expected, expected, sizeof(expected));
    assert(proto_v2_pack_hello_ack(&hello_ack.value, payload, sizeof(payload), &length));
    assert(length == 14U && memcmp(payload, expected, length) == 0);
    assert(proto_v2_unpack_hello_ack(payload, length, &decoded_ack));
    assert(decoded_ack.board_id == hello_ack.value.board_id);
    assert(decoded_ack.firmware_version == hello_ack.value.firmware_version);
    assert(decoded_ack.boot_id == hello_ack.value.boot_id);
    assert(decoded_ack.accepted_features == hello_ack.value.accepted_features);

    length = parse_hex(time_request.expected, expected, sizeof(expected));
    assert(proto_v2_pack_time_sync_request(time_request.value, payload,
                                           sizeof(payload), &length));
    assert(length == 8U && memcmp(payload, expected, length) == 0);
    assert(proto_v2_unpack_time_sync_request(payload, length, &decoded_request));
    assert(decoded_request == time_request.value);

    length = parse_hex(time_response.expected, expected, sizeof(expected));
    assert(proto_v2_pack_time_sync_response(&time_response.value, payload,
                                            sizeof(payload), &length));
    assert(length == 24U && memcmp(payload, expected, length) == 0);
    assert(proto_v2_unpack_time_sync_response(payload, length, &decoded_response));
    assert(memcmp(&decoded_response, &time_response.value,
                  sizeof(decoded_response)) == 0);

    length = parse_hex(safety.expected, expected, sizeof(expected));
    assert(proto_v2_pack_safety_command(&safety.value, payload,
                                        sizeof(payload), &length));
    assert(length == 2U && memcmp(payload, expected, length) == 0);
    assert(proto_v2_unpack_safety_command(payload, length, &decoded_safety));
    assert(memcmp(&decoded_safety, &safety.value, sizeof(decoded_safety)) == 0);
}

static void test_state_payload_codecs(void)
{
    static const char drive_golden[] =
        "34 12 01 0f 04 03 02 01 44 33 22 11 08 07 06 05 04 03 02 01 "
        "01 00 00 00 00 00 00 00 fe ff ff ff ff ff ff ff "
        "03 00 00 00 00 00 00 00 fc ff ff ff ff ff ff ff "
        "05 00 00 00 fa ff ff ff 07 00 00 00 f8 ff ff ff";
    static const char arm_golden[] =
        "34 12 01 06 15 02 04 03 02 01 08 07 06 05 04 03 02 01 "
        "01 00 00 00 fe ff ff ff 03 00 00 00 fc ff ff ff 05 00 00 00 fa ff ff ff "
        "f9 ff ff ff 08 00 00 00 f7 ff ff ff 0a 00 00 00 f5 ff ff ff 0c 00 00 00";
    proto_v2_drive_state_t drive = {
        .echoed_command_sequence = 0x1234U, .mode = 1U,
        .feedback_valid_mask = 0x0FU, .fault_bitmap = 0x01020304U,
        .encoder_epoch = 0x11223344U, .sample_time_us = 0x0102030405060708ULL,
        .wheel_position_urad = {1, -2, 3, -4},
        .wheel_velocity_urad_s = {5, -6, 7, -8}
    };
    proto_v2_arm_state_t arm = {
        .echoed_command_sequence = 0x1234U, .mode = 1U, .axis_count = 6U,
        .measured_valid_mask = 0x15U, .measured_source = 2U,
        .fault_bitmap = 0x01020304U, .sample_time_us = 0x0102030405060708ULL,
        .command_position_urad = {1, -2, 3, -4, 5, -6, 0},
        .measured_position_urad = {-7, 8, -9, 10, -11, 12, 0}
    };
    proto_v2_drive_state_t decoded_drive;
    proto_v2_arm_state_t decoded_arm;
    uint8_t payload[PROTO_V2_MAX_PAYLOAD];
    uint8_t expected[PROTO_V2_MAX_PAYLOAD];
    size_t expected_length = parse_hex(drive_golden, expected, sizeof(expected));
    size_t length = 0U;

    assert(proto_v2_pack_drive_state(&drive, payload, sizeof(payload), &length));
    assert(length == expected_length && length == 68U);
    assert(memcmp(payload, expected, length) == 0);
    assert(proto_v2_unpack_drive_state(payload, length, &decoded_drive));
    assert(decoded_drive.echoed_command_sequence == drive.echoed_command_sequence);
    assert(decoded_drive.mode == drive.mode);
    assert(decoded_drive.feedback_valid_mask == drive.feedback_valid_mask);
    assert(decoded_drive.fault_bitmap == drive.fault_bitmap);
    assert(decoded_drive.encoder_epoch == drive.encoder_epoch);
    assert(decoded_drive.sample_time_us == drive.sample_time_us);
    assert(memcmp(decoded_drive.wheel_position_urad, drive.wheel_position_urad,
                  sizeof(drive.wheel_position_urad)) == 0);
    assert(memcmp(decoded_drive.wheel_velocity_urad_s, drive.wheel_velocity_urad_s,
                  sizeof(drive.wheel_velocity_urad_s)) == 0);

    expected_length = parse_hex(arm_golden, expected, sizeof(expected));
    assert(proto_v2_pack_arm_state(&arm, PROTO_V2_FEATURE_ARM_MEASURED_POSITION,
                                   payload, sizeof(payload), &length));
    assert(length == expected_length && length == 66U);
    assert(memcmp(payload, expected, length) == 0);
    assert(proto_v2_unpack_arm_state(payload, length,
                                     PROTO_V2_FEATURE_ARM_MEASURED_POSITION,
                                     &decoded_arm));
    assert(decoded_arm.echoed_command_sequence == arm.echoed_command_sequence);
    assert(decoded_arm.mode == arm.mode && decoded_arm.axis_count == arm.axis_count);
    assert(decoded_arm.measured_valid_mask == arm.measured_valid_mask);
    assert(decoded_arm.measured_source == arm.measured_source);
    assert(decoded_arm.fault_bitmap == arm.fault_bitmap);
    assert(decoded_arm.sample_time_us == arm.sample_time_us);
    assert(memcmp(decoded_arm.command_position_urad, arm.command_position_urad,
                  6U * sizeof(arm.command_position_urad[0])) == 0);
    assert(memcmp(decoded_arm.measured_position_urad, arm.measured_position_urad,
                  6U * sizeof(arm.measured_position_urad[0])) == 0);
}

static void test_diagnostic_payload_codecs(void)
{
    static const struct {
        const char *expected;
        proto_v2_diagnostics_t value;
    } diagnostics = {
        "08 07 06 05 04 03 02 01 18 2e 1e fb "
        "01 00 00 00 02 00 00 00 03 00 00 00 04 00 00 00 "
        "05 00 00 00 06 00 00 00",
        {0x0102030405060708ULL, 11800U, -1250, 1U, 2U, 3U, 4U, 5U, 6U}
    };
    static const struct {
        const char *expected;
        proto_v2_fault_event_t value;
    } fault = {"78 56 34 12 bc 9a 03", {0x12345678U, 0x9ABCU, 3U}};
    proto_v2_diagnostics_t decoded_diagnostics;
    proto_v2_fault_event_t decoded_fault;
    uint8_t payload[PROTO_V2_MAX_PAYLOAD];
    uint8_t expected[PROTO_V2_MAX_PAYLOAD];
    size_t length = parse_hex(diagnostics.expected, expected, sizeof(expected));

    assert(proto_v2_pack_diagnostics(&diagnostics.value, payload,
                                     sizeof(payload), &length));
    assert(length == 36U && memcmp(payload, expected, length) == 0);
    assert(proto_v2_unpack_diagnostics(payload, length, &decoded_diagnostics));
    assert(decoded_diagnostics.device_uptime_us == diagnostics.value.device_uptime_us);
    assert(decoded_diagnostics.battery_mv == diagnostics.value.battery_mv);
    assert(decoded_diagnostics.board_temp_centi_c ==
           diagnostics.value.board_temp_centi_c);
    assert(decoded_diagnostics.crc_error_count == diagnostics.value.crc_error_count);
    assert(decoded_diagnostics.sequence_error_count ==
           diagnostics.value.sequence_error_count);
    assert(decoded_diagnostics.rx_overrun_count == diagnostics.value.rx_overrun_count);
    assert(decoded_diagnostics.tx_drop_count == diagnostics.value.tx_drop_count);
    assert(decoded_diagnostics.control_deadline_miss_count ==
           diagnostics.value.control_deadline_miss_count);
    assert(decoded_diagnostics.reset_count == diagnostics.value.reset_count);

    length = parse_hex(fault.expected, expected, sizeof(expected));
    assert(proto_v2_pack_fault_event(&fault.value, payload, sizeof(payload), &length));
    assert(length == 7U && memcmp(payload, expected, length) == 0);
    assert(proto_v2_unpack_fault_event(payload, length, &decoded_fault));
    assert(decoded_fault.fault_bitmap == fault.value.fault_bitmap);
    assert(decoded_fault.source_command_sequence == fault.value.source_command_sequence);
    assert(decoded_fault.subsystem == fault.value.subsystem);
}

static void test_new_payload_rejections_are_transactional(void)
{
    uint8_t payload[PROTO_V2_MAX_PAYLOAD];
    uint8_t before[PROTO_V2_MAX_PAYLOAD];
    size_t length = 0xA5U;
    proto_v2_hello_t hello = {1U, 0x40U};
    proto_v2_hello_ack_t ack = {0x0407U, 1U, 2U, PROTO_V2_REQUIRED_FEATURES};
    proto_v2_safety_command_t safety = {PROTO_V2_SAFETY_STOP, 0U};
    proto_v2_drive_state_t drive = {
        .mode = 1U, .feedback_valid_mask = 0x10U
    };
    proto_v2_arm_state_t arm = {
        .mode = 1U, .axis_count = 6U, .measured_valid_mask = 0U,
        .measured_source = 4U
    };
    proto_v2_fault_event_t fault = {0x00000800U, 0U, 1U};
    proto_v2_hello_t decoded_hello;
    proto_v2_hello_ack_t decoded_ack;
    proto_v2_safety_command_t decoded_safety;
    proto_v2_drive_state_t decoded_drive;
    proto_v2_arm_state_t decoded_arm;
    proto_v2_fault_event_t decoded_fault;

    memset(payload, 0xA5, sizeof(payload));
    memcpy(before, payload, sizeof(payload));
    assert(!proto_v2_pack_hello(&hello, payload, sizeof(payload), &length));
    assert(length == 0xA5U && memcmp(payload, before, sizeof(payload)) == 0);
    hello.requested_features = PROTO_V2_REQUIRED_FEATURES;

    ack.accepted_features = 0x03U;
    assert(!proto_v2_pack_hello_ack(&ack, payload, sizeof(payload), &length));
    assert(length == 0xA5U && memcmp(payload, before, sizeof(payload)) == 0);
    ack.accepted_features = PROTO_V2_REQUIRED_FEATURES | 0x40U;
    assert(!proto_v2_pack_hello_ack(&ack, payload, sizeof(payload), &length));
    ack.accepted_features = PROTO_V2_REQUIRED_FEATURES;

    memset(payload, 0xA5, sizeof(payload));
    memcpy(before, payload, sizeof(payload));
    length = 0xA5U;
    assert(!proto_v2_pack_safety_command(&safety, payload, sizeof(payload), &length));
    assert(length == 0xA5U && memcmp(payload, before, sizeof(payload)) == 0);
    safety.subsystem_mask = 0x01U;
    assert(proto_v2_pack_safety_command(&safety, payload, sizeof(payload), &length));
    payload[1] = 0x81U;
    memset(&decoded_safety, 0x5A, sizeof(decoded_safety));
    assert(!proto_v2_unpack_safety_command(payload, 2U, &decoded_safety));
    assert(decoded_safety.operation == 0x5AU && decoded_safety.subsystem_mask == 0x5AU);

    memset(&decoded_hello, 0x5A, sizeof(decoded_hello));
    assert(!proto_v2_unpack_hello(payload, 7U, &decoded_hello));
    assert(decoded_hello.host_build_id == 0x5A5A5A5AU);
    memset(&decoded_ack, 0x5A, sizeof(decoded_ack));
    assert(!proto_v2_unpack_hello_ack(payload, 14U, &decoded_ack));
    assert(decoded_ack.board_id == 0x5A5AU);

    memset(payload, 0xA5, sizeof(payload));
    memcpy(before, payload, sizeof(payload));
    length = 0xA5U;
    drive.fault_bitmap = 0x00000800U;
    assert(!proto_v2_pack_drive_state(&drive, payload, sizeof(payload), &length));
    assert(length == 0xA5U && memcmp(payload, before, sizeof(payload)) == 0);
    drive.fault_bitmap = 0U;
    assert(proto_v2_pack_drive_state(&drive, payload, sizeof(payload), &length));
    payload[2] = 2U;
    memset(&decoded_drive, 0x5A, sizeof(decoded_drive));
    assert(!proto_v2_unpack_drive_state(payload, length, &decoded_drive));
    assert(decoded_drive.mode == 0x5AU);

    memset(payload, 0xA5, sizeof(payload));
    memcpy(before, payload, sizeof(payload));
    length = 0xA5U;
    arm.measured_source = 0U;
    arm.measured_valid_mask = 0x40U;
    assert(!proto_v2_pack_arm_state(&arm, 0U, payload, sizeof(payload), &length));
    assert(length == 0xA5U && memcmp(payload, before, sizeof(payload)) == 0);
    arm.measured_valid_mask = 0U;
    arm.measured_source = 1U;
    assert(!proto_v2_pack_arm_state(&arm, 0U, payload, sizeof(payload), &length));
    arm.measured_source = 0U;
    arm.axis_count = 7U;
    assert(!proto_v2_pack_arm_state(&arm, 0U, payload, sizeof(payload), &length));
    assert(length == 0xA5U && memcmp(payload, before, sizeof(payload)) == 0);
    arm.axis_count = 6U;
    assert(proto_v2_pack_arm_state(&arm, 0U, payload, sizeof(payload), &length));
    payload[4] = 0x40U;
    memset(&decoded_arm, 0x5A, sizeof(decoded_arm));
    assert(!proto_v2_unpack_arm_state(payload, length, 0U, &decoded_arm));
    assert(decoded_arm.axis_count == 0x5AU);

    memset(payload, 0xA5, sizeof(payload));
    memcpy(before, payload, sizeof(payload));
    length = 0xA5U;
    fault.subsystem = 4U;
    assert(!proto_v2_pack_fault_event(&fault, payload, sizeof(payload), &length));
    fault.subsystem = 1U;
    fault.fault_bitmap = 0x00000800U;
    assert(!proto_v2_pack_fault_event(&fault, payload, sizeof(payload), &length));
    assert(length == 0xA5U && memcmp(payload, before, sizeof(payload)) == 0);
    fault.fault_bitmap = 0U;
    assert(proto_v2_pack_fault_event(&fault, payload, sizeof(payload), &length));
    payload[6] = 0U;
    memset(&decoded_fault, 0x5A, sizeof(decoded_fault));
    assert(!proto_v2_unpack_fault_event(payload, length, &decoded_fault));
    assert(decoded_fault.subsystem == 0x5AU);

    {
        const uint8_t invalid_time[] = {0U};
        uint64_t unchanged_request = 0xA5A5A5A5A5A5A5A5ULL;
        proto_v2_time_sync_response_t unchanged_response;
        memset(&unchanged_response, 0x5A, sizeof(unchanged_response));
        assert(!proto_v2_unpack_time_sync_request(invalid_time, sizeof(invalid_time),
                                                  &unchanged_request));
        assert(unchanged_request == 0xA5A5A5A5A5A5A5A5ULL);
        assert(!proto_v2_unpack_time_sync_response(invalid_time,
                                                   sizeof(invalid_time),
                                                   &unchanged_response));
        assert(unchanged_response.host_send_monotonic_ns ==
               0x5A5A5A5A5A5A5A5AULL);
    }
}

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

    assert(PROTO_V2_FEATURE_DRIVE_WHEELS == 0x01U);
    assert(PROTO_V2_FEATURE_ARM_POSITION == 0x02U);
    assert(PROTO_V2_FEATURE_WHEEL_ODOMETRY == 0x04U);
    assert(PROTO_V2_REQUIRED_FEATURES == 0x07U);
    assert(proto_v2_required_features_supported(0x07U));
    assert(proto_v2_required_features_supported(0x27U));
    assert(!proto_v2_required_features_supported(0x06U));
    assert(!proto_v2_required_features_supported(0x05U));
    assert(!proto_v2_required_features_supported(0x03U));
    assert(proto_v2_sequence_is_newer(0U, 0xFFFFU));
    assert(proto_v2_sequence_is_newer(0x7FFFU, 0U));
    assert(!proto_v2_sequence_is_newer(0x8000U, 0U));
    assert(!proto_v2_sequence_is_newer(0xFFFFU, 0U));
    assert(!proto_v2_sequence_is_newer(14U, 14U));

    test_golden_frames();
    test_invalid_frames();
    test_command_boundaries();
    test_control_payload_codecs();
    test_state_payload_codecs();
    test_diagnostic_payload_codecs();
    test_new_payload_rejections_are_transactional();

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
    assert(decoded_arm.position_urad[6] == 0);

    {
        proto_v2_drive_command_t drive_boundary = {
            .mode = 1U, .lease_ms = 10U, .valid_mask = 0x0FU,
            .wheel_velocity_urad_s = {0, 0, 0, 0}
        };
        proto_v2_arm_command_t arm_boundary = {
            .mode = 1U, .lease_ms = 200U, .axis_count = 6U, .valid_mask = 0x3FU,
            .position_urad = {0, 0, 0, 0, 0, 0, 0}
        };
        assert(proto_v2_pack_drive(&drive_boundary, payload, sizeof(payload),
                                   &payload_length));
        assert(proto_v2_unpack_drive(payload, payload_length, &decoded_drive));
        assert(decoded_drive.lease_ms == 10U);
        assert(proto_v2_pack_arm(&arm_boundary, 0U, payload, sizeof(payload),
                                 &payload_length));
        assert(proto_v2_unpack_arm(payload, payload_length, 0U, &decoded_arm));
        assert(decoded_arm.lease_ms == 200U);
    }

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

    {
        uint8_t invalid_drive[20] = {1U, 100U, 0U, 0U};
        proto_v2_drive_command_t unchanged = {
            .mode = 7U,
            .lease_ms = 77U,
            .valid_mask = 0xA5U,
            .wheel_velocity_urad_s = {11, 22, 33, 44}
        };
        assert(!proto_v2_unpack_drive(invalid_drive, sizeof(invalid_drive), &unchanged));
        assert(unchanged.mode == 7U);
        assert(unchanged.lease_ms == 77U);
        assert(unchanged.valid_mask == 0xA5U);
        assert(unchanged.wheel_velocity_urad_s[0] == 11);
        assert(unchanged.wheel_velocity_urad_s[3] == 44);
    }

    {
        uint8_t invalid_arm[29] = {1U, 100U, 0U, 6U, 0U};
        uint8_t untouched_payload[33];
        proto_v2_arm_command_t unchanged;
        proto_v2_arm_command_t invalid_disarmed = {
            .mode = 0U, .lease_ms = 100U, .axis_count = 6U, .valid_mask = 0U,
            .position_urad = {1, 0, 0, 0, 0, 0, 0}
        };
        memset(&unchanged, 0x5A, sizeof(unchanged));
        assert(!proto_v2_unpack_arm(invalid_arm, sizeof(invalid_arm), 0U, &unchanged));
        {
            const uint8_t sentinel = 0xA5U;
            memset(untouched_payload, sentinel, sizeof(untouched_payload));
            assert(!proto_v2_pack_arm(&invalid_disarmed, 0U, untouched_payload,
                                      sizeof(untouched_payload), &payload_length));
            for (size_t i = 0U; i < sizeof(untouched_payload); ++i)
                assert(untouched_payload[i] == sentinel);
        }
        {
            static const uint8_t v1_wire[] = {
                0x04U, 0x01U, 0x01U, 0x01U, 0x01U, 0x01U, 0x03U, 0x55U, 0x97U, 0x00U
            };
            assert(!proto_v2_decode(v1_wire, sizeof(v1_wire), &decoded));
            assert(!proto_v2_decode(v1_wire, sizeof(v1_wire) - 1U, &decoded));
        }
    }

    {
        static const uint8_t unknown_type_wire[] = {
            0x04U, 0x02U, 0x7FU, 0x01U, 0x01U, 0x01U, 0x03U, 0x9BU, 0x8BU, 0x00U
        };
        assert(!proto_v2_decode(unknown_type_wire, sizeof(unknown_type_wire), &decoded));
        frame.type = 0x7FU;
        frame.payload_length = 0U;
        assert(!proto_v2_encode(&frame, wire, sizeof(wire), &wire_length));
    }
    return 0;
}
