#include "app_serial_session.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void send_frame(serial_v2_session_t *session, unsigned port,
                       uint8_t type, uint16_t sequence,
                       const uint8_t *payload, size_t payload_length,
                       uint64_t now_us)
{
    proto_v2_frame_t frame = {
        .version = PROTO_V2_VERSION,
        .type = type,
        .sequence = sequence,
        .payload_length = (uint16_t)payload_length
    };
    uint8_t wire[PROTO_V2_MAX_WIRE];
    size_t wire_length = 0U;
    if (payload_length != 0U)
        memcpy(frame.payload, payload, payload_length);
    assert(proto_v2_encode(&frame, wire, sizeof(wire), &wire_length));
    size_t split = wire_length / 2U;
    serial_v2_session_feed(session, port, wire, split, now_us);
    serial_v2_session_feed(session, port, wire + split, wire_length - split,
                           now_us);
}

static void send_hello(serial_v2_session_t *session, unsigned port,
                       uint16_t sequence, uint32_t features,
                       uint64_t now_us)
{
    proto_v2_hello_t hello = {0x10203040U, features};
    uint8_t payload[8];
    size_t length = 0U;
    assert(proto_v2_pack_hello(&hello, payload, sizeof(payload), &length));
    send_frame(session, port, PROTO_V2_TYPE_HELLO, sequence, payload, length,
               now_us);
}

static void drain_tx(serial_v2_session_t *session, unsigned port)
{
    uint8_t wire[PROTO_V2_MAX_WIRE];
    size_t length = 0U;
    while (serial_v2_session_pop_tx(session, port, wire, sizeof(wire), &length))
        ;
}

static int pop_type(serial_v2_session_t *session, unsigned port, uint8_t type,
                    proto_v2_frame_t *decoded)
{
    uint8_t wire[PROTO_V2_MAX_WIRE];
    size_t length = 0U;
    while (serial_v2_session_pop_tx(session, port, wire, sizeof(wire), &length)) {
        proto_v2_frame_t frame;
        assert(proto_v2_decode(wire, length, &frame));
        if (frame.type == type) {
            if (decoded != NULL) *decoded = frame;
            return 1;
        }
    }
    return 0;
}

static void test_required_capabilities_and_boot_handshake(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    uint8_t wire[PROTO_V2_MAX_WIRE];
    size_t length = 0U;
    proto_v2_frame_t frame;
    proto_v2_hello_ack_t ack;

    serial_v2_session_init(&session, 0xA1B2C3D4U, 0x0407U, 0x00010203U);
    send_hello(&session, 0U, 0xFFFEU, PROTO_V2_REQUIRED_FEATURES |
               PROTO_V2_FEATURE_DEVICE_TIME_SYNC, 1000U);
    assert(serial_v2_session_pop_tx(&session, 0U, wire, sizeof(wire), &length));
    assert(proto_v2_decode(wire, length, &frame));
    assert(frame.type == PROTO_V2_TYPE_HELLO_ACK);
    assert(proto_v2_unpack_hello_ack(frame.payload, frame.payload_length, &ack));
    assert(ack.boot_id == 0xA1B2C3D4U);
    assert(ack.accepted_features == (PROTO_V2_REQUIRED_FEATURES |
                                     PROTO_V2_FEATURE_DEVICE_TIME_SYNC));
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.handshake_complete);
    assert(!snapshot.drive_armed && !snapshot.arm_armed);

    send_hello(&session, 1U, 1U, PROTO_V2_REQUIRED_FEATURES, 2000U);
    assert(!serial_v2_session_pop_tx(&session, 1U, wire, sizeof(wire), &length));
    assert(!serial_v2_session_pop_tx(&session, 0U, wire, sizeof(wire), &length));
}

static void test_incomplete_features_and_v1_are_rejected(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    static const uint8_t v1_wire[] = {
        0x04U, 0x01U, 0x01U, 0x01U, 0x01U,
        0x01U, 0x03U, 0x55U, 0x97U, 0x00U
    };

    serial_v2_session_init(&session, 1U, 0x0407U, 1U);
    send_hello(&session, 0U, 4U, PROTO_V2_FEATURE_DRIVE_WHEELS |
               PROTO_V2_FEATURE_ARM_POSITION, 1000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.handshake_complete);
    serial_v2_session_feed(&session, 0U, v1_wire, sizeof(v1_wire), 2000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.handshake_complete);
    assert(snapshot.rx_error_count != 0U);
}

static void test_drive_requires_imu_calibration_and_fresh_feedback(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_drive_command_t command = {
        .mode = 1U, .lease_ms = 100U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {1000000, -1000000, 500000, 0}
    };
    uint8_t payload[20];
    size_t length = 0U;

    serial_v2_session_init(&session, 1U, 0x0407U, 1U);
    send_hello(&session, 0U, 10U, PROTO_V2_REQUIRED_FEATURES, 1000U);
    drain_tx(&session, 0U);
    assert(proto_v2_pack_drive(&command, payload, sizeof(payload), &length));

    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 11U, payload,
               length, 2000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);
    assert((snapshot.fault_bitmap & (1UL << 5)) != 0U);

    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x07U, .sample_time_us = 2500U
    });
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 12U, payload,
               length, 3000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);

    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x0FU, .sample_time_us = 3500U
    });
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 13U, payload,
               length, 4000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.drive_armed);
    assert(snapshot.drive_lease_last_refresh_us == 4000U);
    assert(snapshot.drive_command_sequence == 13U);
    assert(snapshot.wheel_target_urad_s[0] == 1000000);
    assert(snapshot.wheel_target_urad_s[1] == -1000000);

    serial_v2_session_set_drive_feedback(&session,
        &(proto_v2_drive_state_t){0});
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);
    assert((snapshot.fault_bitmap & (1UL << 5)) != 0U);
}

static void test_independent_leases_and_arm_commissioning_lock(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_drive_command_t drive = {
        .mode = 1U, .lease_ms = 100U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    proto_v2_arm_command_t arm = {
        .mode = 1U, .lease_ms = 200U, .axis_count = 6U, .valid_mask = 0x3FU,
        .position_urad = {1000, 2000, 3000, 4000, 5000, 6000}
    };
    uint8_t payload[33];
    size_t length = 0U;

    serial_v2_session_init(&session, 2U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x0FU, .sample_time_us = 1U
    });
    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_ARM_COMMAND, 2U, payload,
               length, 5000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.arm_armed);
    serial_v2_session_set_arm_commissioned(&session, 1U);
    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 3U, payload,
               length, 10000U);
    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_ARM_COMMAND, 4U, payload,
               length, 10000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.drive_armed);
    assert(snapshot.arm_armed);

    serial_v2_session_tick(&session, 110000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);
    assert(snapshot.arm_armed);
    assert((snapshot.fault_bitmap & (1UL << 1)) != 0U);

    serial_v2_session_tick(&session, 210000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.arm_armed);
}

static void test_bad_sequence_does_not_refresh_drive_lease(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_drive_command_t command = {
        .mode = 1U, .lease_ms = 50U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    uint8_t payload[20];
    size_t length = 0U;

    serial_v2_session_init(&session, 3U, 0x0407U, 1U);
    send_hello(&session, 0U, 20U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x0FU, .sample_time_us = 1U
    });
    assert(proto_v2_pack_drive(&command, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 21U, payload,
               length, 1000U);
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 21U, payload,
               length, 40000U);
    serial_v2_session_tick(&session, 51000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);
    assert((snapshot.fault_bitmap & (1UL << 1)) != 0U);
}

static void test_rx_overrun_cannot_refresh_command_leases(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_drive_command_t command = {
        .mode = 1U, .lease_ms = 50U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {1000000, 0, 0, 0}
    };
    proto_v2_arm_command_t arm = {
        .mode = 1U, .lease_ms = 50U, .axis_count = 6U, .valid_mask = 0x3FU,
        .position_urad = {1000, 2000, 3000, 4000, 5000, 6000}
    };
    uint8_t payload[33];
    size_t length = 0U;

    serial_v2_session_init(&session, 10U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x0FU, .sample_time_us = 1U
    });
    serial_v2_session_set_arm_commissioned(&session, 1U);
    assert(proto_v2_pack_drive(&command, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 2U, payload,
               length, 1000U);
    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_ARM_COMMAND, 3U, payload,
               length, 1000U);

    serial_v2_session_note_rx_overrun(&session, 0U);
    assert(proto_v2_pack_drive(&command, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 4U, payload,
               length, 40000U);
    serial_v2_session_note_rx_overrun(&session, 0U);
    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_ARM_COMMAND, 5U, payload,
               length, 40000U);
    serial_v2_session_tick(&session, 51000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed && !snapshot.arm_armed);
    assert((snapshot.fault_bitmap & (1UL << 1)) != 0U);

    assert(proto_v2_pack_drive(&command, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 4U, payload,
               length, 52000U);
    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_ARM_COMMAND, 5U, payload,
               length, 52000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.drive_armed && snapshot.arm_armed);
}

static void test_safety_stop_latch_and_clear_transitions(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_drive_command_t drive = {
        .mode = 1U, .lease_ms = 100U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    proto_v2_safety_command_t safety = {
        PROTO_V2_SAFETY_STOP, PROTO_V2_SUBSYSTEM_DRIVE
    };
    uint8_t payload[20];
    size_t length = 0U;

    serial_v2_session_init(&session, 11U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x0FU, .sample_time_us = 1U
    });
    serial_v2_session_set_estop_clear_allowed(&session, 1U);
    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 2U, payload,
               length, 1000U);
    assert(proto_v2_pack_safety_command(&safety, payload, sizeof(payload),
                                        &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 3U, payload,
               length, 2000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed && !snapshot.safety_inhibited);
    assert(snapshot.safety_clear_generation == 0U);

    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 4U, payload,
               length, 3000U);
    assert(proto_v2_pack_safety_command(&(proto_v2_safety_command_t){
        PROTO_V2_SAFETY_LATCH_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE
    }, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 5U, payload,
               length, 4000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed && snapshot.safety_inhibited);

    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 6U, payload,
               length, 5000U);
    assert(proto_v2_pack_safety_command(&(proto_v2_safety_command_t){
        PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE
    }, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 7U, payload,
               length, 6000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed && !snapshot.safety_inhibited);
    assert(snapshot.safety_clear_generation == 1U);

    proto_v2_drive_command_t disarmed = {
        .mode = 0U, .lease_ms = 100U, .valid_mask = 0U,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    assert(proto_v2_pack_drive(&disarmed, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 8U, payload,
               length, 7000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);

    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 9U, payload,
               length, 8000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.drive_armed);
    assert(snapshot.safety_clear_generation == 1U);
}

static void test_arm_commissioning_lock_survives_safety_clear(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_arm_command_t arm = {
        .mode = 1U, .lease_ms = 100U, .axis_count = 6U, .valid_mask = 0x3FU,
        .position_urad = {1000, 2000, 3000, 4000, 5000, 6000}
    };
    uint8_t payload[33];
    size_t length = 0U;

    serial_v2_session_init(&session, 12U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    assert(proto_v2_pack_safety_command(&(proto_v2_safety_command_t){
        PROTO_V2_SAFETY_LATCH_INHIBIT, PROTO_V2_SUBSYSTEM_ARM
    }, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 2U, payload,
               length, 1000U);
    assert(proto_v2_pack_safety_command(&(proto_v2_safety_command_t){
        PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_ARM
    }, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 3U, payload,
               length, 2000U);
    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_ARM_COMMAND, 4U, payload,
               length, 3000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.arm_armed && !snapshot.safety_inhibited);
    assert(snapshot.safety_clear_generation == 0U);
}

static void test_drive_clear_requires_fault_gate_and_discards_both_targets(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_drive_command_t drive = {
        .mode = 1U, .lease_ms = 100U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {1000, 2000, 3000, 4000}
    };
    proto_v2_arm_command_t arm = {
        .mode = 1U, .lease_ms = 100U, .axis_count = 6U, .valid_mask = 0x3FU,
        .position_urad = {1000, 2000, 3000, 4000, 5000, 6000}
    };
    uint8_t payload[40];
    size_t length = 0U;

    serial_v2_session_init(&session, 44U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x0FU, .sample_time_us = 1U
    });
    serial_v2_session_set_arm_commissioned(&session, 1U);

    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 2U, payload,
               length, 1000U);
    assert(proto_v2_pack_arm(&arm, 0U, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_ARM_COMMAND, 3U, payload,
               length, 2000U);
    assert(proto_v2_pack_safety_command(&(proto_v2_safety_command_t){
        PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE
    }, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 4U, payload,
               length, 3000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.safety_inhibited);
    assert(snapshot.safety_clear_generation == 0U);
    assert(!snapshot.drive_armed && !snapshot.arm_armed);
    for (unsigned i = 0U; i < 4U; ++i)
        assert(snapshot.wheel_target_urad_s[i] == 0);
    for (unsigned i = 0U; i < 7U; ++i)
        assert(snapshot.arm_command_position_urad[i] == 0);

    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 5U, payload,
               length, 4000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.safety_inhibited);
    assert(!snapshot.drive_armed);

    serial_v2_session_set_estop_clear_allowed(&session, 1U);
    assert(proto_v2_pack_safety_command(&(proto_v2_safety_command_t){
        PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE
    }, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 6U, payload,
               length, 5000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.safety_inhibited);
    assert(snapshot.safety_clear_generation == 1U);
    assert(!snapshot.drive_armed && !snapshot.arm_armed);

    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 5U, payload,
               length, 6000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);

    proto_v2_drive_command_t disarmed = {
        .mode = 0U, .lease_ms = 100U, .valid_mask = 0U,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    assert(proto_v2_pack_drive(&disarmed, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 7U, payload,
               length, 7000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);

    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 8U, payload,
               length, 8000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.drive_armed);
}

static void test_drive_clear_requires_later_disarmed_confirmation(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_drive_command_t armed = {
        .mode = 1U, .lease_ms = 100U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {1000, 2000, 3000, 4000}
    };
    proto_v2_drive_command_t disarmed = {
        .mode = 0U, .lease_ms = 100U, .valid_mask = 0U,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    proto_v2_safety_command_t clear = {
        PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE
    };
    proto_v2_frame_t clear_frame = {
        .version = PROTO_V2_VERSION,
        .type = PROTO_V2_TYPE_SAFETY_COMMAND,
        .sequence = 3U
    };
    proto_v2_frame_t premature_drive_frame = {
        .version = PROTO_V2_VERSION,
        .type = PROTO_V2_TYPE_DRIVE_COMMAND,
        .sequence = 4U
    };
    uint8_t payload[20];
    uint8_t clear_wire[PROTO_V2_MAX_WIRE];
    uint8_t drive_wire[PROTO_V2_MAX_WIRE];
    uint8_t batch[PROTO_V2_MAX_WIRE * 2U];
    size_t payload_length = 0U;
    size_t clear_length = 0U;
    size_t drive_length = 0U;

    serial_v2_session_init(&session, 46U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x0FU, .sample_time_us = 1U
    });
    serial_v2_session_set_estop_clear_allowed(&session, 1U);

    assert(proto_v2_pack_drive(&armed, payload, sizeof(payload),
                               &payload_length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 2U, payload,
               payload_length, 10000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.drive_armed);
    assert(snapshot.drive_lease_last_refresh_us == 10000U);

    assert(proto_v2_pack_safety_command(&clear, clear_frame.payload,
                                        sizeof(clear_frame.payload),
                                        &payload_length));
    clear_frame.payload_length = (uint16_t)payload_length;
    assert(proto_v2_encode(&clear_frame, clear_wire, sizeof(clear_wire),
                           &clear_length));
    assert(proto_v2_pack_drive(&armed, premature_drive_frame.payload,
                               sizeof(premature_drive_frame.payload),
                               &payload_length));
    premature_drive_frame.payload_length = (uint16_t)payload_length;
    assert(proto_v2_encode(&premature_drive_frame, drive_wire,
                           sizeof(drive_wire), &drive_length));
    memcpy(batch, clear_wire, clear_length);
    memcpy(batch + clear_length, drive_wire, drive_length);
    serial_v2_session_feed(&session, 0U, batch, clear_length + drive_length,
                           20000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.safety_clear_generation == 1U);
    assert(!snapshot.drive_armed);
    assert(snapshot.drive_command_sequence == 2U);
    assert(snapshot.drive_lease_last_refresh_us == 10000U);
    for (unsigned i = 0U; i < 4U; ++i)
        assert(snapshot.wheel_target_urad_s[i] == 0);

    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 2U, payload,
               payload_length, 20500U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);
    assert(snapshot.drive_lease_last_refresh_us == 10000U);

    assert(proto_v2_pack_drive(&disarmed, payload, sizeof(payload),
                               &payload_length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 5U, payload,
               payload_length, 21000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);
    assert(snapshot.drive_lease_last_refresh_us == 21000U);

    serial_v2_session_set_drive_prerequisites(&session, 0U, 1U, 1U);
    assert(proto_v2_pack_drive(&armed, payload, sizeof(payload),
                               &payload_length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 6U, payload,
               payload_length, 22000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);
    assert(snapshot.drive_lease_last_refresh_us == 21000U);

    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 7U, payload,
               payload_length, 23000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.drive_armed);
    assert(snapshot.drive_lease_last_refresh_us == 23000U);
}

static void test_car_guard_failure_reinhibits_the_usb_session(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    uint8_t payload[8];
    size_t length = 0U;

    serial_v2_session_init(&session, 45U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_estop_clear_allowed(&session, 1U);
    assert(proto_v2_pack_safety_command(&(proto_v2_safety_command_t){
        PROTO_V2_SAFETY_LATCH_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE
    }, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 2U, payload,
               length, 1000U);
    assert(proto_v2_pack_safety_command(&(proto_v2_safety_command_t){
        PROTO_V2_SAFETY_CLEAR_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE
    }, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 3U, payload,
               length, 2000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.safety_inhibited);
    assert(snapshot.safety_clear_generation == 1U);

    serial_v2_session_reject_drive_clear(&session);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.safety_inhibited);
    assert(!snapshot.drive_armed);
    assert((snapshot.fault_bitmap & (1UL << 0)) != 0U);
}

static void test_non_owner_port_cannot_control_owner_session(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_drive_command_t drive = {
        .mode = 1U, .lease_ms = 100U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    uint8_t payload[20];
    size_t length = 0U;

    serial_v2_session_init(&session, 13U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x0FU, .sample_time_us = 1U
    });
    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 2U, payload,
               length, 1000U);
    send_hello(&session, 1U, 1U, PROTO_V2_REQUIRED_FEATURES, 2000U);
    assert(proto_v2_pack_safety_command(&(proto_v2_safety_command_t){
        PROTO_V2_SAFETY_LATCH_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE
    }, payload, sizeof(payload), &length));
    send_frame(&session, 1U, PROTO_V2_TYPE_SAFETY_COMMAND, 2U, payload,
               length, 3000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.owner_valid && snapshot.owner_port == 0U);
    assert(snapshot.drive_armed && !snapshot.safety_inhibited);
}

static void test_disarmed_feedback_is_never_fabricated(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;

    serial_v2_session_init(&session, 4U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    serial_v2_session_tick(&session, 10000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);
    assert(snapshot.drive_feedback_valid_mask == 0U);
    assert(snapshot.sample_time_us == 0U);
    for (unsigned i = 0U; i < 4U; ++i) {
        assert(snapshot.wheel_position_urad[i] == 0);
        assert(snapshot.wheel_velocity_urad_s[i] == 0);
    }
    assert(snapshot.arm_measured_valid_mask == 0U);
    assert(snapshot.arm_measured_source == 0U);
}

static void test_sequence_wrap_and_invalid_crc(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_drive_command_t command = {
        .mode = 1U, .lease_ms = 100U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    uint8_t payload[20];
    uint8_t wire[PROTO_V2_MAX_WIRE];
    size_t length = 0U;

    serial_v2_session_init(&session, 5U, 0x0407U, 1U);
    send_hello(&session, 0U, 0xFFFEU, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x0FU, .sample_time_us = 1U
    });
    assert(proto_v2_pack_drive(&command, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 0xFFFFU, payload,
               length, 1000U);
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 0U, payload,
               length, 2000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.drive_armed);
    assert(snapshot.drive_command_sequence == 0U);

    proto_v2_frame_t frame = {
        .version = PROTO_V2_VERSION,
        .type = PROTO_V2_TYPE_DRIVE_COMMAND,
        .sequence = 1U,
        .payload_length = sizeof(payload)
    };
    memcpy(frame.payload, payload, sizeof(payload));
    assert(proto_v2_encode(&frame, wire, sizeof(wire), &length));
    wire[length - 2U] ^= 0x40U;
    serial_v2_session_feed(&session, 0U, wire, length, 3000U);
    serial_v2_session_tick(&session, 102001U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.drive_armed);
}

static void test_safety_and_time_sync_dispatch(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_safety_command_t safety = {
        PROTO_V2_SAFETY_LATCH_INHIBIT, PROTO_V2_SUBSYSTEM_DRIVE
    };
    proto_v2_frame_t response;
    uint8_t payload[24];
    size_t length = 0U;
    uint64_t host_time = 0x1122334455667788ULL;

    serial_v2_session_init(&session, 6U, 0x0407U, 1U);
    send_hello(&session, 0U, 10U, PROTO_V2_REQUIRED_FEATURES |
               PROTO_V2_FEATURE_DEVICE_TIME_SYNC, 1000U);
    drain_tx(&session, 0U);

    assert(proto_v2_pack_time_sync_request(host_time, payload, 8U, &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_TIME_SYNC_REQUEST, 11U, payload,
               length, 2500U);
    assert(pop_type(&session, 0U, PROTO_V2_TYPE_TIME_SYNC_RESPONSE, &response));
    proto_v2_time_sync_response_t time_response;
    assert(proto_v2_unpack_time_sync_response(response.payload,
                                               response.payload_length,
                                               &time_response));
    assert(time_response.host_send_monotonic_ns == host_time);
    assert(time_response.device_rx_uptime_us == 2500U);
    assert(time_response.device_tx_uptime_us >= time_response.device_rx_uptime_us);

    assert(proto_v2_pack_safety_command(&safety, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 12U, payload,
               length, 3000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.safety_inhibited);
    assert(!snapshot.drive_armed);

    safety.operation = PROTO_V2_SAFETY_CLEAR_INHIBIT;
    serial_v2_session_set_estop_clear_allowed(&session, 1U);
    assert(proto_v2_pack_safety_command(&safety, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 13U, payload,
               length, 4000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(!snapshot.safety_inhibited);
}

static void test_arm_inhibit_does_not_block_drive_subsystem(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    proto_v2_safety_command_t safety = {
        PROTO_V2_SAFETY_LATCH_INHIBIT, PROTO_V2_SUBSYSTEM_ARM
    };
    proto_v2_drive_command_t drive = {
        .mode = 1U, .lease_ms = 100U, .valid_mask = 0x0FU,
        .wheel_velocity_urad_s = {0, 0, 0, 0}
    };
    uint8_t payload[20];
    size_t length = 0U;

    serial_v2_session_init(&session, 9U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_drive_prerequisites(&session, 1U, 1U, 1U);
    serial_v2_session_set_drive_feedback(&session, &(proto_v2_drive_state_t){
        .feedback_valid_mask = 0x0FU, .sample_time_us = 1U
    });
    assert(proto_v2_pack_safety_command(&safety, payload, sizeof(payload),
                                        &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_SAFETY_COMMAND, 2U, payload,
               length, 1000U);
    assert(proto_v2_pack_drive(&drive, payload, sizeof(payload), &length));
    send_frame(&session, 0U, PROTO_V2_TYPE_DRIVE_COMMAND, 3U, payload,
               length, 2000U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.safety_inhibited);
    assert(snapshot.drive_armed);
}

static void test_state_and_diagnostics_transmit(void)
{
    serial_v2_session_t session;
    proto_v2_frame_t frame;
    proto_v2_drive_state_t drive;
    proto_v2_arm_state_t arm;
    proto_v2_diagnostics_t diagnostics;

    serial_v2_session_init(&session, 7U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES, 0U);
    drain_tx(&session, 0U);
    serial_v2_session_set_diagnostics(&session, 12000U, -125);
    serial_v2_session_tick(&session, 10000U);
    assert(pop_type(&session, 0U, PROTO_V2_TYPE_DRIVE_STATE, &frame));
    assert(proto_v2_unpack_drive_state(frame.payload, frame.payload_length, &drive));
    assert(drive.mode == 0U && drive.feedback_valid_mask == 0U);
    assert(pop_type(&session, 0U, PROTO_V2_TYPE_ARM_STATE, &frame));
    assert(proto_v2_unpack_arm_state(frame.payload, frame.payload_length,
                                     PROTO_V2_REQUIRED_FEATURES, &arm));
    assert(arm.mode == 0U && arm.measured_valid_mask == 0U);
    assert(arm.measured_source == 0U);

    serial_v2_session_tick(&session, 1000000U);
    assert(pop_type(&session, 0U, PROTO_V2_TYPE_DIAGNOSTICS, &frame));
    assert(proto_v2_unpack_diagnostics(frame.payload, frame.payload_length,
                                       &diagnostics));
    assert(diagnostics.battery_mv == 12000U);
    assert(diagnostics.board_temp_centi_c == -125);
}

static void test_tx_queue_is_bounded(void)
{
    serial_v2_session_t session;
    serial_v2_snapshot_t snapshot;
    uint8_t payload[8];
    size_t length = 0U;

    serial_v2_session_init(&session, 8U, 0x0407U, 1U);
    send_hello(&session, 0U, 1U, PROTO_V2_REQUIRED_FEATURES |
               PROTO_V2_FEATURE_DEVICE_TIME_SYNC, 0U);
    drain_tx(&session, 0U);
    assert(proto_v2_pack_time_sync_request(1U, payload, sizeof(payload), &length));
    for (uint16_t sequence = 2U; sequence < 12U; ++sequence)
        send_frame(&session, 0U, PROTO_V2_TYPE_TIME_SYNC_REQUEST, sequence,
                   payload, length, (uint64_t)sequence * 100U);
    serial_v2_session_snapshot(&session, &snapshot);
    assert(snapshot.tx_drop_count == 2U);
}

int main(void)
{
    test_required_capabilities_and_boot_handshake();
    test_incomplete_features_and_v1_are_rejected();
    test_drive_requires_imu_calibration_and_fresh_feedback();
    test_independent_leases_and_arm_commissioning_lock();
    test_bad_sequence_does_not_refresh_drive_lease();
    test_rx_overrun_cannot_refresh_command_leases();
    test_safety_stop_latch_and_clear_transitions();
    test_drive_clear_requires_fault_gate_and_discards_both_targets();
    test_drive_clear_requires_later_disarmed_confirmation();
    test_car_guard_failure_reinhibits_the_usb_session();
    test_arm_commissioning_lock_survives_safety_clear();
    test_non_owner_port_cannot_control_owner_session();
    test_disarmed_feedback_is_never_fabricated();
    test_sequence_wrap_and_invalid_crc();
    test_safety_and_time_sync_dispatch();
    test_arm_inhibit_does_not_block_drive_subsystem();
    test_state_and_diagnostics_transmit();
    test_tx_queue_is_bounded();
    puts("UART v2 session 测试通过");
    return 0;
}
