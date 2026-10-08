#include "app_serial_session.h"

#include <string.h>

#define V2_SESSION_MAGIC 0x56325332UL
#define V2_LEASE_FAULT   (1UL << 1)
#define V2_WHEEL_FAULT   (1UL << 5)
#define V2_INHIBIT_FAULT (1UL << 0)
#define V2_STATE_PERIOD_US 10000ULL
#define V2_DIAG_PERIOD_US 1000000ULL

typedef struct {
    uint16_t length;
    uint8_t data[SERIAL_V2_TX_FRAME_MAX];
} serial_v2_tx_frame_t;

typedef struct {
    uint32_t magic;
    uint32_t boot_id;
    uint32_t firmware_version;
    uint16_t board_id;
    uint16_t tx_sequence;
    uint16_t rx_sequence;
    uint8_t rx_sequence_valid;
    uint8_t owner_valid;
    uint8_t owner_port;
    uint8_t handshake_complete;
    uint8_t drive_armed;
    uint8_t arm_armed;
    uint8_t drive_inhibited;
    uint8_t arm_inhibited;
    uint8_t drive_rearm_confirmation_required;
    uint8_t imu_ready;
    uint8_t calibration_ready;
    uint8_t feedback_fresh;
    uint8_t arm_commissioned;
    uint8_t estop_clear_allowed;
    uint32_t accepted_features;
    uint32_t fault_bitmap;
    uint32_t external_fault_bitmap;
    uint32_t safety_latch_generation;
    uint32_t safety_clear_generation;
    uint32_t fault_reported;
    uint32_t crc_error_count;
    uint32_t sequence_error_count;
    uint32_t rx_overrun_count;
    uint32_t tx_drop_count;
    uint32_t encoder_epoch;
    uint64_t sample_time_us;
    uint64_t drive_last_us;
    uint64_t arm_last_us;
    uint64_t last_state_us;
    uint64_t last_diag_us;
    uint16_t drive_lease_ms;
    uint16_t arm_lease_ms;
    uint16_t drive_command_sequence;
    uint16_t arm_command_sequence;
    uint16_t drive_clear_sequence;
    uint16_t battery_mv;
    int16_t board_temp_centi_c;
    uint8_t drive_feedback_valid_mask;
    int64_t wheel_position_urad[4];
    int32_t wheel_velocity_urad_s[4];
    int32_t wheel_target_urad_s[4];
    uint8_t arm_axis_count;
    uint8_t arm_valid_mask;
    int32_t arm_command_position_urad[7];
    uint8_t rx_wire_length[SERIAL_V2_PORT_COUNT];
    uint8_t rx_overflow[SERIAL_V2_PORT_COUNT];
    uint8_t rx_discard_until_delimiter[SERIAL_V2_PORT_COUNT];
    uint8_t rx_wire[SERIAL_V2_PORT_COUNT][PROTO_V2_MAX_WIRE];
    uint8_t tx_head[SERIAL_V2_PORT_COUNT];
    uint8_t tx_tail[SERIAL_V2_PORT_COUNT];
    uint8_t tx_count[SERIAL_V2_PORT_COUNT];
    serial_v2_tx_frame_t tx_queue[SERIAL_V2_PORT_COUNT][SERIAL_V2_TX_QUEUE_DEPTH];
} serial_v2_session_state_t;

_Static_assert(sizeof(serial_v2_session_state_t) <= SERIAL_V2_SESSION_STORAGE_SIZE,
               "serial v2 session storage is too small");

static serial_v2_session_state_t *session_state(serial_v2_session_t *session)
{
    return (serial_v2_session_state_t *)(void *)session->opaque;
}

static const serial_v2_session_state_t *session_state_const(
    const serial_v2_session_t *session)
{
    return (const serial_v2_session_state_t *)(const void *)session->opaque;
}

static int session_is_valid(const serial_v2_session_state_t *state)
{
    return state != NULL && state->magic == V2_SESSION_MAGIC;
}

static void queue_wire(serial_v2_session_state_t *state, unsigned port,
                       const uint8_t *wire, size_t length)
{
    uint8_t slot;
    if (port >= SERIAL_V2_PORT_COUNT || length > SERIAL_V2_TX_FRAME_MAX)
        return;
    if (state->tx_count[port] >= SERIAL_V2_TX_QUEUE_DEPTH) {
        state->tx_drop_count++;
        return;
    }
    slot = state->tx_head[port];
    memcpy(state->tx_queue[port][slot].data, wire, length);
    state->tx_queue[port][slot].length = (uint16_t)length;
    state->tx_head[port] =
        (uint8_t)((slot + 1U) % SERIAL_V2_TX_QUEUE_DEPTH);
    state->tx_count[port]++;
}

static void send_frame(serial_v2_session_state_t *state, uint8_t type,
                       const uint8_t *payload, size_t payload_length,
                       uint64_t now_us)
{
    proto_v2_frame_t frame = {
        .version = PROTO_V2_VERSION,
        .type = type,
        .sequence = state->tx_sequence++,
        .payload_length = (uint16_t)payload_length
    };
    uint8_t wire[PROTO_V2_MAX_WIRE];
    size_t wire_length = 0U;
    if (!state->owner_valid || payload_length > PROTO_V2_MAX_PAYLOAD)
        return;
    if (payload_length != 0U)
        memcpy(frame.payload, payload, payload_length);
    (void)now_us;
    if (proto_v2_encode(&frame, wire, sizeof(wire), &wire_length))
        queue_wire(state, state->owner_port, wire, wire_length);
}

static void send_fault_event(serial_v2_session_state_t *state,
                             uint32_t faults, uint8_t subsystem,
                             uint16_t source_sequence, uint64_t now_us)
{
    proto_v2_fault_event_t event = {
        .fault_bitmap = faults,
        .source_command_sequence = source_sequence,
        .subsystem = subsystem
    };
    uint8_t payload[7];
    size_t length = 0U;
    if (faults == 0U ||
        !proto_v2_pack_fault_event(&event, payload, sizeof(payload), &length))
        return;
    send_frame(state, PROTO_V2_TYPE_FAULT_EVENT, payload, length, now_us);
}

static uint32_t effective_fault_bitmap(const serial_v2_session_state_t *state)
{
    return state->fault_bitmap | state->external_fault_bitmap;
}

static void update_fault_events(serial_v2_session_state_t *state,
                                uint64_t now_us)
{
    uint32_t faults = effective_fault_bitmap(state);
    uint32_t rising = faults & ~state->fault_reported;
    if (rising != 0U) {
        uint8_t subsystem = PROTO_V2_SUBSYSTEM_SYSTEM;
        if ((rising & V2_INHIBIT_FAULT) != 0U) {
            subsystem = (uint8_t)((state->drive_inhibited
                ? PROTO_V2_SUBSYSTEM_DRIVE : 0U) |
                (state->arm_inhibited ? PROTO_V2_SUBSYSTEM_ARM : 0U));
        } else if ((rising & (V2_WHEEL_FAULT | V2_LEASE_FAULT)) != 0U) {
            subsystem = PROTO_V2_SUBSYSTEM_DRIVE;
        }
        if (subsystem == 0U)
            subsystem = PROTO_V2_SUBSYSTEM_SYSTEM;
        send_fault_event(state, rising, subsystem,
                         state->drive_command_sequence, now_us);
    }
    state->fault_reported = faults;
}

static void clear_drive_command(serial_v2_session_state_t *state)
{
    state->drive_armed = 0U;
    memset(state->wheel_target_urad_s, 0,
           sizeof(state->wheel_target_urad_s));
}

static void clear_arm_command(serial_v2_session_state_t *state)
{
    state->arm_armed = 0U;
    state->arm_valid_mask = 0U;
    memset(state->arm_command_position_urad, 0,
           sizeof(state->arm_command_position_urad));
}

static int all_drive_prerequisites_ready(const serial_v2_session_state_t *state)
{
    /* [AI] R34: mask==0x0F 不再作为 ARM 前置——未初始化轮由 ISR 开环 kick
     * 学习极性（vmask 随学习进度上升到 0x0F）；保留 IMU/标定/采样时间前置。 */
    return state->imu_ready != 0U && state->calibration_ready != 0U &&
           state->sample_time_us != 0U;
}

static int accept_hello(serial_v2_session_state_t *state, unsigned port,
                        const proto_v2_frame_t *frame, uint64_t now_us)
{
    proto_v2_hello_t hello;
    proto_v2_hello_ack_t ack;
    uint8_t payload[14];
    size_t length = 0U;
    uint32_t accepted = PROTO_V2_REQUIRED_FEATURES;

    if (!proto_v2_unpack_hello(frame->payload, frame->payload_length, &hello) ||
        !proto_v2_required_features_supported(hello.requested_features))
        return 0;
    if (state->owner_valid && state->owner_port != port)
        return 0;
    if ((hello.requested_features & PROTO_V2_FEATURE_DEVICE_TIME_SYNC) != 0U)
        accepted |= PROTO_V2_FEATURE_DEVICE_TIME_SYNC;

    state->owner_valid = 1U;
    state->owner_port = (uint8_t)port;
    state->handshake_complete = 1U;
    state->accepted_features = accepted;
    state->rx_sequence = frame->sequence;
    state->rx_sequence_valid = 1U;
    clear_drive_command(state);
    clear_arm_command(state);
    state->drive_last_us = now_us;
    state->arm_last_us = now_us;
    state->last_state_us = now_us;
    state->last_diag_us = now_us;
    state->fault_bitmap &= ~V2_LEASE_FAULT;

    ack.board_id = state->board_id;
    ack.firmware_version = state->firmware_version;
    ack.boot_id = state->boot_id;
    ack.accepted_features = accepted;
    if (!proto_v2_pack_hello_ack(&ack, payload, sizeof(payload), &length))
        return 0;
    send_frame(state, PROTO_V2_TYPE_HELLO_ACK, payload, length, now_us);
    return 1;
}

static int process_drive_command(serial_v2_session_state_t *state,
                                 const proto_v2_frame_t *frame,
                                 uint64_t now_us)
{
    proto_v2_drive_command_t command;
    uint32_t drive_faults;
    if (!proto_v2_unpack_drive(frame->payload, frame->payload_length, &command))
        return 0;
    if (command.mode == 0U) {
        clear_drive_command(state);
        memset(state->wheel_target_urad_s, 0,
               sizeof(state->wheel_target_urad_s));
        if (state->drive_rearm_confirmation_required == 0U ||
            proto_v2_sequence_is_newer(frame->sequence,
                                       state->drive_clear_sequence)) {
            state->drive_command_sequence = frame->sequence;
            state->drive_last_us = now_us;
            state->drive_rearm_confirmation_required = 0U;
        }
        return 1;
    }
    if (state->drive_rearm_confirmation_required != 0U) {
        clear_drive_command(state);
        return 1;
    }
    state->drive_command_sequence = frame->sequence;
    drive_faults = effective_fault_bitmap(state);
    if (!state->drive_inhibited && state->arm_inhibited)
        drive_faults &= ~V2_INHIBIT_FAULT;
    if (state->drive_inhibited || state->external_fault_bitmap != 0U ||
        (drive_faults &
         ~(V2_WHEEL_FAULT | V2_LEASE_FAULT)) != 0U ||
        !all_drive_prerequisites_ready(state)) {
        clear_drive_command(state);
        memset(state->wheel_target_urad_s, 0,
               sizeof(state->wheel_target_urad_s));
        state->fault_bitmap |= V2_WHEEL_FAULT;
        return 1;
    }
    state->fault_bitmap &= ~V2_WHEEL_FAULT;
    state->fault_bitmap &= ~V2_LEASE_FAULT;
    state->drive_armed = 1U;
    state->drive_lease_ms = command.lease_ms;
    state->drive_last_us = now_us;
    memcpy(state->wheel_target_urad_s, command.wheel_velocity_urad_s,
           sizeof(state->wheel_target_urad_s));
    return 1;
}

static int process_arm_command(serial_v2_session_state_t *state,
                               const proto_v2_frame_t *frame,
                               uint64_t now_us)
{
    proto_v2_arm_command_t command;
    if (!proto_v2_unpack_arm(frame->payload, frame->payload_length,
                             state->accepted_features, &command))
        return 0;
    state->arm_command_sequence = frame->sequence;
    if (command.mode == 0U) {
        clear_arm_command(state);
        state->arm_axis_count = command.axis_count;
        state->arm_last_us = now_us;
        return 1;
    }
    if (!state->arm_commissioned || state->arm_inhibited ||
        state->external_fault_bitmap != 0U)
        return 1;
    state->arm_armed = 1U;
    state->arm_lease_ms = command.lease_ms;
    state->arm_last_us = now_us;
    state->arm_axis_count = command.axis_count;
    state->arm_valid_mask = command.valid_mask;
    memcpy(state->arm_command_position_urad, command.position_urad,
           sizeof(state->arm_command_position_urad));
    return 1;
}

static int process_safety_command(serial_v2_session_state_t *state,
                                  const proto_v2_frame_t *frame,
                                  uint64_t now_us)
{
    proto_v2_safety_command_t command;
    if (!proto_v2_unpack_safety_command(frame->payload, frame->payload_length,
                                        &command))
        return 0;
    if ((command.subsystem_mask & PROTO_V2_SUBSYSTEM_DRIVE) != 0U) {
        clear_drive_command(state);
        if (command.operation == PROTO_V2_SAFETY_LATCH_INHIBIT) {
            state->drive_inhibited = 1U;
            state->fault_bitmap |= V2_INHIBIT_FAULT;
            state->safety_latch_generation++;
        } else if (command.operation == PROTO_V2_SAFETY_CLEAR_INHIBIT) {
            clear_arm_command(state);
            if (state->estop_clear_allowed != 0U) {
                state->drive_inhibited = 0U;
                state->drive_rearm_confirmation_required = 1U;
                state->drive_clear_sequence = frame->sequence;
                state->safety_clear_generation++;
                if (!state->arm_inhibited)
                    state->fault_bitmap &= ~V2_INHIBIT_FAULT;
            } else {
                state->drive_inhibited = 1U;
                state->fault_bitmap |= V2_INHIBIT_FAULT;
            }
        }
    }
    if ((command.subsystem_mask & PROTO_V2_SUBSYSTEM_ARM) != 0U) {
        clear_arm_command(state);
        if (command.operation == PROTO_V2_SAFETY_LATCH_INHIBIT) {
            state->arm_inhibited = 1U;
            state->fault_bitmap |= V2_INHIBIT_FAULT;
        } else if (command.operation == PROTO_V2_SAFETY_CLEAR_INHIBIT) {
            state->arm_inhibited = 0U;
            if (!state->drive_inhibited)
                state->fault_bitmap &= ~V2_INHIBIT_FAULT;
        }
    }
    (void)now_us;
    return 1;
}

static int process_frame(serial_v2_session_state_t *state, unsigned port,
                         const uint8_t *wire, size_t wire_length,
                         uint64_t now_us)
{
    proto_v2_frame_t frame;
    if (!proto_v2_decode(wire, wire_length, &frame)) {
        state->crc_error_count++;
        return 0;
    }
    if (frame.type == PROTO_V2_TYPE_HELLO)
        return accept_hello(state, port, &frame, now_us);
    if (!state->handshake_complete || !state->owner_valid ||
        state->owner_port != port)
        return 0;
    if (!state->rx_sequence_valid ||
        !proto_v2_sequence_is_newer(frame.sequence, state->rx_sequence)) {
        state->sequence_error_count++;
        return 0;
    }

    int accepted = 0;
    switch (frame.type) {
    case PROTO_V2_TYPE_TIME_SYNC_REQUEST: {
        uint64_t host_send_ns;
        proto_v2_time_sync_response_t response;
        uint8_t payload[24];
        size_t length = 0U;
        if ((state->accepted_features &
             PROTO_V2_FEATURE_DEVICE_TIME_SYNC) == 0U ||
            !proto_v2_unpack_time_sync_request(frame.payload,
                                                frame.payload_length,
                                                &host_send_ns))
            break;
        response.host_send_monotonic_ns = host_send_ns;
        response.device_rx_uptime_us = now_us;
        response.device_tx_uptime_us = now_us;
        if (!proto_v2_pack_time_sync_response(&response, payload,
                                               sizeof(payload), &length))
            break;
        send_frame(state, PROTO_V2_TYPE_TIME_SYNC_RESPONSE, payload, length,
                   now_us);
        accepted = 1;
        break;
    }
    case PROTO_V2_TYPE_DRIVE_COMMAND:
        accepted = process_drive_command(state, &frame, now_us);
        break;
    case PROTO_V2_TYPE_ARM_COMMAND:
        accepted = process_arm_command(state, &frame, now_us);
        break;
    case PROTO_V2_TYPE_SAFETY_COMMAND:
        accepted = process_safety_command(state, &frame, now_us);
        break;
    case PROTO_V2_TYPE_FAULT_EVENT: {
        proto_v2_fault_event_t event;
        if (proto_v2_unpack_fault_event(frame.payload, frame.payload_length,
                                        &event)) {
            state->fault_bitmap |= event.fault_bitmap;
            if ((event.subsystem & PROTO_V2_SUBSYSTEM_DRIVE) != 0U)
                clear_drive_command(state);
            if ((event.subsystem & PROTO_V2_SUBSYSTEM_ARM) != 0U)
                clear_arm_command(state);
            accepted = 1;
        }
        break;
    }
    default:
        break;
    }
    if (accepted)
        state->rx_sequence = frame.sequence;
    return accepted;
}

static void consume_byte(serial_v2_session_state_t *state, unsigned port,
                         uint8_t byte, uint64_t now_us)
{
    uint8_t *length = &state->rx_wire_length[port];
    if (state->rx_discard_until_delimiter[port] != 0U) {
        if (byte == 0U)
            state->rx_discard_until_delimiter[port] = 0U;
        return;
    }
    if (byte == 0U) {
        if (*length != 0U && state->rx_overflow[port] == 0U) {
            uint8_t wire[PROTO_V2_MAX_WIRE];
            size_t wire_length = *length;
            memcpy(wire, state->rx_wire[port], wire_length);
            wire[wire_length++] = 0U;
            (void)process_frame(state, port, wire, wire_length, now_us);
        }
        if (state->rx_overflow[port] != 0U)
            state->rx_overrun_count++;
        *length = 0U;
        state->rx_overflow[port] = 0U;
        return;
    }
    if (state->rx_overflow[port] != 0U)
        return;
    if (*length >= PROTO_V2_MAX_WIRE - 1U) {
        state->rx_overflow[port] = 1U;
        return;
    }
    state->rx_wire[port][(*length)++] = byte;
}

void serial_v2_session_init(serial_v2_session_t *session, uint32_t boot_id,
                            uint16_t board_id, uint32_t firmware_version)
{
    serial_v2_session_state_t *state;
    if (session == NULL)
        return;
    memset(session, 0, sizeof(*session));
    state = session_state(session);
    state->boot_id = boot_id != 0U ? boot_id : 1U;
    state->board_id = board_id;
    state->firmware_version = firmware_version;
    state->tx_sequence = 1U;
    state->arm_axis_count = 6U;
    state->magic = V2_SESSION_MAGIC;
}

void serial_v2_session_feed(serial_v2_session_t *session, unsigned port,
                            const uint8_t *data, size_t length,
                            uint64_t now_us)
{
    serial_v2_session_state_t *state;
    if (session == NULL || data == NULL || port >= SERIAL_V2_PORT_COUNT)
        return;
    state = session_state(session);
    if (!session_is_valid(state))
        return;
    for (size_t i = 0U; i < length; ++i)
        consume_byte(state, port, data[i], now_us);
}

void serial_v2_session_set_drive_prerequisites(serial_v2_session_t *session,
                                                uint8_t imu_ready,
                                                uint8_t calibration_ready,
                                                uint8_t feedback_fresh)
{
    serial_v2_session_state_t *state;
    if (session == NULL)
        return;
    state = session_state(session);
    if (!session_is_valid(state))
        return;
    state->imu_ready = (uint8_t)(imu_ready != 0U);
    state->calibration_ready = (uint8_t)(calibration_ready != 0U);
    state->feedback_fresh = (uint8_t)(feedback_fresh != 0U);
    if (state->drive_armed && !all_drive_prerequisites_ready(state)) {
        clear_drive_command(state);
        memset(state->wheel_target_urad_s, 0,
               sizeof(state->wheel_target_urad_s));
        state->fault_bitmap |= V2_WHEEL_FAULT;
    }
}

void serial_v2_session_set_external_faults(serial_v2_session_t *session,
                                            uint32_t fault_bitmap)
{
    serial_v2_session_state_t *state;
    if (session == NULL)
        return;
    state = session_state(session);
    if (!session_is_valid(state))
        return;
    state->external_fault_bitmap = fault_bitmap & PROTO_V2_KNOWN_FAULTS;
    if (state->external_fault_bitmap != 0U) {
        clear_drive_command(state);
        memset(state->wheel_target_urad_s, 0,
               sizeof(state->wheel_target_urad_s));
        clear_arm_command(state);
        if ((state->external_fault_bitmap & V2_INHIBIT_FAULT) != 0U) {
            state->drive_inhibited = 1U;
            state->arm_inhibited = 1U;
            state->fault_bitmap |= V2_INHIBIT_FAULT;
        }
    }
}

void serial_v2_session_set_estop_clear_allowed(serial_v2_session_t *session,
                                                uint8_t allowed)
{
    serial_v2_session_state_t *state;
    if (session == NULL)
        return;
    state = session_state(session);
    if (session_is_valid(state))
        state->estop_clear_allowed = (uint8_t)(allowed != 0U);
}

void serial_v2_session_reject_drive_clear(serial_v2_session_t *session)
{
    serial_v2_session_state_t *state;
    if (session == NULL)
        return;
    state = session_state(session);
    if (!session_is_valid(state))
        return;
    clear_drive_command(state);
    clear_arm_command(state);
    state->drive_inhibited = 1U;
    state->fault_bitmap |= V2_INHIBIT_FAULT;
}

void serial_v2_session_set_arm_commissioned(serial_v2_session_t *session,
                                             uint8_t commissioned)
{
    serial_v2_session_state_t *state;
    if (session == NULL)
        return;
    state = session_state(session);
    if (session_is_valid(state))
        state->arm_commissioned = (uint8_t)(commissioned != 0U);
}

void serial_v2_session_set_drive_feedback(
    serial_v2_session_t *session, const proto_v2_drive_state_t *feedback)
{
    serial_v2_session_state_t *state;
    if (session == NULL || feedback == NULL)
        return;
    state = session_state(session);
    if (!session_is_valid(state))
        return;
    state->drive_feedback_valid_mask =
        (uint8_t)(feedback->feedback_valid_mask & 0x0FU);
    state->encoder_epoch = feedback->encoder_epoch;
    /* [AI] R35: 采样时间戳不再依赖 mask——mask=0（极性未学习）时清零时间戳
     * 会让 ARM 前置永远不满足，形成"不动→学不了→不动"死锁（R34 联测实测） */
    state->sample_time_us = feedback->sample_time_us;
    for (unsigned i = 0U; i < 4U; ++i) {
        if ((state->drive_feedback_valid_mask & (1U << i)) != 0U) {
            state->wheel_position_urad[i] = feedback->wheel_position_urad[i];
            state->wheel_velocity_urad_s[i] = feedback->wheel_velocity_urad_s[i];
        } else {
            state->wheel_position_urad[i] = 0;
            state->wheel_velocity_urad_s[i] = 0;
        }
    }
    if (state->drive_armed && state->sample_time_us == 0U) {
        /* [AI] R36: 只有采样时间戳缺失才算反馈失效；mask<0x0F 是极性学习期
         * 的正常状态（未初始化轮由 ISR kick 学习），不得据此撤销 ARM */
        state->fault_bitmap |= V2_WHEEL_FAULT;
        state->drive_rearm_confirmation_required = 1U;
        state->drive_armed = 0U;
    }
}

void serial_v2_session_set_diagnostics(serial_v2_session_t *session,
                                         uint16_t battery_mv,
                                         int16_t board_temp_centi_c)
{
    serial_v2_session_state_t *state;
    if (session == NULL)
        return;
    state = session_state(session);
    if (!session_is_valid(state))
        return;
    state->battery_mv = battery_mv;
    state->board_temp_centi_c = board_temp_centi_c;
}

void serial_v2_session_note_rx_overrun(serial_v2_session_t *session,
                                       unsigned port)
{
    serial_v2_session_state_t *state;
    if (session == NULL || port >= SERIAL_V2_PORT_COUNT)
        return;
    state = session_state(session);
    if (!session_is_valid(state))
        return;
    state->rx_overrun_count++;
    state->rx_wire_length[port] = 0U;
    state->rx_overflow[port] = 0U;
    state->rx_discard_until_delimiter[port] = 1U;
}

void serial_v2_session_note_tx_drop(serial_v2_session_t *session)
{
    serial_v2_session_state_t *state;
    if (session == NULL)
        return;
    state = session_state(session);
    if (session_is_valid(state))
        state->tx_drop_count++;
}

void serial_v2_session_tick(serial_v2_session_t *session, uint64_t now_us)
{
    serial_v2_session_state_t *state;
    if (session == NULL)
        return;
    state = session_state(session);
    if (!session_is_valid(state) || !state->handshake_complete)
        return;

    if (state->drive_armed &&
        now_us - state->drive_last_us >= (uint64_t)state->drive_lease_ms * 1000U) {
        clear_drive_command(state);
        state->fault_bitmap |= V2_LEASE_FAULT;
    }
    if (state->arm_armed &&
        now_us - state->arm_last_us >= (uint64_t)state->arm_lease_ms * 1000U)
        clear_arm_command(state);
    update_fault_events(state, now_us);

    if (now_us - state->last_state_us >= V2_STATE_PERIOD_US) {
        proto_v2_drive_state_t drive = {0};
        proto_v2_arm_state_t arm = {0};
        uint8_t payload[PROTO_V2_MAX_PAYLOAD];
        size_t length = 0U;

        drive.echoed_command_sequence = state->drive_command_sequence;
        drive.mode = state->drive_armed ? 1U : 0U;
        drive.feedback_valid_mask = state->drive_feedback_valid_mask;
        drive.fault_bitmap = effective_fault_bitmap(state);
        drive.encoder_epoch = state->encoder_epoch;
        drive.sample_time_us = state->sample_time_us;
        memcpy(drive.wheel_position_urad, state->wheel_position_urad,
               sizeof(drive.wheel_position_urad));
        memcpy(drive.wheel_velocity_urad_s, state->wheel_velocity_urad_s,
               sizeof(drive.wheel_velocity_urad_s));
        if (proto_v2_pack_drive_state(&drive, payload, sizeof(payload), &length))
            send_frame(state, PROTO_V2_TYPE_DRIVE_STATE, payload, length, now_us);

        arm.echoed_command_sequence = state->arm_command_sequence;
        arm.mode = state->arm_armed ? 1U : 0U;
        arm.axis_count = state->arm_axis_count;
        arm.measured_valid_mask = 0U;
        arm.measured_source = 0U;
        arm.fault_bitmap = effective_fault_bitmap(state);
        arm.sample_time_us = 0U;
        memcpy(arm.command_position_urad, state->arm_command_position_urad,
               sizeof(arm.command_position_urad));
        if (proto_v2_pack_arm_state(&arm, state->accepted_features, payload,
                                    sizeof(payload), &length))
            send_frame(state, PROTO_V2_TYPE_ARM_STATE, payload, length, now_us);
        state->last_state_us = now_us;
    }

    if (now_us - state->last_diag_us >= V2_DIAG_PERIOD_US) {
        proto_v2_diagnostics_t diagnostics = {
            .device_uptime_us = now_us,
            .battery_mv = state->battery_mv,
            .board_temp_centi_c = state->board_temp_centi_c,
            .crc_error_count = state->crc_error_count,
            .sequence_error_count = state->sequence_error_count,
            .rx_overrun_count = state->rx_overrun_count,
            .tx_drop_count = state->tx_drop_count,
            .control_deadline_miss_count = 0U,
            .reset_count = 0U
        };
        uint8_t payload[36];
        size_t length = 0U;
        if (proto_v2_pack_diagnostics(&diagnostics, payload, sizeof(payload),
                                     &length))
            send_frame(state, PROTO_V2_TYPE_DIAGNOSTICS, payload, length, now_us);
        state->last_diag_us = now_us;
    }
}

int serial_v2_session_pop_tx(serial_v2_session_t *session, unsigned port,
                             uint8_t *wire, size_t capacity, size_t *length)
{
    serial_v2_session_state_t *state;
    serial_v2_tx_frame_t *frame;
    if (session == NULL || wire == NULL || length == NULL ||
        port >= SERIAL_V2_PORT_COUNT)
        return 0;
    state = session_state(session);
    if (!session_is_valid(state) || state->tx_count[port] == 0U)
        return 0;
    frame = &state->tx_queue[port][state->tx_tail[port]];
    if (capacity < frame->length)
        return 0;
    memcpy(wire, frame->data, frame->length);
    *length = frame->length;
    state->tx_tail[port] =
        (uint8_t)((state->tx_tail[port] + 1U) % SERIAL_V2_TX_QUEUE_DEPTH);
    state->tx_count[port]--;
    return 1;
}

void serial_v2_session_snapshot(const serial_v2_session_t *session,
                                serial_v2_snapshot_t *snapshot)
{
    const serial_v2_session_state_t *state;
    if (session == NULL || snapshot == NULL)
        return;
    state = session_state_const(session);
    if (!session_is_valid(state))
        return;
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->handshake_complete = state->handshake_complete;
    snapshot->owner_valid = state->owner_valid;
    snapshot->owner_port = state->owner_port;
    snapshot->drive_armed = state->drive_armed;
    snapshot->arm_armed = state->arm_armed;
    snapshot->safety_inhibited =
        (uint8_t)(state->drive_inhibited || state->arm_inhibited);
    snapshot->drive_feedback_valid_mask = state->drive_feedback_valid_mask;
    snapshot->arm_command_valid_mask = state->arm_valid_mask;   /* [AI] R42 */
    snapshot->arm_measured_valid_mask = 0U;
    snapshot->arm_measured_source = 0U;
    snapshot->accepted_features = state->accepted_features;
    snapshot->fault_bitmap = effective_fault_bitmap(state);
    snapshot->safety_latch_generation = state->safety_latch_generation;
    snapshot->safety_clear_generation = state->safety_clear_generation;
    snapshot->encoder_epoch = state->encoder_epoch;
    snapshot->sample_time_us = state->sample_time_us;
    snapshot->drive_lease_last_refresh_us = state->drive_last_us;
    snapshot->drive_command_sequence = state->drive_command_sequence;
    snapshot->arm_command_sequence = state->arm_command_sequence;
    snapshot->tx_drop_count = (uint16_t)state->tx_drop_count;
    snapshot->rx_error_count = (uint16_t)(state->crc_error_count +
                                           state->sequence_error_count +
                                           state->rx_overrun_count);
    memcpy(snapshot->wheel_position_urad, state->wheel_position_urad,
           sizeof(snapshot->wheel_position_urad));
    memcpy(snapshot->wheel_velocity_urad_s, state->wheel_velocity_urad_s,
           sizeof(snapshot->wheel_velocity_urad_s));
    memcpy(snapshot->wheel_target_urad_s, state->wheel_target_urad_s,
           sizeof(snapshot->wheel_target_urad_s));
    memcpy(snapshot->arm_command_position_urad,
           state->arm_command_position_urad,
           sizeof(snapshot->arm_command_position_urad));
    snapshot->battery_mv = state->battery_mv;
    snapshot->board_temp_centi_c = state->board_temp_centi_c;
}
