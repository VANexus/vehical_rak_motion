#ifndef APP_SERIAL_SESSION_H
#define APP_SERIAL_SESSION_H

#include "app_protocol_v2.h"

#include <stddef.h>
#include <stdint.h>

#define SERIAL_V2_PORT_COUNT 2U
#define SERIAL_V2_TX_QUEUE_DEPTH 8U
#define SERIAL_V2_TX_FRAME_MAX PROTO_V2_MAX_WIRE
#define SERIAL_V2_SESSION_STORAGE_SIZE 4096U

typedef union {
    max_align_t alignment;
    uint8_t opaque[SERIAL_V2_SESSION_STORAGE_SIZE];
} serial_v2_session_t;

typedef struct {
    uint8_t handshake_complete;
    uint8_t owner_valid;
    uint8_t owner_port;
    uint8_t drive_armed;
    uint8_t arm_armed;
    uint8_t safety_inhibited;
    uint8_t drive_feedback_valid_mask;
    uint8_t arm_command_valid_mask;   /* [AI] R42: ARM 命令有效轴掩码（落地舵机用） */
    uint8_t arm_measured_valid_mask;
    uint8_t arm_measured_source;
    uint32_t accepted_features;
    uint32_t fault_bitmap;
    uint32_t safety_latch_generation;
    uint32_t safety_clear_generation;
    uint32_t encoder_epoch;
    uint64_t sample_time_us;
    uint64_t drive_lease_last_refresh_us;
    uint16_t drive_command_sequence;
    uint16_t arm_command_sequence;
    uint16_t tx_drop_count;
    uint16_t rx_error_count;
    int64_t wheel_position_urad[4];
    int32_t wheel_velocity_urad_s[4];
    int32_t wheel_target_urad_s[4];
    int32_t arm_command_position_urad[7];
    uint16_t battery_mv;
    int16_t board_temp_centi_c;
} serial_v2_snapshot_t;

void serial_v2_session_init(serial_v2_session_t *session, uint32_t boot_id,
                            uint16_t board_id, uint32_t firmware_version);
void serial_v2_session_feed(serial_v2_session_t *session, unsigned port,
                            const uint8_t *data, size_t length,
                            uint64_t now_us);
void serial_v2_session_set_drive_prerequisites(serial_v2_session_t *session,
                                                uint8_t imu_ready,
                                                uint8_t calibration_ready,
                                                uint8_t feedback_fresh);
void serial_v2_session_set_external_faults(serial_v2_session_t *session,
                                            uint32_t fault_bitmap);
void serial_v2_session_set_estop_clear_allowed(serial_v2_session_t *session,
                                                uint8_t allowed);
void serial_v2_session_reject_drive_clear(serial_v2_session_t *session);
void serial_v2_session_set_arm_commissioned(serial_v2_session_t *session,
                                             uint8_t commissioned);
void serial_v2_session_set_drive_feedback(
    serial_v2_session_t *session, const proto_v2_drive_state_t *feedback);
void serial_v2_session_set_diagnostics(serial_v2_session_t *session,
                                         uint16_t battery_mv,
                                         int16_t board_temp_centi_c);
void serial_v2_session_note_rx_overrun(serial_v2_session_t *session,
                                       unsigned port);
void serial_v2_session_note_tx_drop(serial_v2_session_t *session);
void serial_v2_session_tick(serial_v2_session_t *session, uint64_t now_us);
int serial_v2_session_pop_tx(serial_v2_session_t *session, unsigned port,
                             uint8_t *wire, size_t capacity, size_t *length);
void serial_v2_session_snapshot(const serial_v2_session_t *session,
                                serial_v2_snapshot_t *snapshot);

#endif
