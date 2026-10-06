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
#define PROTO_V2_FEATURE_DRIVE_WHEELS   (1UL << 0)
#define PROTO_V2_FEATURE_ARM_POSITION   (1UL << 1)
#define PROTO_V2_FEATURE_WHEEL_ODOMETRY (1UL << 2)
#define PROTO_V2_FEATURE_ARM_MEASURED_POSITION (1UL << 3)
#define PROTO_V2_FEATURE_GRIPPER_SERVO  (1UL << 4)
#define PROTO_V2_FEATURE_DEVICE_TIME_SYNC (1UL << 5)
#define PROTO_V2_REQUIRED_FEATURES      (PROTO_V2_FEATURE_DRIVE_WHEELS | \
                                         PROTO_V2_FEATURE_ARM_POSITION | \
                                         PROTO_V2_FEATURE_WHEEL_ODOMETRY)
#define PROTO_V2_KNOWN_FEATURES         0x3FU
#define PROTO_V2_KNOWN_FAULTS           0x000007FFUL
#define PROTO_V2_SAFETY_STOP            1U
#define PROTO_V2_SAFETY_LATCH_INHIBIT   2U
#define PROTO_V2_SAFETY_CLEAR_INHIBIT   3U
#define PROTO_V2_SUBSYSTEM_DRIVE        0x01U
#define PROTO_V2_SUBSYSTEM_ARM          0x02U
#define PROTO_V2_SUBSYSTEM_SYSTEM       0x03U

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

typedef struct {
    uint32_t host_build_id;
    uint32_t requested_features;
} proto_v2_hello_t;

typedef struct {
    uint16_t board_id;
    uint32_t firmware_version;
    uint32_t boot_id;
    uint32_t accepted_features;
} proto_v2_hello_ack_t;

typedef struct {
    uint64_t host_send_monotonic_ns;
    uint64_t device_rx_uptime_us;
    uint64_t device_tx_uptime_us;
} proto_v2_time_sync_response_t;

typedef struct {
    uint8_t operation;
    uint8_t subsystem_mask;
} proto_v2_safety_command_t;

typedef struct {
    uint16_t echoed_command_sequence;
    uint8_t mode;
    uint8_t feedback_valid_mask;
    uint32_t fault_bitmap;
    uint32_t encoder_epoch;
    uint64_t sample_time_us;
    int64_t wheel_position_urad[4];
    int32_t wheel_velocity_urad_s[4];
} proto_v2_drive_state_t;

typedef struct {
    uint16_t echoed_command_sequence;
    uint8_t mode;
    uint8_t axis_count;
    uint8_t measured_valid_mask;
    uint8_t measured_source;
    uint32_t fault_bitmap;
    uint64_t sample_time_us;
    int32_t command_position_urad[7];
    int32_t measured_position_urad[7];
} proto_v2_arm_state_t;

typedef struct {
    uint64_t device_uptime_us;
    uint16_t battery_mv;
    int16_t board_temp_centi_c;
    uint32_t crc_error_count;
    uint32_t sequence_error_count;
    uint32_t rx_overrun_count;
    uint32_t tx_drop_count;
    uint32_t control_deadline_miss_count;
    uint32_t reset_count;
} proto_v2_diagnostics_t;

typedef struct {
    uint32_t fault_bitmap;
    uint16_t source_command_sequence;
    uint8_t subsystem;
} proto_v2_fault_event_t;

uint16_t proto_v2_crc16(const uint8_t *data, size_t length);
int proto_v2_required_features_supported(uint32_t accepted_features);
int proto_v2_sequence_is_newer(uint16_t candidate, uint16_t previous);
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
int proto_v2_pack_hello(const proto_v2_hello_t *hello, uint8_t *payload,
                        size_t capacity, size_t *length);
int proto_v2_unpack_hello(const uint8_t *payload, size_t length,
                          proto_v2_hello_t *hello);
int proto_v2_pack_hello_ack(const proto_v2_hello_ack_t *ack, uint8_t *payload,
                            size_t capacity, size_t *length);
int proto_v2_unpack_hello_ack(const uint8_t *payload, size_t length,
                              proto_v2_hello_ack_t *ack);
int proto_v2_pack_time_sync_request(uint64_t host_send_monotonic_ns,
                                    uint8_t *payload, size_t capacity,
                                    size_t *length);
int proto_v2_unpack_time_sync_request(const uint8_t *payload, size_t length,
                                      uint64_t *host_send_monotonic_ns);
int proto_v2_pack_time_sync_response(const proto_v2_time_sync_response_t *response,
                                     uint8_t *payload, size_t capacity,
                                     size_t *length);
int proto_v2_unpack_time_sync_response(const uint8_t *payload, size_t length,
                                       proto_v2_time_sync_response_t *response);
int proto_v2_pack_safety_command(const proto_v2_safety_command_t *command,
                                 uint8_t *payload, size_t capacity,
                                 size_t *length);
int proto_v2_unpack_safety_command(const uint8_t *payload, size_t length,
                                   proto_v2_safety_command_t *command);
int proto_v2_pack_drive_state(const proto_v2_drive_state_t *state,
                              uint8_t *payload, size_t capacity,
                              size_t *length);
int proto_v2_unpack_drive_state(const uint8_t *payload, size_t length,
                                proto_v2_drive_state_t *state);
int proto_v2_pack_arm_state(const proto_v2_arm_state_t *state,
                            uint32_t accepted_features, uint8_t *payload,
                            size_t capacity, size_t *length);
int proto_v2_unpack_arm_state(const uint8_t *payload, size_t length,
                              uint32_t accepted_features,
                              proto_v2_arm_state_t *state);
int proto_v2_pack_diagnostics(const proto_v2_diagnostics_t *diagnostics,
                              uint8_t *payload, size_t capacity,
                              size_t *length);
int proto_v2_unpack_diagnostics(const uint8_t *payload, size_t length,
                                proto_v2_diagnostics_t *diagnostics);
int proto_v2_pack_fault_event(const proto_v2_fault_event_t *event,
                              uint8_t *payload, size_t capacity,
                              size_t *length);
int proto_v2_unpack_fault_event(const uint8_t *payload, size_t length,
                                proto_v2_fault_event_t *event);

#endif
