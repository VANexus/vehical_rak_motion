#ifndef APP_WHEEL_CONTROL_H
#define APP_WHEEL_CONTROL_H

#include <stdatomic.h>
#include <stdint.h>

#define WHEEL_COUNT 4U
#define WHEEL_SAMPLE_MAX_AGE_MS 20U
#define WHEEL_URAD_PER_REV 6283185LL

typedef struct {
    int32_t target_counts_ms[WHEEL_COUNT];
    uint32_t command_tick;
    uint8_t source;
    uint8_t enabled;
} wheel_target_snapshot_t;

typedef struct {
    atomic_uchar slot_bytes[2][sizeof(wheel_target_snapshot_t)];
    atomic_uint_least32_t slot_generation[2];
    atomic_uint_least32_t active_slot;
} wheel_target_mailbox_t;

typedef struct {
    uint32_t last_tick;
    uint32_t wrap_count;
    uint64_t last_uptime_us;
    uint8_t initialized;
} wheel_uptime_clock_t;

typedef enum {
    WHEEL_SAMPLE_INVALID = 0,
    WHEEL_SAMPLE_FUTURE = 1,
    WHEEL_SAMPLE_FRESH = 2,
    WHEEL_SAMPLE_STALE = 3
} wheel_sample_age_t;

typedef struct {
    uint32_t counts_per_revolution[WHEEL_COUNT];
    uint32_t wheel_radius_um;
    uint32_t rotational_lever_um;
    int8_t encoder_direction[WHEEL_COUNT];
    uint8_t valid;
} wheel_calibration_t;

extern const wheel_calibration_t car_wheel_calibration;

int wheel_control_calibration_valid(const wheel_calibration_t *calibration);
wheel_sample_age_t wheel_control_sample_age(uint32_t now_ms,
                                            uint32_t sample_ms,
                                            uint8_t sample_valid);
uint8_t wheel_control_sample_fresh(uint32_t now_ms, uint32_t sample_ms,
                                   uint8_t sample_valid);
uint8_t wheel_control_sample_fresh_from_clock(uint32_t (*read_tick)(void),
                                              uint32_t sample_ms,
                                              uint8_t sample_valid);
uint64_t wheel_control_uptime_us(wheel_uptime_clock_t *clock,
                                 uint32_t tick_ms);
void wheel_target_mailbox_init(wheel_target_mailbox_t *mailbox);
void wheel_target_mailbox_publish(wheel_target_mailbox_t *mailbox,
                                  const wheel_target_snapshot_t *snapshot);
uint8_t wheel_target_mailbox_read(const wheel_target_mailbox_t *mailbox,
                                  wheel_target_snapshot_t *snapshot);
uint8_t wheel_control_encoder_generation_stable(uint32_t before,
                                                uint32_t after);
void wheel_control_encoder_generation_write_begin(
    atomic_uint_least32_t *generation);
void wheel_control_encoder_generation_write_end(
    atomic_uint_least32_t *generation);
uint8_t wheel_control_encoder_state(
    const int32_t raw_position_counts[WHEEL_COUNT],
    const int32_t normalized_speed_counts_ms[WHEEL_COUNT],
    uint8_t sampled_mask, const wheel_calibration_t *calibration,
    int64_t position_urad[WHEEL_COUNT],
    int32_t velocity_urad_s[WHEEL_COUNT], uint8_t *valid_mask);
uint8_t wheel_control_normalized_counts_to_urad_s(
    const int32_t target_counts_ms[WHEEL_COUNT],
    const wheel_calibration_t *calibration,
    int32_t target_urad_s[WHEEL_COUNT]);
uint8_t wheel_control_command_to_counts(
    const int32_t wheel_velocity_urad_s[WHEEL_COUNT],
    const wheel_calibration_t *calibration,
    int32_t target_counts_ms[WHEEL_COUNT]);
uint8_t wheel_control_stabilize(
    int32_t wheel_target_urad_s[WHEEL_COUNT],
    const int32_t measured_wheel_urad_s[WHEEL_COUNT],
    int32_t imu_yaw_rate_mrad_s,
    const wheel_calibration_t *calibration,
    uint32_t max_correction_urad_s, uint16_t gain_permille,
    int32_t correction_urad_s[WHEEL_COUNT]);
uint8_t wheel_control_common_saturate(
    int32_t wheel_target[WHEEL_COUNT], int32_t max_abs_target);

#endif
