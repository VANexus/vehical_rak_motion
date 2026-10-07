#include "app_wheel_control.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define WHEEL_URAD_S_PER_REV_MS (WHEEL_URAD_PER_REV * 1000LL)

/* [AI] R30: v2 首次实驱烟测标定表（车轮架空、低速、有急停前提下使用）。
 * 数值出处（均为历史台架/设计值，非本轮实测，待实车标定复核）：
 * - counts_per_revolution: 编码器 11PPR*4*30:1=61440，A 轮手测约 61602，B/C/D 未单独实测，
 *   暂用台架估算 61600（见 app_car.c LEGACY_BENCH_CPR_ESTIMATE 注释）。
 * - wheel_radius_um: 台架注释 2*pi*32.5mm（app_car.c IK 注释），未实测。
 * - rotational_lever_um: app_car.c ik_lw=150mm（半轴距+半轮距之和），未实测。
 * - encoder_direction: 与 app_car.c forward_dir 一致（历史实测的前进方向）。 */
const wheel_calibration_t car_wheel_calibration = {
    .counts_per_revolution = {61600U, 61600U, 61600U, 61600U},
    .wheel_radius_um = 32500U,
    .rotational_lever_um = 150000U,
    .encoder_direction = {1, -1, -1, 1},
    .valid = 1U
};

static int64_t clamp_i64(int64_t value, int64_t minimum, int64_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

int wheel_control_calibration_valid(const wheel_calibration_t *calibration)
{
    if (calibration == NULL || calibration->valid == 0U ||
        calibration->wheel_radius_um == 0U ||
        calibration->wheel_radius_um > 1000000U ||
        calibration->rotational_lever_um == 0U ||
        calibration->rotational_lever_um > 1000000U)
        return 0;

    for (unsigned i = 0U; i < WHEEL_COUNT; ++i) {
        if (calibration->counts_per_revolution[i] < 1000U ||
            calibration->counts_per_revolution[i] > 10000000U ||
            (calibration->encoder_direction[i] != 1 &&
             calibration->encoder_direction[i] != -1))
            return 0;
    }
    return 1;
}

uint8_t wheel_control_sample_fresh(uint32_t now_ms, uint32_t sample_ms,
                                   uint8_t sample_valid)
{
    return (uint8_t)(wheel_control_sample_age(now_ms, sample_ms,
                                              sample_valid) ==
                     WHEEL_SAMPLE_FRESH);
}

wheel_sample_age_t wheel_control_sample_age(uint32_t now_ms,
                                            uint32_t sample_ms,
                                            uint8_t sample_valid)
{
    if (sample_valid == 0U)
        return WHEEL_SAMPLE_INVALID;

    int32_t signed_age_ms = (int32_t)(now_ms - sample_ms);
    if (signed_age_ms < 0)
        return WHEEL_SAMPLE_FUTURE;
    if ((uint32_t)signed_age_ms > WHEEL_SAMPLE_MAX_AGE_MS)
        return WHEEL_SAMPLE_STALE;
    return WHEEL_SAMPLE_FRESH;
}

uint8_t wheel_control_sample_fresh_from_clock(uint32_t (*read_tick)(void),
                                              uint32_t sample_ms,
                                              uint8_t sample_valid)
{
    if (read_tick == NULL)
        return 0U;
    return wheel_control_sample_fresh(read_tick(), sample_ms, sample_valid);
}

uint64_t wheel_control_uptime_us(wheel_uptime_clock_t *clock,
                                 uint32_t tick_ms)
{
    if (clock == NULL)
        return 0U;

    uint32_t wrap_count = clock->wrap_count;
    if (clock->initialized == 0U) {
        clock->initialized = 1U;
        clock->last_tick = tick_ms;
    } else if (tick_ms < clock->last_tick) {
        if ((uint32_t)(clock->last_tick - tick_ms) > INT32_MAX) {
            clock->wrap_count++;
            clock->last_tick = tick_ms;
            wrap_count = clock->wrap_count;
        }
    } else {
        uint32_t forward_delta = tick_ms - clock->last_tick;
        if (forward_delta <= INT32_MAX) {
            clock->last_tick = tick_ms;
        } else if (clock->wrap_count != 0U) {
            wrap_count = clock->wrap_count - 1U;
        }
    }

    uint64_t uptime_us = (((uint64_t)wrap_count << 32) | tick_ms) * 1000ULL;
    if (uptime_us < clock->last_uptime_us)
        uptime_us = clock->last_uptime_us;
    clock->last_uptime_us = uptime_us;
    return uptime_us;
}

void wheel_target_mailbox_init(wheel_target_mailbox_t *mailbox)
{
    if (mailbox == NULL)
        return;

    for (unsigned slot = 0U; slot < 2U; ++slot) {
        for (unsigned byte = 0U;
             byte < sizeof(wheel_target_snapshot_t); ++byte)
            atomic_init(&mailbox->slot_bytes[slot][byte], 0U);
    }
    atomic_init(&mailbox->slot_generation[0], 0U);
    atomic_init(&mailbox->slot_generation[1], 0U);
    atomic_init(&mailbox->active_slot, 0U);
}

void wheel_target_mailbox_publish(wheel_target_mailbox_t *mailbox,
                                  const wheel_target_snapshot_t *snapshot)
{
    if (mailbox == NULL || snapshot == NULL)
        return;

    uint32_t active = atomic_load_explicit(&mailbox->active_slot,
                                           memory_order_relaxed) & 1U;
    uint32_t inactive = active ^ 1U;
    uint32_t generation = atomic_load_explicit(
        &mailbox->slot_generation[inactive], memory_order_relaxed);
    if ((generation & 1U) != 0U)
        generation++;

    atomic_store_explicit(&mailbox->slot_generation[inactive],
                          generation + 1U, memory_order_release);
    const unsigned char *source = (const unsigned char *)snapshot;
    for (unsigned byte = 0U;
         byte < sizeof(wheel_target_snapshot_t); ++byte)
        atomic_store_explicit(&mailbox->slot_bytes[inactive][byte],
                              source[byte], memory_order_relaxed);
    atomic_store_explicit(&mailbox->slot_generation[inactive],
                          generation + 2U, memory_order_release);
    atomic_store_explicit(&mailbox->active_slot, inactive,
                          memory_order_release);
}

uint8_t wheel_target_mailbox_read(const wheel_target_mailbox_t *mailbox,
                                  wheel_target_snapshot_t *snapshot)
{
    if (mailbox == NULL || snapshot == NULL)
        return 0U;

    for (unsigned attempt = 0U; attempt < 8U; ++attempt) {
        uint32_t active = atomic_load_explicit(&mailbox->active_slot,
                                               memory_order_acquire) & 1U;
        uint32_t before = atomic_load_explicit(
            &mailbox->slot_generation[active], memory_order_acquire);
        if ((before & 1U) != 0U)
            continue;

        wheel_target_snapshot_t candidate;
        unsigned char *destination = (unsigned char *)&candidate;
        for (unsigned byte = 0U;
             byte < sizeof(wheel_target_snapshot_t); ++byte)
            destination[byte] = atomic_load_explicit(
                &mailbox->slot_bytes[active][byte], memory_order_relaxed);
        atomic_thread_fence(memory_order_acquire);
        uint32_t after = atomic_load_explicit(
            &mailbox->slot_generation[active], memory_order_acquire);
        uint32_t active_after = atomic_load_explicit(&mailbox->active_slot,
                                                     memory_order_acquire) & 1U;
        if (before == after && (after & 1U) == 0U &&
            active == active_after) {
            *snapshot = candidate;
            return 1U;
        }
    }
    return 0U;
}

uint8_t wheel_control_encoder_generation_stable(uint32_t before,
                                                uint32_t after)
{
    return (uint8_t)(before == after && (before & 1U) == 0U);
}

void wheel_control_encoder_generation_write_begin(
    atomic_uint_least32_t *generation)
{
    if (generation != NULL)
        (void)atomic_fetch_add_explicit(generation, 1U, memory_order_acq_rel);
}

void wheel_control_encoder_generation_write_end(
    atomic_uint_least32_t *generation)
{
    if (generation != NULL)
        (void)atomic_fetch_add_explicit(generation, 1U, memory_order_release);
}

static int32_t counts_per_ms_to_urad_s(int64_t counts_per_ms,
                                        uint32_t counts_per_revolution)
{
    int64_t whole = counts_per_ms / counts_per_revolution;
    int64_t remainder = counts_per_ms % counts_per_revolution;
    int64_t converted =
        whole * WHEEL_URAD_S_PER_REV_MS +
        (remainder * WHEEL_URAD_S_PER_REV_MS) / counts_per_revolution;
    return (int32_t)clamp_i64(converted, INT32_MIN, INT32_MAX);
}

uint8_t wheel_control_normalized_counts_to_urad_s(
    const int32_t target_counts_ms[WHEEL_COUNT],
    const wheel_calibration_t *calibration,
    int32_t target_urad_s[WHEEL_COUNT])
{
    if (target_counts_ms == NULL || target_urad_s == NULL ||
        !wheel_control_calibration_valid(calibration))
        return 0U;

    for (unsigned i = 0U; i < WHEEL_COUNT; ++i) {
        target_urad_s[i] = counts_per_ms_to_urad_s(
            target_counts_ms[i], calibration->counts_per_revolution[i]);
    }
    return 1U;
}

uint8_t wheel_control_encoder_state(
    const int32_t raw_position_counts[WHEEL_COUNT],
    const int32_t normalized_speed_counts_ms[WHEEL_COUNT],
    uint8_t sampled_mask, const wheel_calibration_t *calibration,
    int64_t position_urad[WHEEL_COUNT],
    int32_t velocity_urad_s[WHEEL_COUNT], uint8_t *valid_mask)
{
    if (position_urad == NULL || velocity_urad_s == NULL ||
        valid_mask == NULL)
        return 0U;

    *valid_mask = 0U;
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i) {
        position_urad[i] = 0;
        velocity_urad_s[i] = 0;
    }
    if (raw_position_counts == NULL || normalized_speed_counts_ms == NULL ||
        !wheel_control_calibration_valid(calibration))
        return 0U;

    sampled_mask &= 0x0FU;
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i) {
        if ((sampled_mask & (uint8_t)(1U << i)) == 0U)
            continue;

        int64_t direction = calibration->encoder_direction[i];
        int64_t counts = (int64_t)raw_position_counts[i] * direction;
        position_urad[i] =
            counts * WHEEL_URAD_PER_REV /
            calibration->counts_per_revolution[i];
        velocity_urad_s[i] = counts_per_ms_to_urad_s(
            normalized_speed_counts_ms[i],
            calibration->counts_per_revolution[i]);
        *valid_mask |= (uint8_t)(1U << i);
    }
    return 1U;
}

uint8_t wheel_control_command_to_counts(
    const int32_t wheel_velocity_urad_s[WHEEL_COUNT],
    const wheel_calibration_t *calibration,
    int32_t target_counts_ms[WHEEL_COUNT])
{
    if (wheel_velocity_urad_s == NULL || target_counts_ms == NULL ||
        !wheel_control_calibration_valid(calibration))
        return 0U;

    for (unsigned i = 0U; i < WHEEL_COUNT; ++i) {
        int64_t counts =
            (int64_t)wheel_velocity_urad_s[i] *
            calibration->counts_per_revolution[i] /
            WHEEL_URAD_S_PER_REV_MS;
        target_counts_ms[i] =
            (int32_t)clamp_i64(counts, INT32_MIN, INT32_MAX);
    }
    return 1U;
}

uint8_t wheel_control_stabilize(
    int32_t wheel_target_urad_s[WHEEL_COUNT],
    const int32_t measured_wheel_urad_s[WHEEL_COUNT],
    int32_t imu_yaw_rate_mrad_s,
    const wheel_calibration_t *calibration,
    uint32_t max_correction_urad_s, uint16_t gain_permille,
    int32_t correction_urad_s[WHEEL_COUNT])
{
    static const int8_t rotation_pattern[WHEEL_COUNT] = {-1, 1, -1, 1};
    int64_t yaw_from_encoders_mrad_s;
    int64_t yaw_error_mrad_s;
    int64_t correction;

    if (correction_urad_s == NULL)
        return 0U;
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
        correction_urad_s[i] = 0;

    if (wheel_target_urad_s == NULL || measured_wheel_urad_s == NULL ||
        !wheel_control_calibration_valid(calibration) ||
        gain_permille > 1000U)
        return 0U;

    int64_t rotation_sum =
        -(int64_t)measured_wheel_urad_s[0] +
          (int64_t)measured_wheel_urad_s[1] -
          (int64_t)measured_wheel_urad_s[2] +
          (int64_t)measured_wheel_urad_s[3];
    yaw_from_encoders_mrad_s =
        rotation_sum * calibration->wheel_radius_um /
        calibration->rotational_lever_um / 4000LL;
    yaw_from_encoders_mrad_s =
        clamp_i64(yaw_from_encoders_mrad_s, -2000000LL, 2000000LL);
    yaw_error_mrad_s = (int64_t)imu_yaw_rate_mrad_s -
                       yaw_from_encoders_mrad_s;
    correction = -yaw_error_mrad_s *
                 calibration->rotational_lever_um * gain_permille /
                 calibration->wheel_radius_um;
    if (max_correction_urad_s > (uint32_t)INT32_MAX)
        max_correction_urad_s = (uint32_t)INT32_MAX;
    correction = clamp_i64(correction, -(int64_t)max_correction_urad_s,
                           (int64_t)max_correction_urad_s);

    for (unsigned i = 0U; i < WHEEL_COUNT; ++i) {
        int32_t before = wheel_target_urad_s[i];
        int64_t after = (int64_t)before +
                        rotation_pattern[i] * correction;
        wheel_target_urad_s[i] =
            (int32_t)clamp_i64(after, INT32_MIN, INT32_MAX);
        correction_urad_s[i] = wheel_target_urad_s[i] - before;
    }
    return 1U;
}

uint8_t wheel_control_common_saturate(
    int32_t wheel_target[WHEEL_COUNT], int32_t max_abs_target)
{
    int64_t peak = 0;
    if (wheel_target == NULL || max_abs_target <= 0)
        return 0U;

    for (unsigned i = 0U; i < WHEEL_COUNT; ++i) {
        int64_t magnitude = wheel_target[i] < 0
            ? -(int64_t)wheel_target[i] : (int64_t)wheel_target[i];
        if (magnitude > peak)
            peak = magnitude;
    }
    if (peak <= max_abs_target)
        return 1U;

    for (unsigned i = 0U; i < WHEEL_COUNT; ++i) {
        wheel_target[i] = (int32_t)(
            (int64_t)wheel_target[i] * max_abs_target / peak);
    }
    return 1U;
}
