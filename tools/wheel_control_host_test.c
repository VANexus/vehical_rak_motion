#include "app_wheel_control.h"

#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>

extern uint8_t wheel_control_sample_fresh(uint32_t now_ms,
                                           uint32_t sample_ms,
                                           uint8_t sample_valid);

static uint32_t test_tick_value;
static uint32_t test_tick_reads;
static wheel_target_mailbox_t concurrent_mailbox;
static atomic_uint_least32_t encoder_test_generation;
static atomic_uint_least32_t encoder_test_position[WHEEL_COUNT];
static atomic_uint_least32_t encoder_test_epoch;

static uint32_t advancing_tick(void)
{
    test_tick_reads++;
    return ++test_tick_value;
}

static wheel_calibration_t signed_calibration(void)
{
    wheel_calibration_t calibration = {
        .counts_per_revolution = {60000U, 61000U, 62000U, 63000U},
        .wheel_radius_um = 50000U,
        .rotational_lever_um = 150000U,
        .encoder_direction = {1, -1, -1, 1},
        .valid = 1U
    };
    return calibration;
}

static wheel_target_snapshot_t make_snapshot(uint32_t sequence)
{
    wheel_target_snapshot_t snapshot = {
        .target_counts_ms = {
            (int32_t)sequence, -(int32_t)sequence,
            (int32_t)(sequence * 3U), -(int32_t)(sequence * 3U)
        },
        .command_tick = sequence,
        .source = (uint8_t)(sequence & 1U),
        .enabled = (uint8_t)((sequence >> 1U) & 1U)
    };
    return snapshot;
}

static void *mailbox_writer(void *context)
{
    uint32_t count = *(const uint32_t *)context;
    for (uint32_t sequence = 1U; sequence <= count; ++sequence) {
        wheel_target_snapshot_t snapshot = make_snapshot(sequence);
        wheel_target_mailbox_publish(&concurrent_mailbox, &snapshot);
        if ((sequence & 7U) == 0U)
            sched_yield();
    }
    return NULL;
}

static void *encoder_reset_writer(void *context)
{
    uint32_t count = *(const uint32_t *)context;
    for (uint32_t epoch = 1U; epoch <= count; ++epoch) {
        wheel_control_encoder_generation_write_begin(
            &encoder_test_generation);
        for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
            atomic_store_explicit(&encoder_test_position[i], epoch,
                                  memory_order_relaxed);
        atomic_store_explicit(&encoder_test_epoch, epoch,
                              memory_order_relaxed);
        wheel_control_encoder_generation_write_end(
            &encoder_test_generation);
    }
    return NULL;
}

/* [AI] R46: 用例语义随 R30 标定表预置而更新——默认 car_wheel_calibration
   已是合法预置表（61600 CPR 等，历史台架值待实车复核），不再断言"无效"。 */
static void test_default_calibration_is_preloaded_and_valid(void)
{
    int32_t raw_position[WHEEL_COUNT] = {100, 100, 100, 100};
    int32_t raw_speed[WHEEL_COUNT] = {10, 10, 10, 10};
    int64_t position[WHEEL_COUNT];
    int32_t velocity[WHEEL_COUNT];
    uint8_t valid_mask = 0x0FU;

    assert(wheel_control_calibration_valid(&car_wheel_calibration));
    assert(wheel_control_encoder_state(
        raw_position, raw_speed, 0x0FU, &car_wheel_calibration,
        position, velocity, &valid_mask));
    assert(valid_mask == 0x0FU);
    /* [AI] R46: 位置符号跟随 encoder_direction 预置 {1,-1,-1,1}；
       速度符号只由 normalized_speed 符号决定（不经 direction 翻转） */
    assert(position[0] > 0 && position[1] < 0);
    assert(position[2] < 0 && position[3] > 0);
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
        assert(velocity[i] > 0);
}

static void test_encoder_position_velocity_and_direction_units(void)
{
    wheel_calibration_t calibration = signed_calibration();
    int32_t raw_position[WHEEL_COUNT] = {60000, 61000, -31000, -63000};
    int32_t normalized_speed[WHEEL_COUNT] = {1, 1, -1, -1};
    int64_t position[WHEEL_COUNT];
    int32_t velocity[WHEEL_COUNT];
    uint8_t valid_mask = 0U;

    assert(wheel_control_encoder_state(
        raw_position, normalized_speed, 0x0FU, &calibration,
        position, velocity, &valid_mask));
    assert(valid_mask == 0x0FU);
    assert(position[0] == 6283185LL);
    assert(position[1] == -6283185LL);
    assert(position[2] == 3141592LL);
    assert(position[3] == -6283185LL);
    assert(velocity[0] == 104719);
    assert(velocity[1] == 103003);
    assert(velocity[2] == -101341);
    assert(velocity[3] == -99733);

    assert(wheel_control_encoder_state(
        raw_position, normalized_speed, 0x05U, &calibration,
        position, velocity, &valid_mask));
    assert(valid_mask == 0x05U);
    assert(position[1] == 0 && velocity[1] == 0);
    assert(position[3] == 0 && velocity[3] == 0);
}

static void test_command_conversion_uses_each_wheel_cpr(void)
{
    wheel_calibration_t calibration = signed_calibration();
    int32_t command[WHEEL_COUNT] = {
        6283185, -6283185, 3141592, -3141592
    };
    int32_t target[WHEEL_COUNT] = {0};

    assert(wheel_control_command_to_counts(command, &calibration, target));
    assert(target[0] == 60);
    assert(target[1] == -61);
    assert(target[2] == 30);
    assert(target[3] == -31);
}

static void test_normalized_bench_target_does_not_apply_encoder_sign(void)
{
    wheel_calibration_t calibration = signed_calibration();
    int32_t target_counts_ms[WHEEL_COUNT] = {60, 61, 62, 63};
    int32_t target_urad_s[WHEEL_COUNT] = {0};
    int32_t converted_counts_ms[WHEEL_COUNT] = {0};

    assert(wheel_control_normalized_counts_to_urad_s(
        target_counts_ms, &calibration, target_urad_s));
    assert(wheel_control_command_to_counts(
        target_urad_s, &calibration, converted_counts_ms));
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
        assert(converted_counts_ms[i] == target_counts_ms[i]);
}

static void test_target_mailbox_publishes_one_complete_frame(void)
{
    wheel_target_mailbox_t mailbox;
    wheel_target_snapshot_t written = {
        .target_counts_ms = {101, -202, 303, -404},
        .command_tick = 0x12345678U,
        .source = 1U,
        .enabled = 1U
    };
    wheel_target_snapshot_t read = {0};

    wheel_target_mailbox_init(&mailbox);
    wheel_target_mailbox_publish(&mailbox, &written);
    assert(wheel_target_mailbox_read(&mailbox, &read));
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
        assert(read.target_counts_ms[i] == written.target_counts_ms[i]);
    assert(read.command_tick == written.command_tick);
    assert(read.source == written.source);
    assert(read.enabled == written.enabled);

    written.target_counts_ms[0] = -505;
    written.target_counts_ms[1] = 606;
    written.target_counts_ms[2] = -707;
    written.target_counts_ms[3] = 808;
    written.command_tick++;
    written.source = 0U;
    written.enabled = 0U;
    wheel_target_mailbox_publish(&mailbox, &written);
    assert(wheel_target_mailbox_read(&mailbox, &read));
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
        assert(read.target_counts_ms[i] == written.target_counts_ms[i]);
    assert(read.command_tick == written.command_tick);
    assert(read.source == written.source);
    assert(read.enabled == written.enabled);
}

static void test_target_mailbox_never_returns_a_mixed_frame(void)
{
    const uint32_t publish_count = 200000U;
    pthread_t writer;
    wheel_target_snapshot_t snapshot;
    uint32_t successful_reads = 0U;

    wheel_target_mailbox_init(&concurrent_mailbox);
    assert(pthread_create(&writer, NULL, mailbox_writer,
                          (void *)&publish_count) == 0);
    while (successful_reads < publish_count) {
        if (!wheel_target_mailbox_read(&concurrent_mailbox, &snapshot)) {
            sched_yield();
            continue;
        }
        if (snapshot.command_tick == 0U)
            continue;
        assert(snapshot.target_counts_ms[0] == (int32_t)snapshot.command_tick);
        assert(snapshot.target_counts_ms[1] == -(int32_t)snapshot.command_tick);
        assert(snapshot.target_counts_ms[2] ==
               (int32_t)(snapshot.command_tick * 3U));
        assert(snapshot.target_counts_ms[3] ==
               -(int32_t)(snapshot.command_tick * 3U));
        assert(snapshot.source == (uint8_t)(snapshot.command_tick & 1U));
        assert(snapshot.enabled ==
               (uint8_t)((snapshot.command_tick >> 1U) & 1U));
        successful_reads++;
    }
    assert(pthread_join(writer, NULL) == 0);
}

static void test_encoder_generation_rejects_reset_overlap(void)
{
    assert(wheel_control_encoder_generation_stable(12U, 12U));
    assert(!wheel_control_encoder_generation_stable(13U, 13U));
    assert(!wheel_control_encoder_generation_stable(12U, 14U));
}

static void test_reset_snapshot_keeps_position_and_epoch_in_one_generation(void)
{
    const uint32_t reset_count = 100000U;
    pthread_t writer;
    uint32_t accepted = 0U;

    atomic_init(&encoder_test_generation, 0U);
    atomic_init(&encoder_test_epoch, 0U);
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
        atomic_init(&encoder_test_position[i], 0U);
    assert(pthread_create(&writer, NULL, encoder_reset_writer,
                          (void *)&reset_count) == 0);

    while (accepted < reset_count) {
        uint32_t before = atomic_load_explicit(&encoder_test_generation,
                                              memory_order_acquire);
        uint32_t positions[WHEEL_COUNT];
        uint32_t epoch = 0U;
        if ((before & 1U) != 0U)
            continue;
        for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
            positions[i] = atomic_load_explicit(&encoder_test_position[i],
                                                memory_order_relaxed);
        epoch = atomic_load_explicit(&encoder_test_epoch,
                                     memory_order_relaxed);
        uint32_t after = atomic_load_explicit(&encoder_test_generation,
                                             memory_order_acquire);
        if (!wheel_control_encoder_generation_stable(before, after))
            continue;
        for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
            assert(positions[i] == epoch);
        accepted++;
    }
    assert(pthread_join(writer, NULL) == 0);
}

static void test_normalized_encoder_speed_saturates_without_resigning(void)
{
    wheel_calibration_t calibration = signed_calibration();
    int32_t raw_position[WHEEL_COUNT] = {0};
    int32_t normalized_speed[WHEEL_COUNT] = {0, INT32_MIN, 0, 0};
    int64_t position[WHEEL_COUNT];
    int32_t velocity[WHEEL_COUNT];
    uint8_t valid_mask = 0U;

    assert(wheel_control_encoder_state(
        raw_position, normalized_speed, 0x02U, &calibration,
        position, velocity, &valid_mask));
    assert(valid_mask == 0x02U);
    assert(velocity[1] == INT32_MIN);
}

static void test_stabilizer_correction_is_wheel_domain_and_bounded(void)
{
    wheel_calibration_t calibration = signed_calibration();
    int32_t target[WHEEL_COUNT] = {1000000, 1000000, 1000000, 1000000};
    int32_t measured[WHEEL_COUNT] = {0, 0, 0, 0};
    int32_t correction[WHEEL_COUNT] = {0};
    int32_t yaw_err = 0;

    /* [AI] R46: kd=0, deriv=0 → 纯 P 行为（与 R45 之前一致） */
    assert(wheel_control_stabilize(target, measured, 1000, &calibration,
                                   50000U, 500U, correction,
                                   &yaw_err, 0, 0U));
    assert(yaw_err == 1000);
    assert(correction[0] == 50000);
    assert(correction[1] == -50000);
    assert(correction[2] == 50000);
    assert(correction[3] == -50000);
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
        assert(target[i] == 1000000 + correction[i]);

    /* [AI] R46: D 项——误差 1000 mrad/s、微分 2000 mrad/s²、kd=150：
       P=1000*150000/32500*500‰≈2.31e6, D=2000*150000/32500*150‰≈1.38e6,
       合成 3.69e6 > 限幅 50000 → 仍满限幅，但符号随误差/微分同向。 */
    int32_t target2[WHEEL_COUNT] = {1000000, 1000000, 1000000, 1000000};
    int32_t correction2[WHEEL_COUNT] = {0};
    assert(wheel_control_stabilize(target2, measured, 1000, &calibration,
                                   50000U, 500U, correction2,
                                   NULL, 2000, 150U));
    assert(correction2[0] == 50000);
    assert(correction2[1] == -50000);

    /* [AI] R46: kd>0、微分反向 → 修正量低于纯 P（D 抵消瞬态）。
       小误差场景：err=100 mrad/s → P≈1538; deriv=-2000, kd=150 → D≈-1.38e6,
       合成负 → 钳位到 -50000（限幅内则应为负值）。 */
    int32_t target3[WHEEL_COUNT] = {1000000, 1000000, 1000000, 1000000};
    int32_t correction3[WHEEL_COUNT] = {0};
    assert(wheel_control_stabilize(target3, measured, 100, &calibration,
                                   50000U, 500U, correction3,
                                   NULL, -2000, 150U));
    assert(correction3[0] < 0);
    assert(correction3[1] > 0);

    /* [AI] R46: 非法标定 → 拒绝且清零（R30 起默认表合法，需本地构造非法表） */
    wheel_calibration_t invalid_cal = signed_calibration();
    invalid_cal.valid = 0U;
    memset(correction, 0x7F, sizeof(correction));
    assert(!wheel_control_stabilize(target, measured, 1000,
                                    &invalid_cal,
                                    50000U, 500U, correction,
                                    NULL, 0, 0U));
    for (unsigned i = 0U; i < WHEEL_COUNT; ++i)
        assert(correction[i] == 0);
}

static void test_common_saturation_preserves_wheel_ratios(void)
{
    int32_t target[WHEEL_COUNT] = {1500, -3000, 750, -1500};

    assert(wheel_control_common_saturate(target, 1000));
    assert(target[0] == 500);
    assert(target[1] == -1000);
    assert(target[2] == 250);
    assert(target[3] == -500);

    int32_t unchanged[WHEEL_COUNT] = {INT32_MIN, 0, 0, 0};
    assert(!wheel_control_common_saturate(unchanged, 0));
    assert(unchanged[0] == INT32_MIN);
}

static void test_sample_freshness_boundary_and_tick_wrap(void)
{
    assert(wheel_control_sample_fresh(120U, 100U, 1U));
    assert(!wheel_control_sample_fresh(121U, 100U, 1U));
    assert(wheel_control_sample_fresh(3U, UINT32_MAX - 16U, 1U));
    assert(!wheel_control_sample_fresh(100U, 101U, 1U));
    assert(!wheel_control_sample_fresh(3U, UINT32_MAX - 16U, 0U));

    test_tick_value = 100U;
    test_tick_reads = 0U;
    assert(wheel_control_sample_fresh_from_clock(
        advancing_tick, 101U, 1U));
    assert(test_tick_reads == 1U);
}

static void test_sample_age_distinguishes_future_from_stale_across_wrap(void)
{
    assert(wheel_control_sample_age(120U, 100U, 1U) == WHEEL_SAMPLE_FRESH);
    assert(wheel_control_sample_age(121U, 100U, 1U) == WHEEL_SAMPLE_STALE);
    assert(wheel_control_sample_age(100U, 101U, 1U) == WHEEL_SAMPLE_FUTURE);
    assert(wheel_control_sample_age(3U, UINT32_MAX - 16U, 1U) ==
           WHEEL_SAMPLE_FRESH);
    assert(wheel_control_sample_age(100U, 101U, 0U) ==
           WHEEL_SAMPLE_INVALID);
}

static void test_uptime_timestamp_extends_tick_wrap_monotonically(void)
{
    wheel_uptime_clock_t clock = {0};
    uint64_t before = wheel_control_uptime_us(&clock, UINT32_MAX - 2U);
    uint64_t after = wheel_control_uptime_us(&clock, 3U);
    uint64_t late_previous_tick =
        wheel_control_uptime_us(&clock, UINT32_MAX - 1U);
    uint64_t next = wheel_control_uptime_us(&clock, 4U);

    assert(before == (uint64_t)(UINT32_MAX - 2U) * 1000ULL);
    assert(after == ((uint64_t)UINT32_MAX + 4ULL) * 1000ULL);
    assert(after > before);
    assert(late_previous_tick == after);
    assert(next == ((uint64_t)UINT32_MAX + 5ULL) * 1000ULL);
}

int main(void)
{
    test_default_calibration_is_preloaded_and_valid();
    test_encoder_position_velocity_and_direction_units();
    test_command_conversion_uses_each_wheel_cpr();
    test_normalized_bench_target_does_not_apply_encoder_sign();
    test_target_mailbox_publishes_one_complete_frame();
    test_target_mailbox_never_returns_a_mixed_frame();
    test_encoder_generation_rejects_reset_overlap();
    test_reset_snapshot_keeps_position_and_epoch_in_one_generation();
    test_normalized_encoder_speed_saturates_without_resigning();
    test_stabilizer_correction_is_wheel_domain_and_bounded();
    test_common_saturation_preserves_wheel_ratios();
    test_sample_freshness_boundary_and_tick_wrap();
    test_sample_age_distinguishes_future_from_stale_across_wrap();
    test_uptime_timestamp_extends_tick_wrap_monotonically();
    puts("轮端控制 host 测试通过");
    return 0;
}
