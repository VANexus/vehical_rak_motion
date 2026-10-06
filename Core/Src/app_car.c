#include "app_car.h"
#include "app_imu.h"
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "iwdg.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* [AI]  @file  app_car.c — 底盘应用层：电机/编码器驱动、25kHz 闭环 ISR、
   100Hz 监督器（台架测试/麦轮自检/遥控安全链/逆运动学）、电池/看门狗。
   控制逻辑移植自台架验证版 four-motor-25k(C2)，PWM/中断周期按实际定时器
   时钟推导。板级真源见 docs/BOARD-C30D.md，协议见 docs/PROTOCOL.md。 */

/* Quadrature counts per output-shaft revolution: MG513P30, 512 lines
   *4 quadrature *30:1 = 61440; 61600 is the hand-measured value. */
#define ENCODER_COUNTS_PER_REV 61600L
#define PWM_HZ 25000U
#define MAX_DUTY_PERCENT 85U
#define MOTOR_TIMEOUT_MS 90000U
#define STALL_TIMEOUT_MS 2500U
#define POSITION_TOLERANCE (ENCODER_COUNTS_PER_REV / 200L)
#define SHORT_ACCEPT       (ENCODER_COUNTS_PER_REV / 100L)  /* 3.00 +-0.01 band */
#define OVERSHOOT_LIMIT    (ENCODER_COUNTS_PER_REV / 8L)   /* gross overshoot -> FAULT */

/* 25 kHz closed loop (design: PID interrupt, fixed point, no float).
   The timer ISR samples the encoders every 40 us, accumulates a 1 ms
   window for the M-method speed (one 40 us window is only ~6 counts
   +-1, too coarse) and runs the PI at 1 kHz while the PWM is refreshed
   every 40 us.  All four motors chase the same target speed, so weak
   motors automatically run more duty - nobody outruns anybody. */
#define BENCH_CRUISE   150L   /* counts/ms (150 counts / 10 ms) */
#define BENCH_APPROACH  80L   /* counts/ms inside the last 1/4 turn */
#define KICK_DUTY       75U   /* breakaway kick: one strong duty for all
                                 four so static friction cannot stagger
                                 the start (raise to 80 if one lags) */
#define SPEED_RAMP_STEP  8L   /* counts/ms of slew per ms */
#define SPEED_KP_DIV     2L   /* duty_q += err / 2           (P) */
#define SPEED_KI_DIV     4L   /* duty_q += err / 4 per 1 ms  (I) */
#define V_CMD_LIMIT    350L   /* counts/ms safety clamp */
#define RC_TIMEOUT_MS  100U   /* Jetson deadman (design: 100 ms) */

/* [AI]  指令入口反溢出钳位 —— 不是物理限幅（真正的限幅在 ik_wheel_target 的
   V_CMD_LIMIT）。主机速度是裸 int32，直接进 stabilizer_apply 的 *vx*stab_scale
   和 IK 的 int32 求和会溢出（有符号溢出 = UB）。这里只为把算术拉回良定义域，
   取值远超任何真实指令，合法指令的行为不受影响。 */
#define V_CMD_MAX_MMPS  20000L    /* ±20 m/s */
#define W_CMD_MAX_CRADS 100000L   /* ±1000 rad/s */

/* Mecanum kinematics: wheel rim speed for body velocity
      v_i = sx*Vx + sy*Vy + sw*(lx+ly)*wz   [mm/s, wz in 0.01 rad/s]
   converted with counts/ms = mm/s * 61600 / (2*pi*32.5 mm) / 1000.
   If a wheel runs the wrong way during test, flip its sign(s) here. */
#define IK_PERMM_NUM  3017L   /* counts per mm, x10 */
#define IK_PERMM_DEN 10000L

/* [AI]  R9 陀螺仪自稳定调参（A 板 ICM20948；算法见 stabilizer_apply） */
#define STAB_ENABLE       1
#define STAB_RATE_KP      1.5f   /* 旋转速率误差(°/s) → 修正(0.01rad/s) */
#define STAB_DAMP_K       1.2f   /* 无旋转指令时的阻尼增益 */
#define STAB_HEAD_KP      0.6f   /* 航向保持(每度偏差 → 0.01rad/s) */
#define STAB_CORR_MAX     120L   /* 修正限幅(0.01rad/s) */
#define STAB_HOLD_BAND    3L     /* |wz指令| 小于此值视作"无旋转指令" */
#define SLIP_ERR_DPS      35.0f  /* 轮速推算 vs 陀螺仪 偏航率差阈值 */
#define SLIP_HOLD_MS      300U
#define SLIP_SCALE_MIN     160U  /* 平移降额下限(256=不降) */
#define SLIP_SCALE_STEP    2U
#define SLIP_RECOVER_STEP  4U

enum { MOTOR_COUNT = 4 };
enum run_mode { IDLE, RUNNING, DONE, STOPPED, FAULT, CALIBRATING, CALIB_DONE };
enum motor_phase { MOTOR_DRIVE, MOTOR_BRAKE, MOTOR_DONE };

struct motor_state {
    volatile enum motor_phase phase;   /* [AI]  ISR 读、任务写 —— 必须 volatile */
    uint32_t started_ms;
    uint32_t last_motion_ms;
    uint32_t brake_started_ms;
    uint8_t corrections;
    int32_t stint_pos;   /* sign-normalized position at correction start */
};

static const uint8_t target_turns[MOTOR_COUNT] = {3, 3, 3, 3};  /* equal targets */
/* Brake-trigger distance per motor (counts), used once the motor has
   settled into the 30% approach inside the last 1/4 turn.  Re-centred
   from the H5 landings (A+0.10 / B+0.08 / C+0.07 / D+0.05 turns): the
   stop distance at approach speed is ~6.1-6.7k counts per motor, so
   each trigger sits (coast - 0.005 turn) before the target. */
static const int32_t brake_lead[MOTOR_COUNT] = {6350, 6420, 5800, 6370};
static const int32_t ik_lw = 150L;   /* lx+ly in mm (half wheelbase + half track) */
static const int8_t ik_sx[MOTOR_COUNT] = { 1,  1,  1,  1};
static const int8_t ik_sy[MOTOR_COUNT] = {-1,  1,  1, -1};
static const int8_t ik_sw[MOTOR_COUNT] = {-1,  1, -1,  1};
static const int8_t forward_dir[MOTOR_COUNT] = {1, -1, -1, 1};
static TIM_TypeDef *const enc_tim[MOTOR_COUNT] = {TIM2, TIM3, TIM4, TIM5};
static volatile int32_t position_counts[MOTOR_COUNT];  /* ISR-owned */
static uint32_t previous_count[MOTOR_COUNT];           /* ISR-owned */
static volatile int8_t encoder_sign[MOTOR_COUNT];
static volatile int32_t duty_q[MOTOR_COUNT];           /* 0.01 %, ISR PI state */
static struct motor_state motors[MOTOR_COUNT];
/* 25 kHz control ISR state */
static volatile int32_t win_sum[MOTOR_COUNT];          /* counts, 1 ms window */
static volatile uint32_t win_cnt;
static volatile int32_t wheel_speed[MOTOR_COUNT];      /* counts/ms, signed */
static volatile int32_t v_cmd[MOTOR_COUNT];            /* counts/ms, supervisor */
static int32_t v_ramp[MOTOR_COUNT];                    /* ISR slew-limited target */
/* CAN command state (supervisor consumes, Task_CAN_Rx writes) */
static volatile int32_t cmd_vx_mmps, cmd_vy_mmps, cmd_wz_crads;
static volatile uint8_t cmd_enable;
static volatile uint8_t estop_latched;
static volatile uint8_t car_estop_fast_req;   /* [AI]  CAN 中断置位, 25kHz ISR 执行急停 */
static volatile uint32_t last_jetson_ms;
static volatile int32_t meas_vx_mmps, meas_vy_mmps, meas_wz_crads;
static uint8_t remote_running;
static volatile enum run_mode mode = IDLE;
static volatile unsigned active_motor;
volatile uint32_t motor_command;   /* SWD/CAN mailbox: 1=run 2=stop 3=cal */

static uint32_t pwm_arr;           /* [AI] R20: 取自 htim1.Init.Period(6719 = 168e6/25e3-1) */
static volatile uint32_t motor_vbat_mv;
static volatile int8_t enable_switch;
static uint8_t reset_cause;
const char build_tag[] = "R20";   /* [AI] R20: CubeMX 全量迁移版——外设初始化交 HAL/MX_*_Init,应用层只留逻辑;25kHz ISR 函数体保持寄存器级,经 TIM6_DAC_IRQHandler USER CODE 挂载 */
/* [AI]  R9 自稳定状态/诊断（非 static: SWD 直读） */
volatile int32_t  dbg_stab_corr;
volatile uint16_t dbg_stab_scale = 256U;
volatile uint8_t  dbg_slip;
/* [AI]  R19: 25kHz ISR 占用率自测（AGENTS.md 要求实测；DWT 周期计数）。
   周期 40µs @168MHz = 6720 周期，预算 35% = 2352 周期。SWD 直读这两个量：
   max 是上电以来的高水位，last 是最近一拍。写 max=0 可清零重测。 */
volatile uint32_t dbg_isr_last_cycles;
volatile uint32_t dbg_isr_max_cycles;
static uint16_t stab_scale = 256U;
static uint16_t stab_slip_ms;

/* ---- PWM + motor drivers (AT8236, proven mapping) ----
   [AI] R20 CubeMX 迁移：TIM1/9/10/11 由 MX_TIMx_Init + MspInit 配置
   （PSC=0、TIM1 ARR=6719→25kHz@168MHz、各通道 Pulse=0——上电不驱动），
   本层只启动 PWM 输出并取 ARR 作占空比换算基准。 */
static void motor_pwm_start(void)
{
    pwm_arr = htim1.Init.Period;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    /* C/D: PE9/PE11/PE13/PE14 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);    /* B: PE5/PE6 */
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);   /* A: PB8/PB9 */
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}

static void motor_pair(unsigned i, uint32_t forward, uint32_t reverse)
{
    switch (i) {
    case 0: __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, forward);
            __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, reverse); break;
    case 1: __HAL_TIM_SET_COMPARE(&htim9,  TIM_CHANNEL_1, forward);
            __HAL_TIM_SET_COMPARE(&htim9,  TIM_CHANNEL_2, reverse); break;
    case 2: __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, forward);
            __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_2, reverse); break;
    case 3: __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_3, forward);
            __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_4, reverse); break;
    default: break;
    }
}

static void motor_drive(unsigned i, int direction, unsigned percent)
{
    if (percent > MAX_DUTY_PERCENT) percent = MAX_DUTY_PERCENT;
    uint32_t pulse = (pwm_arr + 1U) * percent / 100U;
    if (direction >= 0) motor_pair(i, pulse, 0);
    else motor_pair(i, 0, pulse);
}

static void motor_brake(unsigned i)
{
    motor_pair(i, pwm_arr + 1U, pwm_arr + 1U);
}

static void motor_stop_all(void)
{
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) motor_pair(i, 0, 0);
}

/* (brake-all is expressed by setting every phase to MOTOR_BRAKE; the
   ISR writes the actual outputs) */

/* ---- encoders (TIM2/3/4/5 quadrature) ----
   [AI] R20 CubeMX 迁移：编码器模式(TI12 双边沿 4 倍频)/引脚/ARR 由
   MX_TIM2/3/4/5_Init 配置（TIM3/4 16bit、TIM2/5 32bit——与 ISR 的
   int16/int32 差值逻辑对应），本层只启动计数。 */
static void encoder_start(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);   /* A: PA15/PB3 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);   /* B: PB4/PB5 */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);   /* C: PB6/PB7 */
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);   /* D: PA0/PA1 */
}

/* ---- button / enable switch / battery ADC / LED ----
   [AI] R20 CubeMX 迁移：PE0(按键,上拉低有效)/PD3(使能开关,下拉) 已由
   MX_GPIO_Init 配置，读取走 HAL_GPIO_ReadPin。 */

/* 0 = none, 1 = short press (release edge, held < LONG_PRESS_MS),
   2 = long press (fired once while held >= LONG_PRESS_MS) */
#define BTN_DEBOUNCE_MS  30U
#define LONG_PRESS_MS    1200U
static int button_pressed_event(uint32_t now)
{
    static uint8_t stable;        /* 已消抖的生效电平 */
    static uint8_t raw_prev;
    static uint32_t changed_at;
    static uint32_t pressed_at;
    static uint8_t long_fired;
    uint8_t raw = (uint8_t)(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET);
    int ev = 0;

    if (raw != raw_prev) {
        raw_prev = raw;
        changed_at = now;
    }
    if (raw != stable && (uint32_t)(now - changed_at) >= BTN_DEBOUNCE_MS) {
        stable = raw;
        if (stable != 0U) {
            pressed_at = now;
            long_fired = 0U;
        } else if (long_fired == 0U) {
            ev = 1;               /* [AI]  短按在释放沿确认,避免吞掉长按 */
        }
    }
    if (stable != 0U && long_fired == 0U &&
        (uint32_t)(now - pressed_at) >= LONG_PRESS_MS) {
        long_fired = 1U;
        ev = 2;
    }
    return ev;
}

/* [AI] R20 CubeMX 迁移：ADC1(PB0/CH8, PCLK2/4 + 480 周期采样 + 软件触发)
   由 MX_ADC1_Init 配置（原寄存器版的分频/采样时间修正已进 .ioc）；
   F4 的 HAL 无 ADC 校准 API（无需移植原 CAL 位操作）。 */
static uint32_t battery_read_mv(void)
{
    uint32_t mv = 0U;
    if (HAL_ADC_Start(&hadc1) == HAL_OK &&
        HAL_ADC_PollForConversion(&hadc1, 1U) == HAL_OK) {
        mv = HAL_ADC_GetValue(&hadc1) * 3300UL * 11UL / 4095UL;
    }
    (void)HAL_ADC_Stop(&hadc1);
    return mv;
}

/* [AI] R20: IWDG 已由 MX_IWDG_Init 启动（HAL v1.28.3 的 HAL_IWDG_Init 内含
   __HAL_IWDG_START，2s 超时）——注意 IMU 探测期间的 boot 喂狗在 app_imu.c */

/* ---- control (C2 motor base + four-wheel speed balancing) ---- */

/* [AI]  25kHz ISR 独占状态（previous_count / position_counts / win_sum /
   duty_q / v_ramp / wheel_speed / encoder_sign 及 motors[].phase）的原子改写。
   ⚠️ 必须用这一对，不能用 taskENTER_CRITICAL()：TIM6 ISR 优先级 2 高于
   FreeRTOS 的 configMAX_SYSCALL_INTERRUPT_PRIORITY(=5)，而 taskENTER_CRITICAL()
   只是把 BASEPRI 抬到 5 —— 优先级数值 <5 的中断根本不受它屏蔽，
   临界区是"假"的。这里直接关 TIM6 的 NVIC 使能，才是真原子。 */
static inline void isr_state_lock(void)
{
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    __DSB();
    __ISB();
}

static inline void isr_state_unlock(void)
{
    __DSB();
    __ISB();
    NVIC_EnableIRQ(TIM6_DAC_IRQn);   /* 期间若挂起，此处放行后立即补跑一拍 */
}

static void reset_positions(void)
{
    isr_state_lock();
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        enc_tim[i]->CNT = 0;
        previous_count[i] = 0;
        position_counts[i] = 0;
        encoder_sign[i] = 0;
        duty_q[i] = 0;
        v_ramp[i] = 0;
        v_cmd[i] = 0;
        wheel_speed[i] = 0;
        win_sum[i] = 0;
        motors[i].phase = MOTOR_BRAKE;
    }
    isr_state_unlock();
}

/* [AI]  立即停：四路刹车 + 目标清零（台架/遥控/急停多处复用） */
static void brake_all(void)
{
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].phase = MOTOR_BRAKE;
        v_cmd[i] = 0;
    }
}

/* [AI]  平滑停：目标清零，等转速落稳再刹车；返回 1 = 已停稳 */
static int coast_stop(void)
{
    int moving = 0;
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        v_cmd[i] = 0;
        if (wheel_speed[i] < -5 || wheel_speed[i] > 5) moving = 1;
    }
    if (moving == 0) brake_all();
    return (moving == 0);
}

static void start_run(uint32_t now)
{
    reset_positions();
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].phase = MOTOR_DRIVE;
        motors[i].started_ms = now;
        motors[i].last_motion_ms = now;
        motors[i].brake_started_ms = 0;
        motors[i].corrections = 0;
        motors[i].stint_pos = 0;
        duty_q[i] = 0;
        v_cmd[i] = BENCH_CRUISE;
    }
    active_motor = 0;
    mode = RUNNING;
}

static void start_calibration(uint32_t now)
{
    reset_positions();
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) motors[i].started_ms = now;
    mode = CALIBRATING;   /* the ISR drives 55 % open loop */
}

static void fail_run(unsigned i)
{
    active_motor = i;
    brake_all();
    mode = FAULT;
}

/* encoder accounting moved into the TIM6 control ISR */

/* ---- mecanum motion self-test (long-press USER; no CAN needed) ----
   Runs the same IK path as remote control: fwd / back / left / right /
   ccw / cw, each 1.5 s with 0.8 s pauses, looping until stopped. */
enum { MT_SEQ_N = 12 };
static const int16_t mt_seq[MT_SEQ_N][3] = {
    { 150,   0,  0}, { 0, 0, 0},
    {-150,   0,  0}, { 0, 0, 0},
    {   0, 150,  0}, { 0, 0, 0},
    {   0,-150,  0}, { 0, 0, 0},
    {   0,   0, 50}, { 0, 0, 0},   /* +wz = ccw */
    {   0,   0,-50}, { 0, 0, 0},   /* -wz = cw  */
};
#define MT_RUN_MS    1500U
#define MT_PAUSE_MS   800U
static uint8_t mt_running;
static uint8_t mt_stop_req;
static uint8_t mt_index;
static uint32_t mt_step_ms;

static void start_motion_test(uint32_t now)
{
    reset_positions();
    imu_zero_yaw();   /* [AI]  R9: 以进入自检时的航向为参考 */
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) motors[i].phase = MOTOR_DRIVE;
    mt_index = 0;
    mt_step_ms = now;
    mt_stop_req = 0;
    mt_running = 1;
    mode = IDLE;
}

/* ---- 100 Hz supervisor (runs in Task_Control) ---- */
static int32_t ik_wheel_target(unsigned i, int32_t vx, int32_t vy, int32_t wz)
{
    /* wz is 0.01 rad/s; (lx+ly)*wz_rad = ik_lw * wz / 100  [mm/s] */
    int32_t lin = ik_sx[i] * vx + ik_sy[i] * vy + ik_sw[i] * ((ik_lw * wz) / 100L);
    int32_t cpm = (int32_t)((int64_t)lin * IK_PERMM_NUM / IK_PERMM_DEN);
    if (cpm > V_CMD_LIMIT) cpm = V_CMD_LIMIT;
    if (cpm < -V_CMD_LIMIT) cpm = -V_CMD_LIMIT;
    return cpm;
}

/* [AI] R14: 台架刹车段两个重复模式提取（行为保持）
   brake_settle: 车轮未停稳则重刷刹车起点并返回 1（调用方 continue 等下一拍）
   mark_done:    判定到位,置 DONE 并累计完成数 */
static int brake_settle(struct motor_state *m, uint32_t now, int32_t speed)
{
    if (speed < -10 || speed > 10) { m->brake_started_ms = now; return 1; }
    return 0;
}

static void mark_done(struct motor_state *m, unsigned i, unsigned *finished)
{
    m->phase = MOTOR_DONE;
    v_cmd[i] = 0;
    ++(*finished);
}

static void control_bench(uint32_t now)
{
    unsigned finished = 0;
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        struct motor_state *m = &motors[i];
        if (m->phase == MOTOR_DONE) { ++finished; continue; }
        if ((uint32_t)(now - m->started_ms) > MOTOR_TIMEOUT_MS) {
            fail_run(i); return;
        }
        int32_t raw = position_counts[i];
        if (m->phase == MOTOR_BRAKE) {
            if ((uint32_t)(now - m->brake_started_ms) < 300U) continue;
            int32_t position = raw * encoder_sign[i];
            int32_t error = (int32_t)(target_turns[i] * ENCODER_COUNTS_PER_REV) - position;
            int32_t speed = wheel_speed[i];
            /* Overshoot past target: never reverse into it.  A small
               excess is accepted as done; only gross overshoot faults. */
            if (error < -POSITION_TOLERANCE) {
                if (error < -OVERSHOOT_LIMIT) { fail_run(i); return; }
                if (brake_settle(m, now, speed)) continue;
                mark_done(m, i, &finished);
                continue;
            }
            /* Small shortfall: with no reverse allowed a correction from
               here could only overshoot, so accept anything inside the
               3.00 +-0.01 band instead. */
            if (error <= SHORT_ACCEPT) {
                if (brake_settle(m, now, speed)) continue;
                mark_done(m, i, &finished);
                continue;
            }
            /* Real shortfall: one more correction stint.  Moving wheels
               restart the brake wait; the stint must actually move ~300
               counts before it may brake again (drive path below). */
            if (brake_settle(m, now, speed)) continue;
            if (++m->corrections > 8U) { fail_run(i); return; }
            m->phase = MOTOR_DRIVE;
            m->stint_pos = position;
            m->last_motion_ms = now;
            /* fall through: fresh stint takes the drive path below */
        }
        if ((uint32_t)(now - m->last_motion_ms) > STALL_TIMEOUT_MS) {
            fail_run(i); return;
        }
        if (encoder_sign[i] == 0) continue;   /* ISR kicks open loop */
        if (wheel_speed[i] >= 1 || wheel_speed[i] <= -1) m->last_motion_ms = now;
        int32_t position = raw * encoder_sign[i];
        int32_t error = (int32_t)(target_turns[i] * ENCODER_COUNTS_PER_REV) - position;
        /* Fixed brake distance once the approach has settled.  A
           correction stint must actually move ~300 counts before it may
           brake, otherwise it would sit at the trigger and exhaust. */
        if (m->corrections > 0U && (m->stint_pos - position) < 300L) {
            v_cmd[i] = BENCH_APPROACH;
            continue;
        }
        if (error <= brake_lead[i]) {
            m->phase = MOTOR_BRAKE;           /* ISR applies the brake */
            m->brake_started_ms = now;
            v_cmd[i] = 0;
            continue;
        }
        /* Slower inside the last 1/4 turn; the ISR PI does the rest. */
        v_cmd[i] = (error > ENCODER_COUNTS_PER_REV / 4L) ? BENCH_CRUISE
                                                         : BENCH_APPROACH;
    }
    if (finished == MOTOR_COUNT) {
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) motors[i].phase = MOTOR_BRAKE;
        mode = DONE;
    }
}

/* [AI] ---- 陀螺仪自稳定（R9）：修正 wz + 打滑降额 ----
   麦轮打滑/被带偏时，车轮"声称"的偏航与陀螺仪实测不符：
   1) 有旋转指令：按速率误差(指令-实测)加修正；2) 无旋转指令：阻尼+航向保持；
   3) 打滑判定：轮速推算偏航率 vs 陀螺仪差 > 阈值持续 300ms → 平移分量降额，
      恢复后逐步放行。仅驱动路径生效；IMU 不在线时纯直通。 */
static float wrap180(float e)
{
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

static void stabilizer_apply(int32_t *vx, int32_t *vy, int32_t *wz)
{
    /* [AI]  R10: 旋转指令的结束沿重新取航向基准。若沿用进入模式时的基准，
       故意旋转后一松手车会被拉回原方向（自检停顿段可见回甩）；松手取
       基准后航向保持只防"被带偏"，不再对抗已完成的旋转。 */
    static uint8_t rot_cmd_prev;

    if (STAB_ENABLE == 0 || !imu_ok()) return;

    const float DPS_PER_WZ = 0.572958f;              /* 0.01 rad/s → °/s */
    float dps = imu_yaw_rate_dps();
    float corr;

    if (*wz > -STAB_HOLD_BAND && *wz < STAB_HOLD_BAND) {
        if (rot_cmd_prev != 0U) imu_zero_yaw();
        rot_cmd_prev = 0U;
        corr = -STAB_DAMP_K * dps + STAB_HEAD_KP * wrap180(-imu_yaw_deg());
    } else {
        rot_cmd_prev = 1U;
        corr = STAB_RATE_KP * ((float)(*wz) * DPS_PER_WZ - dps);
    }
    if (corr >  (float)STAB_CORR_MAX) corr =  (float)STAB_CORR_MAX;
    if (corr < -(float)STAB_CORR_MAX) corr = -(float)STAB_CORR_MAX;
    *wz += (int32_t)corr;
    dbg_stab_corr = (int32_t)corr;

    {
        float slip = (float)meas_wz_crads * DPS_PER_WZ - dps;
        if (slip < 0.0f) slip = -slip;
        int driving = (*vx != 0 || *vy != 0 || *wz != 0);
        if (driving && slip > SLIP_ERR_DPS) {
            if (stab_slip_ms < 0xFFFFU) stab_slip_ms++;
            if (stab_slip_ms > SLIP_HOLD_MS) {
                dbg_slip = 1U;
                if (stab_scale > SLIP_SCALE_MIN) stab_scale -= SLIP_SCALE_STEP;
            }
        } else {
            stab_slip_ms = 0U;
            dbg_slip = 0U;
            if (stab_scale < 256U) stab_scale += SLIP_RECOVER_STEP;
        }
        if (stab_scale < 256U) {
            *vx = (*vx * (int32_t)stab_scale) / 256;
            *vy = (*vy * (int32_t)stab_scale) / 256;
        }
        dbg_stab_scale = stab_scale;
    }
}

/* [AI] R14: 遥控与麦轮自检共用的驱动下发路径（稳定器修正 → IK → 四路目标） */
static void drive_apply(int32_t vx, int32_t vy, int32_t wz)
{
    stabilizer_apply(&vx, &vy, &wz);
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].phase = MOTOR_DRIVE;
        v_cmd[i] = ik_wheel_target(i, vx, vy, wz);
    }
}

void car_control_step(uint32_t now)
{
    motor_vbat_mv = battery_read_mv();          /* [AI]  100Hz 采样(原在显示层) */
    enable_switch = (int8_t)(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET);

    /* [AI]  R9: 陀螺仪 100Hz 更新（含静止零偏重置）；"静止"=四路无目标且实测转速≈0 */
    {
        int still = 1;
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
            if (v_cmd[i] != 0 || wheel_speed[i] < -5 || wheel_speed[i] > 5) { still = 0; break; }
        }
        imu_update(now, still);
    }

    /* operator inputs: short press = bench run start/stop, long press =
       mecanum motion self-test start/stop */
    int bev = button_pressed_event(now);
    if (bev == 1) {
        if (mt_running != 0U) {
            mt_stop_req = 1U;
        } else if (mode == RUNNING) {
            brake_all();
            mode = STOPPED;
        } else if (mode != CALIBRATING) {
            start_run(now);
        }
    } else if (bev == 2) {
        if (mt_running != 0U) {
            mt_stop_req = 1U;
        } else if (mode != RUNNING && mode != CALIBRATING) {
            start_motion_test(now);
        }
    }
    if (motor_command != 0U) {
        uint32_t command = motor_command;
        motor_command = 0;
        if (command == 2U && mt_running != 0U) {
            mt_stop_req = 1U;
        } else if (command == 1U && mt_running == 0U && mode != RUNNING) {
            start_run(now);
        } else if (command == 2U) {
            brake_all();
            mode = STOPPED;
        } else if (command == 3U && mt_running == 0U && mode != RUNNING) {
            start_calibration(now);
        }
    }

    /* encoder sign detection */
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        int32_t raw = position_counts[i];
        if (encoder_sign[i] == 0 && (raw >= 100 || raw <= -100)) {
            encoder_sign[i] = raw > 0 ? 1 : -1;
            duty_q[i] = 5000;             /* PI hand-off duty */
            v_ramp[i] = wheel_speed[i];   /* hand off at the kick speed */
        }
    }

    /* forward kinematics: measured body velocity for 0x200 telemetry */
    {
        int32_t w0 = wheel_speed[0], w1 = wheel_speed[1];
        int32_t w2 = wheel_speed[2], w3 = wheel_speed[3];
        int32_t vx_c = (w0 + w1 + w2 + w3) / 4;
        int32_t vy_c = (-w0 + w1 + w2 - w3) / 4;
        int32_t wz_c = (-w0 + w1 - w2 + w3) / 4;
        meas_vx_mmps = (int32_t)((int64_t)vx_c * 10000L / IK_PERMM_NUM);
        meas_vy_mmps = (int32_t)((int64_t)vy_c * 10000L / IK_PERMM_NUM);
        meas_wz_crads = (int32_t)((int64_t)wz_c * 1000000L /
                                  (IK_PERMM_NUM * ik_lw));
    }

    /* 5 s open-loop calibration, driven by the ISR */
    if (mode == CALIBRATING) {
        if ((uint32_t)(now - motors[0].started_ms) >= 5000U) {
            for (unsigned i = 0; i < MOTOR_COUNT; ++i) motors[i].phase = MOTOR_BRAKE;
            mode = CALIB_DONE;
        }
        return;
    }

    /* remote (Jetson) velocity mode */
    {
        uint32_t age = now - last_jetson_ms;
        int link = (last_jetson_ms != 0U) && (age <= RC_TIMEOUT_MS);
        int bench_busy = (mode == RUNNING || mode == CALIBRATING ||
                          mt_running != 0U);
        if (estop_latched != 0U) {
            if (remote_running != 0U) {
                brake_all();
                remote_running = 0;
                mode = STOPPED;
            }
            if (mt_running != 0U) {
                brake_all();
                mt_running = 0;
                mode = STOPPED;
            }
        } else if (!bench_busy && mode != FAULT) {
            if (link && cmd_enable != 0U) {
                int32_t vx = cmd_vx_mmps, vy = cmd_vy_mmps, wz = cmd_wz_crads;
                if (remote_running == 0U) imu_zero_yaw();   /* [AI]  R9: 进入遥控以当前航向为参考 */
                drive_apply(vx, vy, wz);                    /* [AI]  R9: 陀螺仪自稳定 */
                remote_running = 1;
            } else if (remote_running != 0U) {
                /* deadman / enable dropped: decelerate, then stop */
                if (coast_stop()) {
                    remote_running = 0;
                    mode = STOPPED;
                }
            }
        }
    }

    /* mecanum motion self-test engine */
    if (mt_running != 0U) {
        if (mt_stop_req != 0U) {
            if (coast_stop()) {
                mt_running = 0;
                mode = STOPPED;
            }
        } else {
            uint32_t period = (mt_index & 1U) ? MT_PAUSE_MS : MT_RUN_MS;
            if ((uint32_t)(now - mt_step_ms) >= period) {
                mt_step_ms = now;
                mt_index = (uint8_t)((mt_index + 1U) % MT_SEQ_N);
            }
            drive_apply(mt_seq[mt_index][0], mt_seq[mt_index][1], mt_seq[mt_index][2]);
        }
    }

    if (mode == RUNNING) control_bench(now);
}

/* ---- 25 kHz control timer (design: PID interrupt every 40 us) ----
   [AI] R20 CubeMX 迁移：TIM6 由 MX_TIM6_Init 配置（ARR=3359→25kHz@84MHz，
   NVIC 优先级 2 高于 configMAX_SYSCALL_INTERRUPT_PRIORITY=5——ISR 内禁用
   FreeRTOS API，原子改写用 NVIC_DisableIRQ 这对真临界区，见 isr_state_lock），
   本层只负责首拍启动。 */

/* ---- public interface ---- */
void car_init(void)
{
    /* [AI] R20 CubeMX 迁移：原时钟兜底(HSI→HSE 168MHz)已删——
       SystemClock_Config 由 CubeMX 生成且 HSE 8MHz+PLL 已确认（AGENTS.md 红线 3，
       重生成后必查 HSE 未被抹回 HSI）。 */

    {
        uint32_t csr = RCC->CSR;
        if (csr & RCC_CSR_BORRSTF) reset_cause = (uint8_t)'B';
        else if (csr & RCC_CSR_IWDGRSTF) reset_cause = (uint8_t)'I';
        else if (csr & RCC_CSR_SFTRSTF) reset_cause = (uint8_t)'S';
        else if (csr & RCC_CSR_PORRSTF) reset_cause = (uint8_t)'P';
        else reset_cause = (uint8_t)'-';
        RCC->CSR |= RCC_CSR_RMVF;
    }
    /* Debugger halts must not starve the watchdog. */
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    motor_pwm_start();
    motor_stop_all();      /* [AI] 双保险：MX 已把各 Pulse 配成 0(上电不驱动) */
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].phase = MOTOR_BRAKE;
        v_cmd[i] = 0;
        v_ramp[i] = 0;
        duty_q[i] = 0;
    }
    encoder_start();
    /* [AI] 按键(PE0)/使能开关(PD3)/ADC1(PB0)/IWDG 已由 MX_*_Init 配置；
       IWDG 在 MX_IWDG_Init 即启动(2s)，IMU 探测期间的喂狗在 app_imu.c */
    imu_init();      /* [AI]  R9: 板载 ICM20948 陀螺仪(未找到时静默降级) */
    /* [AI]  25kHz 中断不在此启动：由 Task_Control 首拍触发
       (car_start_control_isr)，boot 与实时中断解耦（见 README4 §三） */
}

/* [AI]  25kHz 控制 ISR 启动点（调度器运行后、仅一次） */
static uint8_t control_isr_started;   /* [AI]  R16: BOOTING 状态判定用 */

void car_start_control_isr(void)
{
    if (control_isr_started != 0U) return;
    control_isr_started = 1U;
    /* [AI]  R19: 使能 DWT 周期计数器（ISR 占用率自测用；幂等） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    (void)HAL_TIM_Base_Start_IT(&htim6);   /* [AI] CEN+UIE，首拍即进 car_control_isr */
}

uint8_t car_control_ready(void)       /* [AI]  R16: 0=还在 BOOTING(25kHz 未启动) */
{
    return control_isr_started;
}

void car_feed_watchdog(void)
{
    (void)HAL_IWDG_Refresh(&hiwdg);
}

uint32_t car_battery_mv(void) { return motor_vbat_mv; }
uint8_t car_enable_switch(void) { return (uint8_t)enable_switch; }
uint8_t car_mode(void) { return (uint8_t)mode; }

/* ---- CAN command inputs (called by Task_CAN_Rx) ---- */
void car_can_velocity(int32_t vx_mmps, int32_t vy_mmps, int32_t wz_crads,
                      uint8_t enable, uint32_t now)
{
    /* [AI]  入口钳位，见 V_CMD_MAX_MMPS 注释：防下游 int32 溢出(UB) */
    if (vx_mmps >  V_CMD_MAX_MMPS)  vx_mmps =  V_CMD_MAX_MMPS;
    if (vx_mmps < -V_CMD_MAX_MMPS)  vx_mmps = -V_CMD_MAX_MMPS;
    if (vy_mmps >  V_CMD_MAX_MMPS)  vy_mmps =  V_CMD_MAX_MMPS;
    if (vy_mmps < -V_CMD_MAX_MMPS)  vy_mmps = -V_CMD_MAX_MMPS;
    if (wz_crads >  W_CMD_MAX_CRADS) wz_crads =  W_CMD_MAX_CRADS;
    if (wz_crads < -W_CMD_MAX_CRADS) wz_crads = -W_CMD_MAX_CRADS;

    cmd_vx_mmps = vx_mmps;
    cmd_vy_mmps = vy_mmps;
    cmd_wz_crads = wz_crads;
    cmd_enable = enable;
    last_jetson_ms = now;
}

void car_can_control(uint8_t cmd, uint32_t now)
{
    /* [AI]  CAN v3 SAFETY_CMD 码（§7.1）: 1=START 2=STOP 4=EMERGENCY_STOP
       5=CLEAR_ESTOP 6=CLEAR_FAULT（4 亦可经 car_estop_request_fast） */
    switch (cmd & 0x0FU) {
    case 1U: break;                            /* [AI]  START: 使能位允许即动 */
    case 2U:
        brake_all();
        mode = STOPPED;
        break;
    case 4U: estop_latched = 1U; break;
    case 5U: estop_latched = 0U; break;        /* CLEAR_ESTOP */
    case 6U: if (mode == FAULT) mode = IDLE; break;
    default: break;
    }
    last_jetson_ms = now;
}

void car_estop_request_fast(void)
{
    car_estop_fast_req = 1U;   /* [AI]  CAN 中断调用: 25kHz ISR 执行刹车 */
}

/* ---- telemetry outputs (called by Task_CAN_Tx) ---- */
void car_get_velocity(int32_t *vx_mmps, int32_t *vy_mmps, int32_t *wz_crads)
{
    *vx_mmps = meas_vx_mmps;
    *vy_mmps = meas_vy_mmps;
    *wz_crads = meas_wz_crads;
}

uint8_t car_can_status(void)
{
    uint8_t s = (uint8_t)((mode & 0x07U) << 3);
    if (remote_running != 0U) s |= 0x01U;
    if (mode == FAULT) s |= 0x02U;
    if (estop_latched != 0U) s |= 0x04U;
    return s;
}

/* [AI]  R13 调试显示数据出口（app_oled 用）：单值对齐读取天然原子，
       四轮快照允许跨拍混合（显示用途无碍），无需临界区 */
uint8_t car_reset_cause(void) { return reset_cause; }
uint8_t car_mt_index(void)    { return mt_index; }
uint8_t car_active_motor(void){ return (uint8_t)active_motor; }
uint8_t car_flags(void)       /* bit0=遥控中 bit1=急停锁存 bit2=自检中 */
{
    return (uint8_t)((remote_running != 0U ? 1U : 0U) |
                     (estop_latched != 0U ? 2U : 0U) |
                     (mt_running != 0U ? 4U : 0U));
}
void car_wheel_speeds(int32_t *out)   /* counts/ms,四轮 */
{
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) out[i] = wheel_speed[i];
}

/* ---- 25 kHz control ISR: encoder accounting, ramp, PI, PWM ----
   [AI] R20 CubeMX 迁移：改名 car_control_isr，由 stm32f4xx_it.c 的
   TIM6_DAC_IRQHandler（CubeMX 生成，USER CODE 块内）调用。UIF 检查/清零
   保留在本函数：先干活后返回，HAL_TIM_IRQHandler 再查 UIF 已清→空操作。
   时序预算不变：ISR 内禁浮点/除法链/阻塞，转速/位置/占空比单一写者。 */
void car_control_isr(void)
{
    if ((TIM6->SR & TIM_SR_UIF) == 0U) return;
    TIM6->SR = (uint16_t)~TIM_SR_UIF;
    uint32_t isr_t0 = DWT->CYCCNT;      /* [AI]  R19: 占用率计时起点 */

    /* [AI]  急停快速路径: CAN 中断置位 → 本周期(≤40µs)立即刹车锁存（§7.1） */
    if (car_estop_fast_req != 0U) {
        car_estop_fast_req = 0U;
        estop_latched = 1U;
        brake_all();
    }

    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        uint32_t current = enc_tim[i]->CNT;
        int32_t delta = (i == 1U || i == 2U)
            ? (int16_t)(uint16_t)(current - previous_count[i])
            : (int32_t)(current - previous_count[i]);
        previous_count[i] = current;
        position_counts[i] += delta;
        win_sum[i] += delta;
    }

    if (++win_cnt >= 25U) {              /* 1 ms: M-method speed + PI */
        win_cnt = 0U;
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
            int32_t sum = win_sum[i];
            win_sum[i] = 0;
            if (encoder_sign[i] == 0) continue;
            wheel_speed[i] = sum * encoder_sign[i];       /* counts/ms */
            int32_t d = v_cmd[i] - v_ramp[i];
            if (d > SPEED_RAMP_STEP) d = SPEED_RAMP_STEP;
            if (d < -SPEED_RAMP_STEP) d = -SPEED_RAMP_STEP;
            v_ramp[i] += d;                                /* slew limit */
            int32_t e = v_ramp[i] - wheel_speed[i];
            duty_q[i] += e / SPEED_KP_DIV + e / SPEED_KI_DIV;
            int32_t lim = (int32_t)MAX_DUTY_PERCENT * 100;
            if (duty_q[i] > lim) duty_q[i] = lim;
            if (duty_q[i] < -lim) duty_q[i] = -lim;
        }
    }

    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        if (mode == CALIBRATING) {
            motor_drive(i, forward_dir[i], 55U);
            continue;
        }
        if (motors[i].phase != MOTOR_DRIVE) {
            motor_brake(i);               /* hold / stop */
            continue;
        }
        if (encoder_sign[i] == 0) {
            /* Simultaneous breakaway: every motor gets the same strong
               duty from the same instant, so none starts early. */
            motor_drive(i, forward_dir[i], KICK_DUTY);
            continue;
        }
        /* signed duty: a negative target drives the reverse input */
        int32_t dq = duty_q[i];
        int dir = forward_dir[i];
        if (dq < 0) { dir = -dir; dq = -dq; }
        motor_drive(i, dir, (unsigned)(dq / 100));
    }

    /* [AI]  R19: 占用率统计。注意这里量的是"清 UIF 之后到退出"的跨度，
       不含 NVIC 入栈/出栈（那部分由硬件自动完成，无法从 C 里计时）。 */
    {
        uint32_t dt = DWT->CYCCNT - isr_t0;
        dbg_isr_last_cycles = dt;
        if (dt > dbg_isr_max_cycles) dbg_isr_max_cycles = dt;
    }
}
