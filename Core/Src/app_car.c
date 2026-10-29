#include "app_car.h"
#include "app_sign_store.h"
#include "app_bt.h"
#include "app_dbg.h"
#include "app_buzz.h"
#include "app_imu.h"
#include "app_car_safety.h"
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
/* [AI] R23：历史 61600 不是四轮实测值；仅 A 轮约 61602 有手测记录，B/C/D 未测。此估算只供旧台架转数路径，v2 不读取。 */
#define LEGACY_BENCH_CPR_ESTIMATE 61600L
#define PWM_HZ 25000U
#define MAX_DUTY_PERCENT 85U
#define MOTOR_TIMEOUT_MS 90000U
#define STALL_TIMEOUT_MS 2500U
#define POSITION_TOLERANCE (LEGACY_BENCH_CPR_ESTIMATE / 200L)
#define SHORT_ACCEPT       (LEGACY_BENCH_CPR_ESTIMATE / 100L)  /* 3.00 +-0.01 band */
#define OVERSHOOT_LIMIT    (LEGACY_BENCH_CPR_ESTIMATE / 8L)   /* gross overshoot -> FAULT */

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
/* [AI] R23：以下麦轮估算常数仅保留旧 CAN/台架路径，不作为 v2 标定数据。 */
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

enum { MOTOR_COUNT = WHEEL_COUNT };
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
/* [AI] R38: 预载极性被矛盾自检纠正时置位，触发重新落盘（任务态读写） */
static volatile uint8_t sign_store_dirty;
static volatile int32_t duty_q[MOTOR_COUNT];           /* 0.01 %, ISR PI state */
static struct motor_state motors[MOTOR_COUNT];
/* 25 kHz control ISR state */
static volatile int32_t win_sum[MOTOR_COUNT];          /* counts, 1 ms window */
static volatile int32_t win_raw[MOTOR_COUNT];          /* [AI] R33: 未归一 1ms 窗增量（判向用，ISR 单写者） */
static volatile uint32_t win_cnt;
static volatile int32_t wheel_speed[MOTOR_COUNT];      /* counts/ms, signed */
/* [AI] R39: 遥测 EMA 滤波值，Q8 定点（256×counts/ms）——M-method ±1 count/ms
 * 量化噪声 (≈±0.10 rad/s) 压到 1/4 以下；PI 快环仍用原始 wheel_speed 不加滞后。
 * Q8 避免整型 EMA 的截断死区（第一版 (raw-filt)>>4 在 |差|<16 时冻结——实测）。 */
static volatile int32_t wheel_filt_q8[MOTOR_COUNT];
static volatile int32_t v_cmd[MOTOR_COUNT];            /* counts/ms, supervisor */
static int32_t v_ramp[MOTOR_COUNT];                    /* ISR slew-limited target */
/* [AI] USB v2 四轮目标通过双槽快照发布，控制任务只读完整帧。 */
static wheel_target_mailbox_t wheel_command_mailbox;
static volatile uint32_t encoder_epoch;   /* [AI] R22: 编码器累计位置纪元,复位即递增 */
static atomic_uint_least32_t encoder_generation;
static wheel_uptime_clock_t uptime_clock;
static uint64_t car_uptime_us_locked(uint32_t tick_ms);
static volatile uint8_t estop_latched;
static volatile uint8_t car_estop_fast_req;   /* [AI]  CAN 中断置位, 25kHz ISR 执行急停 */
static volatile int32_t meas_vx_mmps, meas_vy_mmps, meas_wz_crads;
static uint8_t remote_running;
static uint8_t estop_button_release_required;
static volatile enum run_mode mode = IDLE;
static volatile unsigned active_motor;
volatile uint32_t motor_command;   /* SWD/CAN mailbox: 1=run 2=stop 3=cal */

static uint32_t pwm_arr;           /* [AI] R20: 取自 htim1.Init.Period(6719 = 168e6/25e3-1) */
static volatile uint32_t motor_vbat_mv;
static volatile int8_t enable_switch;
volatile uint8_t car_drive_gate_dbg;   /* [AI] R58: 驱动门自诊断位图（DIAG STATE bit6-10，协议冻结版） */
static uint8_t reset_cause;
const char build_tag[] = "R69";   /* [AI] R65（2026-10-29）：IMU 6 轴扩展 —— 同一事务 14 字节块读
  * （accel6+temp2+gyro6），新增倾角/冲击/帧间偏航增量；新增 CAN 0x032 CHASSIS_IMU（100 Hz，
  * 纯加法，事件位含 slip，沿用 R58 起 dbg_slip 判据）。陀螺 Z 路径与稳定器/航向逻辑一字未动。
  * 原 R64: 双板架构（2026-10-28）——舵机迁到机械臂板 vehical_rak_arm：删 app_servo（6 路舵机）、删 CAN 0x110/111 ARM_CMD 解析、删 0x210/211 SERVO_STATE 上行；底盘板只留 4 轮闭环 + IMU + CAN。协议 ID/字节布局未变（Jetson 侧零改动）。原 R63: ARM_CMD 换算乘法改 int64 中间量（R62 因子放大后 int32 在 raw>1230° 溢出回绕，e2e 实测 270°→23.8°）；R62: 修 CAN ARM_CMD 量化因子 10× 错（R49 起 0.1°→urad 写成 ×174.533，命令角缩至 1/10——生产路径 e2e 首次非零实测暴露，命令 Home 225° 实动 22.4°）；R61: CAN v4 冻结版——协议内容冻结（驱动门自诊断位图并入 STATE bit6-10，docs/PROTOCOL.md 标 FROZEN），急停全权走 CAN（R59 按钮 PE0 硬件故障禁用，PROTOCOL §6.1 固化）；R54: 回退 R53 栈扩容（4096 打爆 FreeRTOS 堆→任务创建失败→复位循环，uwTick 实测 800ms 循环）；栈溢出假设证伪，幽灵再锁存另查；R52: CAN 清急停跳过 PD3 门（实测断电后 PD3 读高导致所有清除路径被拒；开关藏车内，远程清除语义要求；保留电池+非FAULT+fast_req 门，按钮路径原门不变）；R51: bxCAN 过滤器运行时自愈 @1Hz（RX 静默死亡根因：FA1R.FACT0 被清+FFA1R.FFA0 被置→帧被 ACK 但被过滤器拒收不进 FIFO；SWD 三次稳定读数实锤；与邮箱 ABRQ 自愈同思路）；R50: CAN CLEAR_ESTOP 启用（用户指定：按钮藏车内难按）——0x010 CMD=5 ISR 置请求、1kHz 任务经 car_estop_clear 安全门落地（PD3 使能开关+电池+非 FAULT，与按钮短按同门）；R49: CAN v4 主链路（docs/PROTOCOL.md v4）——app_can.c 重写：下行 0x100 DRIVE_CMD(4×int16 0.01rad/s→urad/s)+0x110/111 ARM_CMD(7×uint16 0.1°→urad, A/B 组齐才落地)+0x010 SAFETY（急停 ISR 快速路径不变，START/STOP 开关 CAN 远程门 can_remote，CLEAR_ESTOP 仍仅按钮）；上行 0x200/201 WHEEL_POS(原始 counts,100Hz)+0x202 WHEEL_SPD(urad/s÷10^4,100Hz)+0x210/211 SERVO_STATE(脉宽→0.1°,50Hz)+0x220 DIAG(10Hz,STATE 位图+MODE+bus-off 计数)+0x030 NODE_INFO(1Hz)；DRIVE 断流 100ms deadman 清零+STATE.bit3；v3 XFER/VEL_FB/NODE_INFO@0x230 作废，can_arm_seen 等恒 0 仅为 UART v1 备用路径编译；新增 car_wheel_positions()/servo_get_pulse() 遥测访问器；R48: 外设配置再收敛——ADC1 改 DMA 连续转换（ContinuousConvMode/DMAContinuousRequests ENABLE + DMA2_Stream0 循环，battery_read_mv 只读缓冲均值，消除 100 Hz 任务里最多 1 ms 的 Start/Poll/Stop 阻塞）；PE0 按键改 EXTI 双沿中断（.ioc GPXTI0+上拉，EXTI0 优先级 7，ISR 只置标志、消抖仍在 100 Hz 任务）；R47: IMU 角速度 α=1/4 一级 IIR（稳定器/航向共用，imu_dbg_still_range 供 IMU_STILL_LSB 实测标定）；编码器位置符号按学习极性对齐（学习≠预置时翻转位移，防上位机里程计反向）；R46: 轮级稳定 P→PD（wheel_control_stabilize 增 D 项：角加速度域，1/4 EMA 差分，KD=150‰ 单点在 WHEEL_STAB_KD_PERMILLE，稳定链路断开时清 D 状态防踢）；BOARD-C30D.md 澄清 PB12 未启用；R45: 复位原因识别改走 HAL 宏（__HAL_RCC_GET_FLAG），car_init 不再直写 RCC->CSR——R20 规范补漏；R44: 上电默认位=Home 安全位（servo_init 输出 500/2166/2500/500/500/500µs = 指令 0/225/270/0/0/0，与 SRDF home 一致），不再先去 1500µs 中位；R43: 按钮生产化——短按=清急停锁存、长按=本地急停，台架启动/自检按钮触发移除；R42: ARM 命令落地（ARM_COMMAND→servo_set_pulse，axis J1..J6+夹爪→ch5..ch0，0~270°↔500~2500µs）；R41: 舵机测试收官；R39: 遥测 EMA；R38: encoder_sign 持久化；R37: 使能门放宽；R36-R33: 反馈守卫/死锁/判向链；R31-R30: 零位检查/轮标定首填；R28: CubeMX 全量迁移 */
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
/* [AI] R48: PE0 EXTI 事件落点（ISR 只置标志，电平在任务侧复读，逻辑全在任务）。
   优先级 7 > configMAX_SYSCALL(5)，回调内不可用 FreeRTOS API；这里连 HAL_GetTick
   都不需要——消抖时钟本来就在 100 Hz 任务里。 */
static volatile uint8_t btn_edge_flag;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
        btn_edge_flag = 1U;
}

static int button_pressed_event(uint32_t now)
{
    /* [AI] R59/R61: PE0 实测卡死恒低（GPIOE IDR 全零×8，上拉应读高）——长按误触发
       急停 + estop_button_release_required 永挂导致按钮/CAN 清锁全被屏蔽。
       v4 冻结语义（2026-10-09 用户拍板）：按钮急停禁用，CAN SAFETY_CMD(0x010
       cmd=4/5) 全权接管急停与清急停（docs/PROTOCOL.md §6.1）；按钮硬件修复后
       删除本 early-return 即恢复原语义。 */
    (void)now;
    return 0;

    static uint8_t stable;        /* 已消抖的生效电平 */
    static uint8_t raw_prev;
    static uint32_t changed_at;
    static uint32_t pressed_at;
    static uint8_t long_fired;
    /* [AI] R48: PE0 改 EXTI 双沿中断（.ioc GPXTI0 + 上拉），中断只打时间戳，
       消抖状态机仍在 100 Hz 任务里跑——ISR 里不做消抖/不做业务逻辑。
       已按下的静息态不产生边沿，需一次按下动作才会被检测（生产可接受）。 */
    uint8_t raw = raw_prev;
    if (btn_edge_flag != 0U) {
        btn_edge_flag = 0U;
        raw = (uint8_t)(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET);
    }
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

/* [AI] R48: 电池 ADC 改 DMA 连续转换（.ioc: ADC1 ContinuousConvMode/DMAContinuousRequests
   ENABLE，DMA2_Stream0 循环半字缓冲，优先级 5）——battery_read_mv 只读缓冲做均值，
   不再每 10 ms Start/Poll/Stop（原轮询最多阻塞 1 ms）。DMA 由 car_init 启动一次。
   缓冲 32 样本：转换率 ≈42 kSa/s（ADCCLK 21 MHz ÷ 492 周期），TC+HT 中断率
   = 42k/32×2 ≈ 2.6 k/s——8 样本时是 ~10 k/s，白烧 CPU（R48 体检修正）。
   [AI] R20 CubeMX 迁移：ADC1(PB0/CH8, PCLK2/4 + 480 周期采样 + 软件触发)
   由 MX_ADC1_Init 配置（原寄存器版的分频/采样时间修正已进 .ioc）；
   F4 的 HAL 无 ADC 校准 API（无需移植原 CAL 位操作）。 */
#define ADC_BATT_BUF_N   32U
static volatile uint16_t adc_batt_buf[ADC_BATT_BUF_N];

static uint32_t battery_read_mv(void)
{
    uint32_t sum = 0U;
    for (unsigned i = 0; i < ADC_BATT_BUF_N; ++i)
        sum += adc_batt_buf[i];
    return sum * 3300UL * 11UL / (4095UL * ADC_BATT_BUF_N);
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
    taskENTER_CRITICAL();
    wheel_control_encoder_generation_write_begin(&encoder_generation);
    isr_state_lock();
    encoder_epoch++;   /* [AI] R22: 位置清零即换纪元,host 侧据此丢弃旧累计位 */
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        enc_tim[i]->CNT = 0;
        previous_count[i] = 0;
        position_counts[i] = 0;
        /* [AI] R31: 恢复自动判向（kick 自检实测"正占空比→计数方向"）。
         * R30 曾用标定表预设 encoder_sign，与闭环内部约定不一致，
         * 导致 FL/RR 闭环正反馈飞车——标定表只提供 CPR/半径/杠杆换算。 */
        encoder_sign[i] = 0;
        duty_q[i] = 0;
        v_ramp[i] = 0;
        v_cmd[i] = 0;
        wheel_speed[i] = 0;
        win_sum[i] = 0;
        win_raw[i] = 0;   /* [AI] R33 */
        motors[i].phase = MOTOR_BRAKE;
    }
    wheel_control_encoder_generation_write_end(&encoder_generation);
    isr_state_unlock();
    taskEXIT_CRITICAL();
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
/* [AI] R43: start_motion_test() 已随按钮生产化移除——自检序列（mt_*）仅存
   遗留状态机与停止分支，无启动入口，保留以最小化本次改动面；
   如需彻底退役再单独清理 mt_* 全链路。 */

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
            int32_t error = (int32_t)(target_turns[i] * LEGACY_BENCH_CPR_ESTIMATE) - position;
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
        int32_t error = (int32_t)(target_turns[i] * LEGACY_BENCH_CPR_ESTIMATE) - position;
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
        v_cmd[i] = (error > LEGACY_BENCH_CPR_ESTIMATE / 4L) ? BENCH_CRUISE
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
/* [AI] R23：旧 body wz 注入与航向保持仅留作历史记录，活动稳定器只改四轮目标。 */
#if 0
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
#endif

/* [AI] R22：关闭 body 指令域修正；新补偿只在四轮目标生成后运行。 */
static void stabilizer_apply(int32_t *vx, int32_t *vy, int32_t *wz)
{
    (void)vx;
    (void)vy;
    (void)wz;
    stab_scale = 256U;
    stab_slip_ms = 0U;
    dbg_stab_corr = 0;
    dbg_stab_scale = 256U;
    dbg_slip = 0U;
}

static int32_t wheel_target_limit_urad_s(void)
{
    uint32_t max_cpr = 0U;
    if (!wheel_control_calibration_valid(&car_wheel_calibration))
        return 0;
    for (unsigned i = 0U; i < MOTOR_COUNT; ++i) {
        if (car_wheel_calibration.counts_per_revolution[i] > max_cpr)
            max_cpr = car_wheel_calibration.counts_per_revolution[i];
    }
    if (max_cpr == 0U)
        return 0;
    int64_t limit = (int64_t)V_CMD_LIMIT * WHEEL_URAD_PER_REV * 1000LL /
                    max_cpr;
    return limit > INT32_MAX ? INT32_MAX : (int32_t)limit;
}

/* [AI] R46: 轮级稳定 D 项参数与状态（仅在 ctrl 任务上下文使用，非 ISR）。
 * KD 取值保守（P 的 1.5 倍千分比），待实车标定复核；微分经 1/4 EMA
 * 抑制编码器量化噪声，调用间隔 >100 ms 视为不连续（D 项清零重来）。 */
#define WHEEL_STAB_KD_PERMILLE   150U
static int32_t  stab_yaw_err_prev;
static uint32_t stab_prev_tick;
static int32_t  stab_deriv_f;
static uint8_t  stab_prev_valid;

static int wheel_stabilize_targets(int32_t target_urad_s[MOTOR_COUNT],
                                   const int32_t measured_urad_s[MOTOR_COUNT],
                                   int32_t imu_yaw_rate_mrad_s)
{
    int32_t corrections[MOTOR_COUNT];
    int32_t yaw_err = 0;
    uint32_t now = HAL_GetTick();
    int32_t deriv = 0;

    /* [AI] R46: 航向角速度误差差分 + 1/4 EMA 低通（D 项输入）。
       间隔异常（首次/停顿/时间回绕）时不输出微分并清状态。 */
    if (stab_prev_valid != 0U) {
        uint32_t dt = (uint32_t)(now - stab_prev_tick);
        if (dt != 0U && dt <= 100U) {
            int64_t de = (int64_t)(imu_yaw_rate_mrad_s - stab_yaw_err_prev) *
                         1000 / (int32_t)dt;
            stab_deriv_f += (int32_t)(de - stab_deriv_f) / 4;
            deriv = stab_deriv_f;
        } else {
            stab_deriv_f = 0;
        }
    } else {
        stab_deriv_f = 0;
    }
    stab_yaw_err_prev = imu_yaw_rate_mrad_s;
    stab_prev_tick = now;
    stab_prev_valid = 1U;

    if (!wheel_control_stabilize(
            target_urad_s, measured_urad_s, imu_yaw_rate_mrad_s,
            &car_wheel_calibration, 100000U, 100U, corrections,
            &yaw_err, deriv, WHEEL_STAB_KD_PERMILLE)) {
        stab_prev_valid = 0U;
        return 0;
    }
    /* [AI] R46: 差分基准用本轮真实误差（含编码器侧），比用 IMU 原值更准 */
    stab_yaw_err_prev = yaw_err;
    if (!wheel_control_common_saturate(
            target_urad_s, wheel_target_limit_urad_s()))
        return 0;

    dbg_stab_corr = corrections[0] / 10000;
    dbg_stab_scale = 256U;
    dbg_slip = (uint8_t)(corrections[0] != 0 || corrections[1] != 0 ||
                         corrections[2] != 0 || corrections[3] != 0);
    return 1;
}

/* [AI] R46: 稳定链路不连续（指令零/数据不新鲜/急停）时清 D 项状态，
   防止恢复驱动瞬间旧微分造成一脚"踢"。 */
static void wheel_stabilize_reset(void)
{
    stab_prev_valid = 0U;
    stab_deriv_f = 0;
}

/* [AI] R14: 遥控与麦轮自检共用的驱动下发路径（稳定器修正 → IK → 四路目标） */
/* [AI] R23：上方为历史说明；当前稳定修正位于 IK 后，只作用四轮目标域。 */
static void drive_apply(int32_t vx, int32_t vy, int32_t wz)
{
    stabilizer_apply(&vx, &vy, &wz);
    int32_t target_counts_ms[MOTOR_COUNT];
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].phase = MOTOR_DRIVE;
        target_counts_ms[i] = ik_wheel_target(i, vx, vy, wz);
    }
    if (car_wheel_calibration_ready() != 0U &&
        imu_sample_fresh(HAL_GetTick())) {
        int64_t position[MOTOR_COUNT];
        int32_t measured[MOTOR_COUNT];
        int32_t target_urad_s[MOTOR_COUNT];
        uint8_t valid_mask;
        uint32_t epoch;
        uint64_t sample_us;
        car_encoder_snapshot(position, measured, &valid_mask, &epoch,
                             &sample_us);
        (void)position;
        (void)epoch;
        if (valid_mask == 0x0FU &&
            wheel_control_sample_fresh_from_clock(
                HAL_GetTick, (uint32_t)(sample_us / 1000ULL),
                (uint8_t)(sample_us != 0U)) &&
            wheel_control_normalized_counts_to_urad_s(
                target_counts_ms, &car_wheel_calibration, target_urad_s) &&
            wheel_stabilize_targets(
                target_urad_s, measured,
                (int32_t)(imu_yaw_rate_dps() * 17.453293f)) &&
            wheel_control_command_to_counts(
                target_urad_s, &car_wheel_calibration,
                target_counts_ms) != 0U) {
            for (unsigned i = 0U; i < MOTOR_COUNT; ++i)
                v_cmd[i] = target_counts_ms[i];
            return;
        }
    }
    wheel_stabilize_reset();   /* [AI] R46: 稳定链路未走通，清 D 项状态 */
    for (unsigned i = 0U; i < MOTOR_COUNT; ++i)
        v_cmd[i] = target_counts_ms[i];
}

void car_control_step(uint32_t now)
{
    wheel_target_snapshot_t wheel_command = {0};
    uint8_t wheel_command_valid =
        wheel_target_mailbox_read(&wheel_command_mailbox, &wheel_command);
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

    uint8_t controls_allowed =
        car_control_inputs_allowed(estop_latched, car_estop_fast_req);
    /* operator inputs: [AI] R43 生产化语义 —— 短按=清急停锁存，长按=本地急停。
       原台架启动/麦轮自检的按钮触发已移除；电机启动只走 USB/CAN 显式指令路径。 */
    int bev = button_pressed_event(now);
    if (controls_allowed == 0U) {
        motor_command = 0U;
        estop_button_release_required =
            (uint8_t)(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET);
        brake_all();
        remote_running = 0U;
        mt_running = 0U;
        mt_stop_req = 0U;
        if (mode != FAULT) mode = STOPPED;
        /* [AI] R43 急停锁存态下的短按清锁：必须在这里处理（本分支 return 早于
           下方 bev 分支）。release_required 保证按住不放不会立刻清锁；
           car_estop_clear 内部再过安全门（使能开关+电池+非 FAULT），不满足则拒绝。 */
        if (bev == 1 && estop_button_release_required == 0U) {
            (void)car_estop_clear();
        }
        return;
    }
    if (estop_button_release_required != 0U) {
        bev = 0;
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_SET)
            estop_button_release_required = 0U;
    }

    /* [AI] R43 按钮生产化语义（未锁存态）：
       短按 = 若自检在跑则停止（自检为遗留测试路径，生产上不会启动）；
       长按 = 本地急停：置 car_estop_fast_req，由 25kHz ISR ≤40µs 刹车锁存
       （与 CAN 0x010 / USB LATCH 同一快速路径），并要求按钮释放后才接受清锁。
       清锁后的复位动作在上方锁存分支处理。 */
    if (bev == 1) {
        if (mt_running != 0U) {
            mt_stop_req = 1U;
        }
    } else if (bev == 2) {
        if (mt_running != 0U) {
            mt_stop_req = 1U;
        }
        car_estop_request_fast();
        estop_button_release_required = 1U;
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

    /* encoder sign detection
     * [AI] R33: 改用 1 ms 窗增量判向（阈值 20 counts/ms ≈ 2 rad/s）。
     * 旧逻辑用位置累计 |raw|≥100：上电后编码器输入漂移/噪声缓慢累计即可
     * 误触发，学到随机极性 → 闭环正反馈飞车（实测 A/D 轮 27 rad/s 飞车）。
     * 窗增量阈值只有真实 kick 转速才够得到；kick 自身 75% 占空比远超阈值。
     * [AI] R38: 极性来源优先 Flash 预载（car_init）；本循环另做矛盾自检——
     * 强正向驱动(duty>40%)时 raw 增量极性若与预载极性相反（换线/预载坏），
     * 立即纠正并重新落盘，防止 R30 式正反馈飞车在预载路径复活。 */
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        if (encoder_sign[i] == 0 && (win_raw[i] >= 20 || win_raw[i] <= -20)) {
            encoder_sign[i] = win_raw[i] > 0 ? 1 : -1;
            duty_q[i] = 5000;             /* PI hand-off duty */
            v_ramp[i] = wheel_speed[i];   /* hand off at the kick speed */
        } else if (encoder_sign[i] != 0 && duty_q[i] > (int32_t)4000U &&
                   (win_raw[i] >= 20 || win_raw[i] <= -20) &&
                   ((win_raw[i] > 0 ? 1 : -1) != encoder_sign[i])) {
            encoder_sign[i] = (int8_t)(-encoder_sign[i]);   /* 预载极性错：翻转 */
            sign_store_dirty = 1U;
        }
    }
    /* [AI] R38: 四轮极性齐且"本次开机尚未落盘/有纠正" → 追加一条记录
     * （append-only，见 app_sign_store.c）。失败不重试（避免每 10ms 撞
     * Flash），下次上电 kick 自检兜底。 */
    {
        static uint8_t store_pending;   /* 0=未存 1=已存 */
        uint8_t need_store = sign_store_dirty;
        int8_t snapshot_signs[MOTOR_COUNT];
        uint8_t all_learned = 1U;
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
            snapshot_signs[i] = encoder_sign[i];
            all_learned = (uint8_t)(all_learned && snapshot_signs[i] != 0);
        }
        if (all_learned && (need_store != 0U || store_pending == 0U)) {
            if (sign_store_save(snapshot_signs) == 0U) {
                sign_store_dirty = 0U;
            }
            store_pending = 1U;   /* 无论成败，本开机不再尝试 */
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
        uint32_t command_ms = wheel_command.command_tick;
        uint32_t link_now = HAL_GetTick();
        int32_t age = (int32_t)(link_now - command_ms);
        int link = wheel_command_valid != 0U &&
                   wheel_command.source == 1U && command_ms != 0U && age >= 0 &&
                   (uint32_t)age <= RC_TIMEOUT_MS;
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
            if (link && wheel_command.enabled != 0U &&
                wheel_command.source == 1U) {
                /* [AI] R22: USB v2 轮级直通 —— 目标已换算为 counts/ms,无
                   body-wz 前置注入、无零偏航航向保持(IMU 补偿迁到轮目标域
                   需单独台架验证,验证前补偿为零)。钳位在命令入口已做。 */
                for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
                    motors[i].phase = MOTOR_DRIVE;
                    v_cmd[i] = wheel_command.target_counts_ms[i];
                }
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
static void publish_wheel_command(const int32_t target_counts_ms[MOTOR_COUNT],
                                  uint8_t source, uint8_t enabled,
                                  uint32_t now)
{
    wheel_target_snapshot_t snapshot = {
        .command_tick = now,
        .source = source,
        .enabled = enabled
    };
    for (unsigned i = 0U; i < MOTOR_COUNT; ++i)
        snapshot.target_counts_ms[i] =
            target_counts_ms != NULL ? target_counts_ms[i] : 0;

    taskENTER_CRITICAL();
    wheel_target_mailbox_publish(&wheel_command_mailbox, &snapshot);
    taskEXIT_CRITICAL();
}

void car_init(void)
{
    /* [AI] R20 CubeMX 迁移：原时钟兜底(HSI→HSE 168MHz)已删——
       SystemClock_Config 由 CubeMX 生成且 HSE 8MHz+PLL 已确认（AGENTS.md 红线 3，
       重生成后必查 HSE 未被抹回 HSI）。 */

    /* [AI] R45: 复位原因改走 HAL 宏（__HAL_RCC_GET_FLAG），不再直写 RCC->CSR——
       R20 起外设/寄存器级操作一律由 CubeMX 生成或 HAL 提供，应用层只调 API。 */
    {
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) reset_cause = (uint8_t)'B';
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) reset_cause = (uint8_t)'I';
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) reset_cause = (uint8_t)'S';
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) reset_cause = (uint8_t)'P';
        else reset_cause = (uint8_t)'-';
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }
    wheel_target_mailbox_init(&wheel_command_mailbox);
    /* [AI] R48: 启动 ADC1 DMA 连续转换（MX_ADC1_Init 已在前，时钟/DMA 均就绪） */
    (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_batt_buf, ADC_BATT_BUF_N);
    atomic_store_explicit(&encoder_generation, 0U, memory_order_relaxed);
    uptime_clock = (wheel_uptime_clock_t){0};
    encoder_epoch = 1U;   /* [AI] R22: 上电纪元从 1 起(0 留给"未初始化") */
    /* [AI] R31: 不再用标定表预设 encoder_sign（car_encoder_reset 内注释），
     * 保留 kick 自检：ARMED 首驱 kick 一拍→观测 raw 极性→闭环保守接管 */
    /* [AI] R38: Flash 扇区满则先擦（此刻 25kHz 未启动、电机未驱动，整片擦
     * ~1-2s 总线停顿安全）；有历史记录则直接预载极性，跳过上电 kick 跳拍。
     * 预载错的防线在 car_control_step 的矛盾自检（强正向驱动极性反转即纠正）。 */
    if (sign_store_prepare() == 0U) {
        int8_t stored_signs[MOTOR_COUNT];
        if (sign_store_load(stored_signs) != 0U) {
            for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
                encoder_sign[i] = stored_signs[i];
            }
        }
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
    BOOT_MARK(21);   /* 进入 car_init 后段 */
    imu_init();      /* [AI]  R9: 板载 ICM20948 陀螺仪(未找到时静默降级) */
    /* [AI] R66: 蜂鸣器(PA8) —— 上电自检短响一声 */
    BOOT_MARK(22);   /* IMU 探测已过 */
    buzz_init();
    buzz_alert(BUZZ_BOOT);
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
uint8_t car_estop_clear_allowed(void)
{
    return car_safety_clear_allowed(
        (uint8_t)(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET),
        (uint8_t)(mode != FAULT &&
                  motor_vbat_mv >= CAR_BATTERY_UV_THRESHOLD_MV),
        car_estop_fast_req);
}
uint8_t car_wheel_calibration_ready(void)
{
    return (uint8_t)wheel_control_calibration_valid(&car_wheel_calibration);
}

/* [AI] R22: USB v2 轮级直通命令入口(serial 层任务上下文调用)。
   四轮 rad/s(urad/s) → counts/ms 定点换算,入口钳位 V_CMD_LIMIT;
   不做 body-twist IK、不动 stabilizer。顺序 = 前左/前右/后左/后右
   (v2 合同固定),与电机序号 0..3 = FL/FR/RL/RR 一致(ik_sy/ik_sw 布局)。 */
/* [AI] R23：本函数应用轮域稳定补偿；上方 R22 注释中的“不动 stabilizer”已被本说明取代。 */
/* [AI] Task 3：实际换算只读 car_wheel_calibration；默认未签核时强制清零并闭锁。 */
void car_wheel_command_urad(const int32_t wheel_urad_s[MOTOR_COUNT],
                            uint8_t enable, uint32_t now)
{
    int32_t target_urad_s[MOTOR_COUNT] = {0};
    int32_t target_counts_ms[MOTOR_COUNT] = {0};
    int64_t position_urad[MOTOR_COUNT];
    int32_t measured_urad_s[MOTOR_COUNT];
    uint8_t valid_mask = 0U;
    uint32_t epoch;
    uint64_t sample_us;

    /* [AI] R58/R61: 驱动门自诊断位图（DIAG/NODE_INFO STATE bit6-10 上报，
       v4 冻结版已纳入协议——docs/protocol-signals.yaml state_bits）：
       bit0=远程门 bit1=标定 bit2=IMU新鲜 bit3=快照新鲜 bit4=稳定+换算（0 起） */
    car_drive_gate_dbg = (uint8_t)((enable != 0U ? 1U : 0U) |
                                   (car_wheel_calibration_ready() != 0U ? 2U : 0U) |
                                   (imu_sample_fresh(now) != 0 ? 4U : 0U));
    enable = (uint8_t)(enable != 0U && wheel_urad_s != NULL &&
                       car_wheel_calibration_ready() != 0U &&
                       imu_sample_fresh(now) != 0);
    if (enable != 0U) {
        car_encoder_snapshot(position_urad, measured_urad_s, &valid_mask,
                             &epoch, &sample_us);
        (void)epoch;
        /* [AI] R37: 不再要求 valid_mask==0x0F——未初始化轮由输出门开环 kick
         * 学习极性（kick 仅在 v_cmd 非零时触发），已初始化轮正常闭环 */
        enable = (uint8_t)(wheel_control_sample_fresh_from_clock(
                HAL_GetTick, (uint32_t)(sample_us / 1000ULL),
                (uint8_t)(sample_us != 0U)));
        car_drive_gate_dbg = (uint8_t)(car_drive_gate_dbg | (enable != 0U ? 8U : 0U));
    }
    if (enable != 0U) {
        for (unsigned i = 0U; i < MOTOR_COUNT; ++i)
            target_urad_s[i] = wheel_urad_s[i];
        enable = (uint8_t)(wheel_stabilize_targets(
            target_urad_s, measured_urad_s,
            (int32_t)(imu_yaw_rate_dps() * 17.453293f)) &&
            wheel_control_command_to_counts(
                target_urad_s, &car_wheel_calibration, target_counts_ms));
        car_drive_gate_dbg = (uint8_t)(car_drive_gate_dbg | (enable != 0U ? 16U : 0U));
    }
    if (enable == 0U) {
        wheel_stabilize_reset();   /* [AI] R46: 稳定链路未走通，清 D 项状态 */
        for (unsigned i = 0U; i < MOTOR_COUNT; ++i)
            target_counts_ms[i] = 0;
    }

    publish_wheel_command(target_counts_ms, 1U, enable, now);
}


static uint64_t car_uptime_us_locked(uint32_t tick_ms)
{
    return wheel_control_uptime_us(&uptime_clock, tick_ms);
}

uint64_t car_uptime_us(uint32_t tick_ms)
{
    uint64_t uptime_us;
    taskENTER_CRITICAL();
    uptime_us = car_uptime_us_locked(tick_ms);
    taskEXIT_CRITICAL();
    return uptime_us;
}

/* [AI] R22: 编码器一致采样快照(serial 层 100Hz 组 DRIVE_STATE 用)。
   isr_state_lock 临界区内取位置/速度同拍快照;位置经 encoder_sign 归一
   (正=标定前进方向)后换算微弧度;速度 counts/ms → urad/s。
   valid_mask=1 表示该轮 encoder_sign 已标定;未标定轮不伪装有效。 */
void car_encoder_snapshot(int64_t pos_urad[MOTOR_COUNT],
                          int32_t vel_urad_s[MOTOR_COUNT],
                          uint8_t *valid_mask, uint32_t *epoch,
                          uint64_t *sample_us)
{
    int32_t raw_pos[MOTOR_COUNT], raw_spd[MOTOR_COUNT];
    int32_t filt_q8_snap[MOTOR_COUNT];   /* [AI] R39: EMA Q8 同拍快照 */
    int8_t signs[MOTOR_COUNT];
    uint8_t sampled_mask = 0U;
    uint32_t snapshot_epoch;
    uint64_t snapshot_sample_us;

    for (;;) {
        uint32_t generation_before = atomic_load_explicit(
            &encoder_generation, memory_order_acquire);
        if ((generation_before & 1U) != 0U)
            continue;

        sampled_mask = 0U;
        taskENTER_CRITICAL();
        uint32_t generation_locked = atomic_load_explicit(
            &encoder_generation, memory_order_acquire);
        if (generation_locked != generation_before ||
            (generation_locked & 1U) != 0U) {
            taskEXIT_CRITICAL();
            continue;
        }
        isr_state_lock();
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
            raw_pos[i] = position_counts[i];
            signs[i] = encoder_sign[i];
            if (signs[i] != 0) {
                /* [AI] R39: EMA 滤波值以 Q8 存放，此处还原 counts/ms 整数部分
                   供 encoder_state 走统一换算；亚 LSB 精度由下方 Q8 直换补足 */
                raw_spd[i] = wheel_filt_q8[i] >> 8;
                filt_q8_snap[i] = wheel_filt_q8[i];   /* [AI] 临界区内同拍快照 */
                sampled_mask |= (uint8_t)(1U << i);
            } else {
                raw_spd[i] = 0;
            }
        }
        snapshot_epoch = encoder_epoch;
        snapshot_sample_us = car_uptime_us_locked(HAL_GetTick());
        isr_state_unlock();
        taskEXIT_CRITICAL();

        uint32_t generation_after = atomic_load_explicit(
            &encoder_generation, memory_order_acquire);
        if (wheel_control_encoder_generation_stable(
                generation_before, generation_after))
            break;
    }

    (void)wheel_control_encoder_state(
        raw_pos, raw_spd, sampled_mask, &car_wheel_calibration,
        pos_urad, vel_urad_s, valid_mask);
    /* [AI] R47: 位置符号一致性——encoder_state 用预置 direction 归一位移，
       但速度用的是 ISR 内学习极性(encoder_sign)归一后的 wheel_speed。
       极性自检翻转/换线后两者可能相反：位置前向、速度反向（或反之），
       上位机里程计直接坏。此处按学习极性对齐：学习≠预置则翻转位移。 */
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        if ((sampled_mask & (1U << i)) != 0U &&
            signs[i] != car_wheel_calibration.encoder_direction[i])
            pos_urad[i] = -pos_urad[i];
    }
    /* [AI] R39: 速度用 Q8 EMA 直换 urad/s——保留 1/256 count 亚 LSB 精度，
       遥测量化噪声从 ±0.10 rad/s 降到 ±0.4 mrad/s 量级（滤波后）。
       counts_ms_q8 × (URAD_PER_REV×1000) / (CPR×256) = urad/s（64 位中间量） */
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        if ((sampled_mask & (1U << i)) != 0U) {
            vel_urad_s[i] = (int32_t)((int64_t)filt_q8_snap[i] *
                WHEEL_URAD_PER_REV * 1000LL /
                ((int64_t)car_wheel_calibration.counts_per_revolution[i] << 8));
        }
    }
    *epoch = snapshot_epoch;
    *sample_us = snapshot_sample_us;
}

void car_estop_request_fast(void)
{
    car_estop_fast_req = 1U;   /* [AI]  CAN 中断调用: 25kHz ISR 执行刹车 */
}

/* [AI] R52: 清急停内部实现——require_pd3 区分按钮/CAN 路径。
   PD3 使能开关藏车内难达（用户 2026-10-09），CAN 清除路径跳过该门，
   保留电池欠压 + 非 FAULT + fast_req==0 三门；按钮路径维持原三重门。 */
static uint8_t car_estop_clear_impl(uint8_t require_pd3)
{
    int32_t zero_target[MOTOR_COUNT] = {0};
    uint8_t calibrating = (uint8_t)(mode == CALIBRATING);

    taskENTER_CRITICAL();
    isr_state_lock();
    if (car_safety_clear_allowed(
            require_pd3 ? (uint8_t)(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) ==
                                     GPIO_PIN_RESET)
                        : 1U,
            (uint8_t)(mode != FAULT &&
                      motor_vbat_mv >= CAR_BATTERY_UV_THRESHOLD_MV),
            car_estop_fast_req) == 0U) {
        isr_state_unlock();
        taskEXIT_CRITICAL();
        return 0U;
    }
    car_safety_clear_pending(
        &motor_command, &estop_latched, &remote_running, &mt_running,
        &mt_stop_req, &calibrating, v_cmd, v_ramp, duty_q, MOTOR_COUNT);
    if (mode != FAULT) mode = STOPPED;
    for (unsigned i = 0U; i < MOTOR_COUNT; ++i) {
        motors[i].phase = MOTOR_BRAKE;
    }
    estop_button_release_required =
        (uint8_t)(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET);
    publish_wheel_command(zero_target, 1U, 0U, HAL_GetTick());
    isr_state_unlock();
    taskEXIT_CRITICAL();
    return 1U;
}

/* [AI] R52: 按钮路径清急停（PD3 门保留）。 */
uint8_t car_estop_clear(void)
{
    return car_estop_clear_impl(1U);
}

/* [AI] R52: CAN 路径清急停（跳过 PD3 门——开关藏车内，远程清除语义要求；
   其余门同按钮路径）。app_can 的 CLEAR_ESTOP(cmd=5) 调用。 */
uint8_t car_estop_clear_can(void)
{
    return car_estop_clear_impl(0U);
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

/* [AI] R45: 编码器累计位置快照（CAN v4 WHEEL_POS 用，原始 quad counts，
   上电=0）。单值读取天然原子，四轮快照允许跨拍混合（遥测用途无碍）。 */
void car_wheel_positions(int32_t *out)
{
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) out[i] = position_counts[i];
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
        buzz_alert(BUZZ_FAULT);   /* [AI] R66: 急停锁存长响（ISR 里只置请求位） */
        bt_event("ESTOP");        /* [AI] R67: 蓝牙事件行（ISR 里只拷贝+置位） */
    }
    if (estop_latched != 0U) {
        brake_all();
        for (unsigned i = 0U; i < MOTOR_COUNT; ++i) {
            v_ramp[i] = 0;
            duty_q[i] = 0;
        }
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
            win_raw[i] = sum;   /* [AI] R33: 判向用原始窗增量（先于归一） */
            if (encoder_sign[i] == 0) continue;
            wheel_speed[i] = sum * encoder_sign[i];       /* counts/ms */
            /* [AI] R39: 遥测 EMA，α=1/16（τ≈16ms @1kHz），Q8 无截断死区 */
            wheel_filt_q8[i] += ((wheel_speed[i] << 8) - wheel_filt_q8[i]) >> 4;
            if (estop_latched != 0U) {
                v_cmd[i] = 0;
                v_ramp[i] = 0;
                duty_q[i] = 0;
                continue;
            }
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
        /* [AI] R34: kick 触发条件——未初始化轮仅在"该轮有非零目标"时才开环
         * 踢动学习极性；零目标永远 BRAKE，避免零指令下轮子意外转动 */
        uint8_t phase_drive = (uint8_t)(motors[i].phase == MOTOR_DRIVE);
        if (encoder_sign[i] == 0 && v_cmd[i] == 0) phase_drive = 0U;
        car_motor_output_t output = car_motor_output_gate(
            estop_latched, (uint8_t)(mode == CALIBRATING),
            phase_drive,
            (uint8_t)(encoder_sign[i] == 0), forward_dir[i], duty_q[i]);
        if (output.reset_controller != 0U) {
            v_cmd[i] = 0;
            v_ramp[i] = 0;
            duty_q[i] = 0;
        }
        if (output.action == CAR_MOTOR_OUTPUT_BRAKE) {
            motor_brake(i);
        } else {
            motor_drive(i, output.direction, output.duty_percent);
        }
    }

    /* [AI]  R19: 占用率统计。注意这里量的是"清 UIF 之后到退出"的跨度，
       不含 NVIC 入栈/出栈（那部分由硬件自动完成，无法从 C 里计时）。 */
    {
        uint32_t dt = DWT->CYCCNT - isr_t0;
        dbg_isr_last_cycles = dt;
        if (dt > dbg_isr_max_cycles) dbg_isr_max_cycles = dt;
    }
}
