/**
 * app_canopen.c — CANopen v5 适配层（P0 基础）
 *
 * v4 的 app_can.c 在 CANopen 构建下被排除；本文件提供同名公开 API，
 * 把 FreeRTOS 的 1 kHz / 接收任务接到 CANopenNode 官方 STM32 移植层：
 *   - 接收：CAN RX 中断由 CO_driver_STM32.c 处理；任务只跑后台 CO_process。
 *   - 1 ms：canopen_app_interrupt() 驱动 SYNC/RPDO/TPDO。
 *   - 上电停在 Pre-operational（CO_app_STM32.c 的 NMT_CONTROL 已改），
 *     由 Jetson NMT Start 后进入 Operational。
 *
 * P0 只验证协议栈：OD 暂用 CS301 模板（0x6000 测试对象），
 * P1 起替换为 v5 应用对象（见 docs/superpowers/specs/2026-10-29-can-v5-canopen-design.md）。
 */
#include "app_canopen.h"
#include "app_car.h"
#include "app_imu.h"

#include "CO_app_STM32.h"
#include "OD.h"
#include "can.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

/* v5 节点分配：底盘 node 1 / 机械臂 node 2 / Jetson 127（NMT master） */
#define APP_CANOPEN_NODE_ID 1U
#define APP_CANOPEN_BAUDRATE 1000U
#define APP_CMD_DEADMAN_MS 100U

static CANopenNodeSTM32 s_node = {
    .desiredNodeID = APP_CANOPEN_NODE_ID,
    .baudrate = APP_CANOPEN_BAUDRATE,
    .timerHandle = NULL,          /* 1 ms 由 task_can_tx 调用 canopen_app_interrupt() */
    .CANHandle = &hcan1,
    .HWInitFunction = MX_CAN1_Init,
};

void can_init(void)
{
    if (canopen_app_init(&s_node) != 0) {
        Error_Handler();
    }
}

void can_rx_task_step(void)
{
    canopen_app_process();
    vTaskDelay(pdMS_TO_TICKS(1));
}

void can_tx_task_step(uint32_t now_ms)
{
    /* [AI] A16/R75（2026-10-30）：CAN 静默/睡死自愈 —— 1 kHz 检查，动作在 100 ms 量级。
     * 台架实录（docs/CAN-BRINGUP-2026-10-30.md §3.2）：臂板曾出现 MSR.SLAK=1
     * （bxCAN 自己进了 Sleep）+ 三个 TX 邮箱全部滞留，总线上表现为
     * "SDO/心跳正常但 TPDO 全灭"→ 完全静默。底盘 R51 有 v4 时代的自愈先例；
     * v5 栈本身没有这层保护，两板暴露在同一风险下。做两件最小的事：
     *   1) 发现 SLAK=1 → 软件清 SLEEP（bxCAN 退出睡眠只能软件做）；
     *   2) 三个 TX 邮箱连续 100 ms 全部非空（TME=000）→ 置 ABRQ 丢弃滞留请求，
     *      让 CANopenNode 下个周期自己重发（与 R51 的"邮箱 ABRQ 自愈"同思路）。
     * 不碰过滤器/不重初始化——那类动作风险高，这里只处理"静默"这一种已实证的形态。 */
    static uint16_t s_tx_stuck_ms;
    if ((CAN1->MSR & CAN_MSR_SLAK) != 0U) {
        CLEAR_BIT(CAN1->MCR, CAN_MCR_SLEEP);
        s_tx_stuck_ms = 0U;
    }
    if ((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0U) {
        if (s_tx_stuck_ms < 0xFFFFU) {
            s_tx_stuck_ms++;
        }
        if (s_tx_stuck_ms >= 100U) {
            SET_BIT(CAN1->TSR,
                    CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2);
            s_tx_stuck_ms = 0U;
        }
    } else {
        s_tx_stuck_ms = 0U;
    }

    (void)now_ms;
    canopen_app_interrupt();
}

/* USB v2 备用链路仍用 CRC8；与 v4 同名同语义，避免串口代码改动。 */
uint8_t crc8(const uint8_t *p, uint8_t n, uint8_t init)
{
    uint8_t crc = init;
    while (n--) {
        crc ^= *p++;
        for (uint8_t b = 0; b < 8U; ++b)
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
    return crc;
}


/* ======================= P1 应用桥接 ======================= */

static uint32_t s_last_seq;
static uint32_t s_seq_ms;
static uint8_t  s_seq_seen;

static uint8_t node_operational(void)
{
    CO_t *co = (canopenNodeSTM32 != NULL) ? canopenNodeSTM32->canOpenStack : NULL;
    return (co != NULL && co->NMT != NULL &&
            co->NMT->operatingState == CO_NMT_OPERATIONAL) ? 1U : 0U;
}

static int16_t urad_to_spd(int32_t u)
{
    int32_t v = (u >= 0) ? (u + 5000) / 10000 : (u - 5000) / 10000;
    if (v > 32767)  v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

static int16_t clamp_i16(int32_t v)
{
    if (v > 32767)  return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

void app_canopen_step(uint32_t now_ms)
{
    CO_t *co = (canopenNodeSTM32 != NULL) ? canopenNodeSTM32->canOpenStack : NULL;
    if (co == NULL) return;

    /* RPDO2 的命令序号是 deadman 的活性证据：序号不动 = 对端断流 */
    uint32_t seq = OD_RAM.x2040_cmd_seq;
    if (seq != s_last_seq) {
        s_last_seq = seq;
        s_seq_ms = now_ms;
        s_seq_seen = 1U;
    }

    uint8_t enable = 0U;
    if (node_operational() != 0U && s_seq_seen != 0U &&
        (uint32_t)(now_ms - s_seq_ms) <= APP_CMD_DEADMAN_MS &&
        (OD_RAM.x2041_cmd_flags & 0x0001U) != 0U) {
        enable = 1U;
    }

    int32_t w[CAR_MOTOR_COUNT] = {0, 0, 0, 0};
    if (enable != 0U) {
        w[0] = (int32_t)OD_RAM.x2000_W1spd * 10000;
        w[1] = (int32_t)OD_RAM.x2001_W2spd * 10000;
        w[2] = (int32_t)OD_RAM.x2002_W3spd * 10000;
        w[3] = (int32_t)OD_RAM.x2003_W4spd * 10000;
    }

    /* [AI] R74：`0x2041 cmd_flags` bit1 = 轮域陀螺稳定器开关（默认 0，架构评审修正）。
       它不是协议新对象（cmd_flags 已是 U16），所以不需重生成 EDS/OD，门禁不受影响。
       只在 Operational 时接受：Pre-operational 期间的残留值不允许打开这个隐藏内环。
       语义与风险见 `docs/CANOPEN-V5.md` §5 与 app_car.c 的 STAB_ENABLE_DEFAULT 注释。 */
    if (node_operational() != 0U) {
        uint8_t want = (uint8_t)((OD_RAM.x2041_cmd_flags & 0x0002U) != 0U);
        if (want != car_stab_enabled()) car_set_stab_en(want);
    }

    car_wheel_command_urad(w, enable, now_ms);

    /* 桥接状态对外可见（SDO 0x2050..0x2053，不占 PDO 带宽） */
    OD_RAM.x2050_cmd_en   = enable;
    OD_RAM.x2051_cmd_age_ms = (s_seq_seen != 0U)
                                ? (uint16_t)((now_ms - s_seq_ms) > 0xFFFFU
                                                 ? 0xFFFFU : (now_ms - s_seq_ms))
                                : 0xFFFFU;
    OD_RAM.x2052_car_state = (uint16_t)car_flags();
    OD_RAM.x2053_car_mode  = car_mode();

    /* 身份：build_tag 前 4 字符打包 + 复位原因（上位机不必刷 SWD 就能核对版本） */
    uint32_t packed = 0U;
    for (unsigned i = 0U; i < 4U && build_tag[i] != '\0'; ++i) {
        packed |= (uint32_t)(uint8_t)build_tag[i] << (8U * i);
    }
    OD_RAM.x2F00_tag_packed  = packed;
    OD_RAM.x2F01_reset_cause = car_reset_cause();

    /* 反馈：先写 OD，由 1 ms 的 CO_process_TPDO 按 SYNC 发出去 */
    int64_t pos_urad[CAR_MOTOR_COUNT];
    int32_t vel_urad_s[CAR_MOTOR_COUNT];
    uint8_t valid_mask;
    uint32_t epoch;
    uint64_t sample_us;
    car_encoder_snapshot(pos_urad, vel_urad_s, &valid_mask, &epoch, &sample_us);
    (void)valid_mask; (void)epoch; (void)sample_us;

    int32_t pos_counts[CAR_MOTOR_COUNT];
    car_wheel_positions(pos_counts);

    OD_RAM.x2010_W1spd_fb = urad_to_spd(vel_urad_s[0]);
    OD_RAM.x2011_W2spd_fb = urad_to_spd(vel_urad_s[1]);
    OD_RAM.x2012_W3spd_fb = urad_to_spd(vel_urad_s[2]);
    OD_RAM.x2013_W4spd_fb = urad_to_spd(vel_urad_s[3]);

    OD_RAM.x2020_W1pos = pos_counts[0];
    OD_RAM.x2021_W2pos = pos_counts[1];
    OD_RAM.x2022_W3pos = pos_counts[2];
    OD_RAM.x2023_W4pos = pos_counts[3];

    float pitch = 0.0f, roll = 0.0f;
    imu_tilt_deg(&pitch, &roll);
    uint8_t evt = imu_events();
    if (dbg_slip != 0U) evt = (uint8_t)(evt | IMU_EVT_SLIP);

    OD_RAM.x2030_imu_dyaw  = (int16_t)imu_take_yaw_delta_mrad();
    OD_RAM.x2031_imu_pitch = clamp_i16((int32_t)(pitch * 100.0f));
    OD_RAM.x2032_imu_roll  = clamp_i16((int32_t)(roll * 100.0f));
    OD_RAM.x2033_imu_evt   = evt;
    OD_RAM.x2034_imu_age   = imu_age_ms();
}
