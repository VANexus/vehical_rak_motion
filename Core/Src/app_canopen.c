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
#include "app_can.h"
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

uint8_t can_arm_seen(void)       { return 0U; }
uint32_t can_arm_last_ms(void)   { return 0U; }
int16_t can_arm_fault_code(void) { return 0; }

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
