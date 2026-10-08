/* 由 tools/can_check.py 从 docs/protocol-signals.yaml 生成，勿手改。 */
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define CAN_PROTO_VER 4
#define CAN_CRC_POLY 0x07u /* 初值 = 帧 Data ID，覆盖字节 0..6，仅 e2e 帧，无反射 */

/* 帧 ID / Data ID / 周期(Hz) */
#define SAFETY_CMD_ID 0x010u
#define SAFETY_CMD_DATA_ID 0x01u
#define SAFETY_CMD_HZ 0u
#define EMCY_ID 0x020u
#define EMCY_DATA_ID 0x02u
#define EMCY_HZ 0u
#define NODE_INFO_ID 0x030u
#define NODE_INFO_DATA_ID 0x03u
#define NODE_INFO_HZ 1u
#define DRIVE_CMD_ID 0x100u
#define DRIVE_CMD_DATA_ID 0x04u
#define DRIVE_CMD_HZ 100u
#define ARM_CMD_A_ID 0x110u
#define ARM_CMD_A_DATA_ID 0x05u
#define ARM_CMD_A_HZ 50u
#define ARM_CMD_B_ID 0x111u
#define ARM_CMD_B_DATA_ID 0x06u
#define ARM_CMD_B_HZ 50u
#define WHEEL_POS_A_ID 0x200u
#define WHEEL_POS_A_DATA_ID 0x07u
#define WHEEL_POS_A_HZ 100u
#define WHEEL_POS_B_ID 0x201u
#define WHEEL_POS_B_DATA_ID 0x08u
#define WHEEL_POS_B_HZ 100u
#define WHEEL_SPD_ID 0x202u
#define WHEEL_SPD_DATA_ID 0x09u
#define WHEEL_SPD_HZ 100u
#define SERVO_STATE_A_ID 0x210u
#define SERVO_STATE_A_DATA_ID 0x0Au
#define SERVO_STATE_A_HZ 50u
#define SERVO_STATE_B_ID 0x211u
#define SERVO_STATE_B_DATA_ID 0x0Bu
#define SERVO_STATE_B_HZ 50u
#define DIAG_ID 0x220u
#define DIAG_DATA_ID 0x0Cu
#define DIAG_HZ 10u

static inline uint8_t can_crc8(const uint8_t *d, uint8_t n, uint8_t data_id)
{
    uint8_t crc = data_id;
    for (uint8_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* 小端读写 */
static inline void can_put16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}
static inline int16_t can_get16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline void can_put32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);          p[1] = (uint8_t)(((uint32_t)v >> 8) & 0xFF);
    p[2] = (uint8_t)(((uint32_t)v >> 16) & 0xFF); p[3] = (uint8_t)(((uint32_t)v >> 24) & 0xFF);
}
static inline int32_t can_get32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
#ifdef __cplusplus
}
#endif
#endif
