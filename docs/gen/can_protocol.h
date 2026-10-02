/* 由 tools/can_check.py 从 docs/protocol-signals.yaml 生成，勿手改。 */
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define CAN_PROTO_VER 3
#define CAN_CRC_POLY 0x07u /* 初值 = 帧 Data ID，覆盖字节 0..6，无反射 */

/* 帧 ID / Data ID / 周期(Hz) */
#define CHASSIS_VEL_CMD_ID 0x100u
#define CHASSIS_VEL_CMD_DATA_ID 0x06u
#define CHASSIS_VEL_CMD_HZ 100u
#define ARM_JOINT_CMD_A_ID 0x101u
#define ARM_JOINT_CMD_A_DATA_ID 0x07u
#define ARM_JOINT_CMD_A_HZ 100u
#define ARM_JOINT_CMD_B_ID 0x102u
#define ARM_JOINT_CMD_B_DATA_ID 0x08u
#define ARM_JOINT_CMD_B_HZ 100u
#define CHASSIS_VEL_FB_ID 0x200u
#define CHASSIS_VEL_FB_DATA_ID 0x09u
#define CHASSIS_VEL_FB_HZ 100u
#define CHASSIS_DIAG_ID 0x201u
#define CHASSIS_DIAG_DATA_ID 0x0Au
#define CHASSIS_DIAG_HZ 10u
#define ARM_JOINT_FB_A_ID 0x210u
#define ARM_JOINT_FB_A_DATA_ID 0x0Bu
#define ARM_JOINT_FB_A_HZ 100u
#define ARM_JOINT_FB_B_ID 0x211u
#define ARM_JOINT_FB_B_DATA_ID 0x0Cu
#define ARM_JOINT_FB_B_HZ 100u
#define ARM_DIAG_ID 0x212u
#define ARM_DIAG_DATA_ID 0x0Du
#define ARM_DIAG_HZ 10u
#define XFER_CH_TO_ARM_ID 0x030u
#define XFER_CH_TO_ARM_DATA_ID 0x04u
#define XFER_CH_TO_ARM_HZ 1000u
#define XFER_ARM_TO_CH_ID 0x031u
#define XFER_ARM_TO_CH_DATA_ID 0x05u
#define XFER_ARM_TO_CH_HZ 1000u
#define SAFETY_CMD_ID 0x010u
#define SAFETY_CMD_DATA_ID 0x01u
#define SAFETY_CMD_HZ 0u
#define EMCY_CHASSIS_ID 0x020u
#define EMCY_CHASSIS_DATA_ID 0x02u
#define EMCY_CHASSIS_HZ 0u
#define EMCY_ARM_ID 0x021u
#define EMCY_ARM_DATA_ID 0x03u
#define EMCY_ARM_HZ 0u
#define NODE_INFO_CHASSIS_ID 0x230u
#define NODE_INFO_CHASSIS_DATA_ID 0x0Eu
#define NODE_INFO_CHASSIS_HZ 1u
#define NODE_INFO_ARM_ID 0x231u
#define NODE_INFO_ARM_DATA_ID 0x0Fu
#define NODE_INFO_ARM_HZ 1u

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

/* 16 bit 小端读写 */
static inline void can_put16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}
static inline int16_t can_get16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
#ifdef __cplusplus
}
#endif
#endif
