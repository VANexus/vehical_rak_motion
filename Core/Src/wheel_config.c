#include "wheel_config.h"

/* 唯一一份轮序/方向/标定常数的定义。见 wheel_config.h 说明。 */
const int8_t wheel_forward_dir[WHEEL_COUNT] = {WHEEL_FWD_DIR_LIST};
const int8_t wheel_encoder_dir[WHEEL_COUNT] = {WHEEL_ENC_DIR_LIST};
const int8_t wheel_yaw_dir[WHEEL_COUNT] = {WHEEL_YAW_DIR_LIST};
const int8_t wheel_lateral_dir[WHEEL_COUNT] = {WHEEL_LATERAL_DIR_LIST};
const uint32_t wheel_cpr[WHEEL_COUNT] = {WHEEL_CPR_LIST};
