#ifndef WHEEL_CONFIG_H
#define WHEEL_CONFIG_H

/* 底盘轮序 / 方向 / 标定常数的**唯一权威定义**。
 *
 * 约定：任何地方需要轮序、电机方向、编码器方向、减速比、轮径、旋转力臂，
 * 一律引用本文件，不得在 .c / 文档 / 上位机配置里另写一份数值。
 * 文档只描述"权威定义在本文件"，不复述具体数字。
 *
 * 数值来源：R30 台架烟测标定表（A 轮手测 61602 计数/圈，B/C/D 用台架估算 61600）。
 * 修改数值前必须先有实车标定结论。 */

#include <stdint.h>

#define WHEEL_COUNT 4U

/* 电机索引 ↔ 硬件通道（与 app_car.c motor_pair() 一致）：
 *   0 = A = TIM10_CH1(+) / TIM11_CH1(-)
 *   1 = B = TIM9_CH1(+)  / TIM9_CH2(-)
 *   2 = C = TIM1_CH1(+)  / TIM1_CH2(-)
 *   3 = D = TIM1_CH3(+)  / TIM1_CH4(-)
 * 对象字典 0x2000..0x2003 与 0x2010.. 一致按本顺序排列。 */
typedef enum {
    WHEEL_A = 0,
    WHEEL_B = 1,
    WHEEL_C = 2,
    WHEEL_D = 3
} wheel_id_t;

/* 列表宏：保证每个数值只出现一次，数组与标定结构体共用同一组数。 */
#define WHEEL_CPR_LIST       61600U, 61600U, 61600U, 61600U
/* 编码器极性乘子：乘到原始 quad counts 上，使"前进"为正。
 * 与下面的指令方向表**相互独立**：电机极性和编码器接线极性是两套硬件事实，
 * 不存在单一向量能同时满足"+指令=前进"与"前进时计数为正"。
 * 数值依据（用户 2026-11-16 指定的上一轮实测极性表 {−,+,+,−}）：
 *   W1/W4 前进时原始计数为负、W2/W3 为正；乘本表后统一为正。 */
#define WHEEL_ENC_DIR_LIST   -1, 1, 1, -1
/* 指令方向表：正向指令下四个轮子的物理转向对齐。
 * 实测依据（W1..W4 = 左前/右前/左后/右后，A/B/C/D 同序）：
 *   轮次1，表 {+1,-1,-1,+1}：W1 后退、W2 前进、W3 后退、W4 前进
 *   轮次2，表 {-1,-1,+1,+1}：W1 后退、W2 后退、W3 前进、W4 前进
 *   规律是"前两个(W1/W2) / 后两个(W3/W4)"分组，不是左右侧：
 *   W3/W4 在轮次2 已正确，W1/W2 再取反一次 ⇒ 全 +1，四轮同向。
 * 两轮的"后退/前进"均为用户现场目视，不是编码器推断。 */
#define WHEEL_FWD_DIR_LIST   1, 1, 1, 1

/* 轮半径 / 力臂的唯一权威（2026-10-14 实测，见
 * `rak/src/rak_description/urdf/rak_chassis.urdf.xacro` 的 wheelbase=0.245、
 * track=0.275、轮径 Ø5.5 cm；跨仓一致性由伞形仓 tools/geometry_gate.py 校验）。
 * 旧值 32500 / 150000 是台架期估计值，2026-11-16 修正并保留原因：
 *   轮径 32.5→27.5 mm：旧值使自检行程与 OLED 体速度偏短 18%（301.7/356.5）；
 *   力臂 150→260 mm：L = 轴距/2 + 轮距/2 = 122.5 + 137.5，旧值使原地转比例错 1.733×。
 * 不要再把这两个旧值抄回来。 */
#define WHEEL_RADIUS_UM      27500U
#define WHEEL_LEVER_UM       260000U

/* 麦轮几何符号（W1..W4 = 左前/右前/左后/右后）：
 *   偏航 wz：左轮(0,2) 与右轮(1,3) 对 yaw 的贡献相反 ⇒ {-1,+1,-1,+1}；
 *   横移 vy：对角同号（FL/RR vs FR/RL）            ⇒ {-1,+1,+1,-1}。
 * 依据：用户 2026-11-15 确认的轮位；符号沿用现有实现（未单独重新标定）。 */
#define WHEEL_YAW_DIR_LIST     -1, 1, -1, 1
#define WHEEL_LATERAL_DIR_LIST -1, 1, 1, -1

extern const int8_t wheel_forward_dir[WHEEL_COUNT];
extern const int8_t wheel_encoder_dir[WHEEL_COUNT];
extern const int8_t wheel_yaw_dir[WHEEL_COUNT];
extern const int8_t wheel_lateral_dir[WHEEL_COUNT];
/* 输出轴每圈编码器计数。 */
extern const uint32_t wheel_cpr[WHEEL_COUNT];

#endif /* WHEEL_CONFIG_H */
