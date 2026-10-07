#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>

/* [AI]  板载陀螺仪（PB10=SCL / PB11=SDA 软 I2C，PB12=INT 不用）。
   R11: 团队两块 C30D 的 IMU 型号不同——ICM-20948(0x00=0xEA) 与 MPU6050(0x75=0x68)
   双芯片自动探测，刷哪块板都能用。未找到芯片时静默降级。
   用途：底盘自稳定——只取偏航角速度(°/s)与积分航向(°)，供 100Hz 监督器
   做"速率跟踪 + 航向保持 + 打滑降额"（见 app_car.c stabilizer_*）。
   架构约束：协议帧不加 yaw 字段、航向权威源在 Jetson——本驱动只服务本地稳定环。 */

void  imu_init(void);                         /* [AI]  调度器前调用；未找到芯片时静默降级 */
void  imu_update(uint32_t now_ms, int still); /* [AI]  100Hz；still=轮子确实静止(允许零偏重置) */
int   imu_ok(void);                           /* [AI]  1=芯片在线 */
int   imu_sample_fresh(uint32_t now_ms);      /* [AI]  最近成功样本年龄不超过 20 ms */
float imu_yaw_deg(void);                      /* [AI]  积分航向(°)；imu_zero_yaw 后的相对值 */
float imu_yaw_rate_dps(void);                 /* [AI]  偏航角速度(°/s)，已扣零偏 */
void  imu_zero_yaw(void);                     /* [AI]  当前航向置 0（进入驱动模式时调用） */

/* [AI]  诊断量（非 static，SWD 直读） */
extern volatile float    imu_dbg_bias_lsb;    /* 当前零偏(raw LSB) */
extern volatile float    imu_dbg_yaw_deg;     /* 航向(°) */
extern volatile float    imu_dbg_rate_dps;    /* 角速度(°/s) */
extern volatile uint8_t  imu_dbg_state;       /* bit0=芯片在线 bit1=最近一次读失败 */
extern volatile uint32_t imu_dbg_fail_cnt;    /* 累计读失败(EMI) */
extern volatile uint8_t  imu_dbg_chip;        /* 0=无 1=ICM20948 2=MPU6050 */
extern volatile uint8_t  imu_dbg_who;         /* 0x68 处 0x00 寄存器实读值 */
extern volatile uint8_t  imu_dbg_lines;       /* 静息电平 (SCL<<1)|SDA */
extern volatile uint8_t  imu_dbg_who75;       /* 0x68 处 0x75 寄存器实读值 */

#endif /* APP_IMU_H */
