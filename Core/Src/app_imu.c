/**
 * [AI]  @file  app_imu.c — 板载陀螺仪驱动（PB10=SCL / PB11=SDA 软 I2C）
 *
 * R11: 团队两块 C30D 的 IMU 型号不同——A 板实测 MPU6050（0x75=0x68,0x00=0x81），
 * 另一块是 ICM-20948（0x00=0xEA）。驱动做双芯片自动探测，刷哪块板都能用。
 * PB12=INT1（不用，轮询）；经 1.8V 电平转换 + 10k 外部上拉；地址 0x68/0x69。
 * 软 I2C 在电机运行时受 EMI 干扰——所有读带 NACK 重试。
 * 只做陀螺仪 Z 轴：偏航角速度(°/s) + 积分航向(°)，供本地自稳定环使用。
 * 参考：car2/tools/references/smart-transport-robot/01_.../bsp/bsp_c30d.c（已验证套路）
 */
#include "app_imu.h"
#include "app_wheel_control.h"
#include "i2c.h"
#include "iwdg.h"
#include "stm32f4xx.h"
#include <math.h>

#define ICM_SCL_PIN      10U
#define ICM_SDA_PIN      11U
#define ICM_I2C_DLY      120U        /* 半周期忙等（~100kHz 量级） */
#define ICM_WHOAMI_VAL   0xEAU
#define ICM_WHO_AM_I     0x00U
#define ICM_PWR_MGMT_1   0x06U
#define ICM_PWR_MGMT_2   0x07U
#define ICM_GYRO_ZOUT_H  0x37U
#define ICM_REG_BANK_SEL 0x7FU
#define ICM_GYRO_SMPLRT_DIV  0x00U   /* bank2: ODR = 1125/(1+7) ≈ 140Hz */
#define ICM_GYRO_CONFIG_1    0x01U   /* bank2 */
#define ICM_GYRO_CONFIG1_VAL 0x33U   /* DLPF 开 + 量程 ±500dps */
#define ICM_ACCEL_CONFIG     0x14U   /* [AI] R65 bank2: 加速度量程/滤波 */
#define ICM_ACCEL_CONFIG_VAL 0x13U   /* ±8g + DLPF */
#define ICM_ACCEL_XOUT_H     0x2DU   /* [AI] R65 bank0: accel6+temp2+gyro6 块读起点 */

/* [AI]  R11: MPU6050 寄存器（无 bank） */
#define MPU_WHOAMI_REG   0x75U
#define MPU_WHOAMI_VAL   0x68U
#define MPU_PWR_MGMT_1   0x6BU
#define MPU_PWR_MGMT_2   0x6CU
#define MPU_SMPLRT_DIV   0x19U   /* 1kHz/(1+7)=125Hz 输出 */
#define MPU_CONFIG       0x1AU   /* DLPF_CFG=3: 44Hz 带宽 */
#define MPU_GYRO_CONFIG  0x1BU
#define MPU_GYRO_ZOUT_H  0x47U
#define MPU_ACCEL_CONFIG 0x1CU   /* [AI] R65 */
#define MPU_ACCEL_CFG_8G 0x10U   /* ±8 g */
#define MPU_ACCEL_XOUT_H 0x3BU   /* [AI] R65: accel6+temp2+gyro6 块读起点 */

/* ---- [AI] R65：加速度/冲击/倾角（新增；陀螺 Z 路径不变） ---- */
#define IMU_LSB_PER_G      4096.0f   /* ±8 g 量程 */
#define IMU_ACCEL_ALPHA    0.0625f   /* 100 Hz 下约 2.5 Hz 拐点 */
#define IMU_IMPACT_DELTA_MG 1500.0f  /* 相对低通基线的瞬时偏差阈值 */
#define IMU_IMPACT_HOLD_MS  200U     /* 冲击置位保持时长（供 100 Hz 帧采到） */
#define IMU_SAT_MG          7000.0f

#define ICM_LSB_PER_DPS  65.5f       /* ±500dps 额定 65.5（课设实测 65.2，可调） */
#define ICM_Z_SIGN       (+1.0f)     /* Z 正方向=逆时针(ccw)；装反改 -1（手转板验证） */
#define IMU_STILL_LSB    150.0f      /* 静止窗口范围阈值(raw LSB)；一次 ~90° 转弯约 2250 LSB。
                                        [AI] R47 标定方法：看 imu_dbg_still_range——
                                        真静止时该值应稳定在 20~60 LSB（12 位噪声底）；
                                        若 >100 说明 EMI/振动把窗口撑大，阈值取
                                        "静止典型值 ×3" 重新实测，勿拍脑袋 */
#define IMU_STILL_MS     400U        /* 静止持续该时长后提交新零偏 */
/* [AI] R47: 角速度一级 IIR（EMA α=1/4）。芯片 DLPF(44Hz) 之外的软件补充，
   主要压编码器侧对比用的 100 Hz 采样噪声；稳态增益=1 不影响标定换算。
   改大分母=更平滑但稳定器相位滞后增大，与轮级 D 项联动调。 */
#define IMU_RATE_EMA_SHIFT 2         /* α = 1/2^2 = 1/4 */

enum { CHIP_NONE = 0, CHIP_ICM = 1, CHIP_MPU = 2 };

static uint8_t  chip;                /* CHIP_xxx */
static uint8_t  zout_reg;            /* 陀螺 Z 高字节寄存器（按芯片） */
static uint8_t  acc_reg;             /* [AI] R65: 14 字节块读起点（按芯片） */
static uint8_t  imu_addr;            /* 0=未找到 */
static float    gyro_bias;
static float    yaw_now;
static float    rate_dps_now;
static uint32_t yaw_last_ms;
static uint32_t last_successful_sample_ms;
static uint8_t  successful_sample_valid;
static uint32_t still_t0, still_n;
static float    still_acc, still_min, still_max;

volatile float    imu_dbg_bias_lsb;
volatile float    imu_dbg_yaw_deg;
volatile float    imu_dbg_rate_dps;
volatile float    imu_dbg_still_range;   /* [AI] R47: 最近提交零偏窗口的 raw 峰谷范围——IMU_STILL_LSB 实测标定依据 */
volatile uint8_t  imu_dbg_state;
volatile uint32_t imu_dbg_fail_cnt;
volatile uint8_t  imu_dbg_chip;     /* 诊断: 0=无 1=ICM20948 2=MPU6050 */
volatile uint8_t  imu_dbg_who;      /* 诊断: 0x68 处 WHO_AM_I 实读值 */
volatile uint8_t  imu_dbg_lines;    /* 诊断: 静息 (SCL<<1)|SDA, 0b11=两线均被上拉 */
volatile uint8_t  imu_dbg_who75;    /* 诊断: 0x68 处寄存器 0x75(MPU系 WHO_AM_I) */

/* [AI] R65：6 轴扩展状态（陀螺 Z 以外都是新增，不影响原稳定器） */
static volatile float   s_acc_mg[3];      /* 低通后加速度 mg */
static volatile float   s_pitch_deg;
static volatile float   s_roll_deg;
static volatile uint8_t s_evt;
static volatile uint8_t s_impact_new;
static volatile uint32_t s_impact_until_ms;
static volatile int32_t s_yaw_delta_mrad;
static uint8_t  s_acc_lp_primed;

/* [AI]  DWT 毫秒延时（boot 路径禁 HAL tick——R3 教训；本驱动自含, 不与 app_car 耦合）
   [AI] R20: 顺带喂 IWDG——HAL v1.28.3 的 HAL_IWDG_Init 在 MX_IWDG_Init 里就
   启动了 2s 看门狗,而双芯片探测(各带重试)最坏 ~7s,不喂会在 boot 中途被咬 */
static void icm_delay_ms(uint32_t ms)
{
    (void)HAL_IWDG_Refresh(&hiwdg);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT;
    uint32_t span = ms * (SystemCoreClock / 1000U);
    while ((uint32_t)(DWT->CYCCNT - start) < span) { __NOP(); }
}

/* ---- [AI] R66：硬件 I2C2（CubeMX 生成：PB10=SCL / PB11=SDA，400 kHz）----
 * 软 I2C 已废弃：CubeMX 把这两脚配成 AF4 后，BSRR 位操作对它们无效（写 ODR 不再影响引脚）。
 * 为何用轮询而不用 DMA：CubeMX 的 -q 脚本**设不了 NVIC**（CUBEMX-SOP §4 实测），
 * 没有 I2C2_EV/ER 中断时 HAL 的 DMA 路径无法收尾；轮询 14 字节约 0.36 ms @400 kHz，
 * 在 100/200 Hz 采样下占用 <10%。DMA 等到 GUI 里勾上两个 I2C2 中断再切。 */
static int icm_write(uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(imu_addr << 1), reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1U, 20U) == HAL_OK) ? 0 : 1;
}

static int icm_read_n(uint8_t reg, uint8_t *buf, int n)
{
    for (int tries = 0; tries < 2; ++tries) {
        /* [AI] R68 致命 bug 修复：HAL 阻塞重试期间不喂狗 → 探测期 25×2×3×20ms ≈3 s
           > IWDG 2 s → 复位循环（OLED 全黑、CAN 全哑）。三道防线：喂狗 / 5 ms 超时 / 2 次重试。 */
        (void)HAL_IWDG_Refresh(&hiwdg);
        if (HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(imu_addr << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)n, 5U) == HAL_OK) {
            return 0;
        }
    }
    return 1;
}

static uint8_t icm_read(uint8_t reg) { uint8_t v = 0; (void)icm_read_n(reg, &v, 1); return v; }
static void    icm_bank(uint8_t b)   { icm_write(ICM_REG_BANK_SEL, (uint8_t)(b << 4)); }

/* [AI] R65: 芯片输出是大端（高字节在前） */
static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

void imu_init(void)
{
    last_successful_sample_ms = 0U;
    successful_sample_valid = 0U;

    /* [AI] R20 CubeMX 迁移：PB10/PB11 已由 MX_GPIO_Init 配置为开漏输出
       （gpio.c, OUTPUT_OD）；总线空闲电平靠外部 10k 上拉（板上有,见文件头），
       不再内置 GPIO 上拉——与 CubeMX 配置单点一致。 */
    /* [AI] R66：PB10/PB11 现在是 CubeMX 配的 I2C2 AF4 引脚（400 kHz） */
    imu_dbg_lines = 0xFFU;   /* 旧软 I2C 的静息电平诊断在此配置下无意义 */

    /* [AI]  R11 双芯片探测（带重试：上电就绪有竞争，单次探测曾误判 0xFF——课设实测教训）
       同地址 0x68/0x69 上先认 ICM20948(0x00=0xEA) 再认 MPU6050(0x75=0x68)。 */
    icm_delay_ms(50);
    imu_addr = 0U;
    chip = CHIP_NONE;
    for (int t = 0; t < 6; ++t) {      /* [AI] R68: 25 轮 → 6 轮，总预算 ≤1 s */
        imu_addr = 0x68U;
        imu_dbg_who = icm_read(ICM_WHO_AM_I);      /* 诊断: ICM 系 WHO_AM_I(0x00) */
        imu_dbg_who75 = icm_read(MPU_WHOAMI_REG);  /* 诊断: MPU 系 WHO_AM_I(0x75) */
        if (imu_dbg_who == ICM_WHOAMI_VAL) { chip = CHIP_ICM; break; }
        if (imu_dbg_who75 == MPU_WHOAMI_VAL) { chip = CHIP_MPU; break; }
        imu_addr = 0x69U;
        if (icm_read(ICM_WHO_AM_I) == ICM_WHOAMI_VAL) { imu_dbg_who = ICM_WHOAMI_VAL; chip = CHIP_ICM; break; }
        if (icm_read(MPU_WHOAMI_REG) == MPU_WHOAMI_VAL) { imu_dbg_who75 = MPU_WHOAMI_VAL; chip = CHIP_MPU; break; }
        imu_addr = 0U;
        icm_delay_ms(20);
    }
    imu_dbg_chip = chip;
    if (chip == CHIP_NONE) { imu_dbg_state = 0U; return; }

    if (chip == CHIP_ICM) {
        zout_reg = ICM_GYRO_ZOUT_H;
        acc_reg  = ICM_ACCEL_XOUT_H;          /* [AI] R65 */
        icm_bank(0);
        icm_write(ICM_PWR_MGMT_1, 0x80U); icm_delay_ms(100);  /* DEVICE_RESET */
        icm_bank(0);
        icm_write(ICM_PWR_MGMT_1, 0x01U);                     /* 唤醒, 自动时钟源 */
        icm_write(ICM_PWR_MGMT_2, 0x00U);                     /* 陀螺+加计全开 */
        icm_bank(2);
        icm_write(ICM_GYRO_SMPLRT_DIV, 0x07U);
        icm_write(ICM_GYRO_CONFIG_1, ICM_GYRO_CONFIG1_VAL);
        icm_write(ICM_ACCEL_CONFIG, ICM_ACCEL_CONFIG_VAL);   /* [AI] R65: ±8 g */
        icm_bank(0);
    } else {
        zout_reg = MPU_GYRO_ZOUT_H;
        acc_reg  = MPU_ACCEL_XOUT_H;          /* [AI] R65 */
        icm_write(MPU_PWR_MGMT_1, 0x80U); icm_delay_ms(100);  /* DEVICE_RESET */
        icm_write(MPU_PWR_MGMT_1, 0x01U);                     /* CLKSEL=1: 陀螺 PLL */
        icm_write(MPU_PWR_MGMT_2, 0x00U);                     /* 陀螺+加计全开 */
        icm_write(MPU_SMPLRT_DIV, 0x07U);
        icm_write(MPU_CONFIG, 0x03U);                         /* DLPF 44Hz */
        icm_write(MPU_GYRO_CONFIG, 0x08U);                    /* ±500dps */
        icm_write(MPU_ACCEL_CONFIG, MPU_ACCEL_CFG_8G);        /* [AI] R65: ±8 g */
    }
    icm_delay_ms(50);

    /* 启动零偏：只在"读数范围足够平"的窗口上提交（上电抖动/电机起动会被范围检查拒掉） */
    {
        float acc = 0.0f;
        for (int t = 0; t < 15; ++t) {
            uint8_t b[2];
            float mn = 32767.0f, mx = -32768.0f;
            acc = 0.0f;
            for (int i = 0; i < 200; ++i) {
                (void)icm_read_n(zout_reg, b, 2);
                float z = (float)(int16_t)(((uint16_t)b[0] << 8) | b[1]);
                if (z < mn) mn = z;
                if (z > mx) mx = z;
                acc += z;
                icm_delay_ms(2);
            }
            if (mx - mn < IMU_STILL_LSB) break;
        }
        gyro_bias = acc / 200.0f;
    }

    yaw_now = 0.0f;
    yaw_last_ms = 0U;
    still_t0 = 0U; still_n = 0U; still_acc = 0.0f;
    still_min = still_max = 0.0f;
    imu_dbg_bias_lsb = gyro_bias;
    imu_dbg_state = 1U;
}

void imu_update(uint32_t now_ms, int still)
{
    if (chip == CHIP_NONE) return;

    /* [AI] R65: 一次 14 字节块读 = accel6 + temp2 + gyro6（陀螺 Z 在 offset 12）。
       单事务比“加速度单独再读一次”省一个 START/STOP，且两轴共用同一个采样时刻。 */
    uint8_t b[14];
    if (icm_read_n(acc_reg, b, 14) != 0) {
        imu_dbg_fail_cnt++;
        imu_dbg_state = (uint8_t)(1U | 2U);      /* 在线但最近读失败 */
        return;
    }
    imu_dbg_state = 1U;
    last_successful_sample_ms = now_ms;
    successful_sample_valid = 1U;
    float raw = (float)be16(&b[12]);          /* 陀螺 Z（与旧 zout_reg 同一寄存器） */

    /* ---- [AI] R65: 加速度 → 低通基线 / 倾角 / 冲击 / 事件 ---- */
    {
        float ax = (float)be16(&b[0]) * (1000.0f / IMU_LSB_PER_G);
        float ay = (float)be16(&b[2]) * (1000.0f / IMU_LSB_PER_G);
        float az = (float)be16(&b[4]) * (1000.0f / IMU_LSB_PER_G);

        if (s_acc_lp_primed == 0U) {          /* 首样本入基线，防开机假冲击 */
            s_acc_mg[0] = ax; s_acc_mg[1] = ay; s_acc_mg[2] = az;
            s_acc_lp_primed = 1U;
        }

        float dx = ax - s_acc_mg[0];
        float dy = ay - s_acc_mg[1];
        float dz = az - s_acc_mg[2];
        float d2 = dx * dx + dy * dy + dz * dz;
        if (d2 > (IMU_IMPACT_DELTA_MG * IMU_IMPACT_DELTA_MG)) {
            s_impact_until_ms = now_ms + IMU_IMPACT_HOLD_MS;
            s_impact_new = 1U;
        }

        s_acc_mg[0] += (ax - s_acc_mg[0]) * IMU_ACCEL_ALPHA;
        s_acc_mg[1] += (ay - s_acc_mg[1]) * IMU_ACCEL_ALPHA;
        s_acc_mg[2] += (az - s_acc_mg[2]) * IMU_ACCEL_ALPHA;

        /* [AI] R76: 倾角按已知竖装（+Y 朝上）重推（rak/docs/canopen-bitfields.md §7）。
           旧实现假设芯片 Z 朝上的小角近似（-ay/az、ax/az）：模块竖装时重力沿
           芯片 Y 轴、az≈0，比值爆炸 → 2026-10-30 台架实测 pitch/roll ≈ ±163°
           且随轮振漂移。现把加速度旋到水平系（x_h=x_c, y_h=z_c, z_h=y_c）后
           用 atan2 解算：车体水平时 a_yc≈+1g、a_zc≈0 → pitch/roll ≈ 0。
           影响面 = 遥测/OLED（imu_tilt_deg 消费者），不碰稳定器闭环；符号约定
           待现场核对——修好确认前 Jetson 不消费 0x2031/0x2032（EKF 航向权威
           = Jetson LSM6DSR，本就不依赖）。 */
        /* [AI] R78 2026-11-17：R76 的"竖装 +Y 朝上"假设**与实物不符**。SWD 直读
           `s_acc_mg = [35.0, 3.6, 955.9] mg`（车静止、地面水平）——重力沿**芯片 +Z**，
           即模块是**平装**（+Z 朝上、X 前置、Y 左）。按竖装解算当然得 ±90°：
           实测 pitch=-89.79°、roll=+84.18°，Note 消费者只能当噪声。
           平装正解（REP-103：x 前 y 左 z 上；加速度计测的是比力，静止时沿"上"为 +1g）：
             抬头(绕 +y 正转) → a_x = -g·sin(pitch)  → pitch = atan2(-a_x, a_z)
             右侧下沉(绕 +x 正转) → a_y = -g·sin(roll) → roll = atan2(-a_y, a_z)
           符号约定仍需现场核对（把车头抬起 10° 应读 +10°），确认前 Jetson 不消费
           0x2031/0x2032；稳定器用的是陀螺 yaw rate，与本函数无关。 */
        if (s_acc_mg[2] > 1.0f || s_acc_mg[2] < -1.0f) {
            s_pitch_deg = -atan2f(s_acc_mg[0], s_acc_mg[2]) * 57.29578f;
            s_roll_deg  = -atan2f(s_acc_mg[1], s_acc_mg[2]) * 57.29578f;
        }

        uint8_t evt = 0U;
        if ((int32_t)(now_ms - s_impact_until_ms) < 0) evt |= IMU_EVT_IMPACT;
        if (d2 > (IMU_SAT_MG * IMU_SAT_MG)) evt |= IMU_EVT_SATURATED;
        s_evt = evt;
    }

    /* 静止零偏重置：按读数"范围"判静止（对陈旧零偏鲁棒——课设实测教训：
       用 |raw-bias| 判静止会锁死）。减速滑行范围大 → 不提交，真停稳才提交。 */
    if (still) {
        if (still_t0 == 0U) {
            still_t0 = now_ms ? now_ms : 1U;
            still_acc = 0.0f; still_n = 0U;
            still_min = raw; still_max = raw;
        }
        if (raw < still_min) still_min = raw;
        if (raw > still_max) still_max = raw;
        if (still_max - still_min > IMU_STILL_LSB) {
            still_t0 = now_ms ? now_ms : 1U;
            still_acc = raw; still_n = 1U;
            still_min = raw; still_max = raw;
        } else {
            still_acc += raw; still_n++;
            if ((uint32_t)(now_ms - still_t0) >= IMU_STILL_MS) {
                gyro_bias = still_acc / (float)still_n;      /* 平窗 → 用真静止重定零偏 */
                imu_dbg_still_range = still_max - still_min; /* [AI] R47: 静止噪声底实测值 */
                still_t0 = now_ms ? now_ms : 1U;
                still_acc = 0.0f; still_n = 0U;
                still_min = raw; still_max = raw;
            }
        }
    } else {
        still_t0 = 0U;                            /* 运动中：关闭零偏重置 */
    }

    float dt = (float)(uint32_t)(now_ms - yaw_last_ms) * 0.001f;
    yaw_last_ms = now_ms;
    if (dt > 0.1f) dt = 0.1f;                     /* 轮询间隙上限，防积分跳变 */
    /* [AI] R47: 一级 IIR α=1/4——先滤波再积分/输出，稳定器与航向共用 */
    rate_dps_now += ((raw - gyro_bias) / ICM_LSB_PER_DPS - rate_dps_now) *
                    (1.0f / (float)(1U << IMU_RATE_EMA_SHIFT));
    yaw_now += ICM_Z_SIGN * rate_dps_now * dt;

    /* [AI] R65: 帧间偏航增量（mrad），供 CAN 0x032 上报；读一次清一次 */
    {
        int32_t d = (int32_t)(ICM_Z_SIGN * rate_dps_now * dt * 17.453293f);
        int32_t acc = s_yaw_delta_mrad + d;
        if (acc > 32000L)  acc = 32000L;      /* 对应 0x032.DYAW 的 i16 上限 */
        if (acc < -32000L) acc = -32000L;
        s_yaw_delta_mrad = acc;
    }

    imu_dbg_bias_lsb = gyro_bias;
    imu_dbg_yaw_deg  = yaw_now;
    imu_dbg_rate_dps = rate_dps_now;
}

int   imu_ok(void)           { return chip != CHIP_NONE; }
int   imu_sample_fresh(uint32_t now_ms)
{
    return chip != CHIP_NONE &&
        wheel_control_sample_fresh(now_ms, last_successful_sample_ms,
                                   successful_sample_valid);
}
float imu_yaw_deg(void)      { return yaw_now; }
float imu_yaw_rate_dps(void) { return rate_dps_now; }
void  imu_zero_yaw(void)     { yaw_now = 0.0f; }

/* ================= [AI] R65：6 轴扩展的对外接口 ================= */

void imu_accel_mg(float a[3])
{
    a[0] = s_acc_mg[0]; a[1] = s_acc_mg[1]; a[2] = s_acc_mg[2];
}

void imu_tilt_deg(float *pitch, float *roll)
{
    if (pitch != NULL) *pitch = s_pitch_deg;
    if (roll != NULL)  *roll  = s_roll_deg;
}

uint8_t imu_events(void)
{
    uint8_t e = s_evt;
    if (chip != CHIP_NONE &&
        wheel_control_sample_fresh(HAL_GetTick(), last_successful_sample_ms,
                                   successful_sample_valid) == 0) {
        e = (uint8_t)(e | IMU_EVT_STALE);
    }
    return e;
}

int16_t imu_take_yaw_delta_mrad(void)
{
    int32_t d = s_yaw_delta_mrad;
    s_yaw_delta_mrad = 0;
    if (d > 32000L)  d = 32000L;
    if (d < -32000L) d = -32000L;
    return (int16_t)d;
}

uint8_t imu_take_impact(void)
{
    uint8_t v = s_impact_new;
    s_impact_new = 0U;
    return v;
}

uint8_t imu_age_ms(void)
{
    uint32_t age = (uint32_t)(HAL_GetTick() - last_successful_sample_ms);
    return (uint8_t)((age > 255U) ? 255U : age);
}
