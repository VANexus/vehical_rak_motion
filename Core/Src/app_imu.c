/**
 * [AI]  @file  app_imu.c — 板载 ICM-20948 陀螺仪驱动（PB10=SCL / PB11=SDA 软 I2C）
 *
 * 硬件事实（同板课设对照原理图/资源 PDF 实测确认）：ICM-20948 挂在 PB10/PB11，
 * PB12=INT1（不用，轮询）；经 1.8V 电平转换 + 10k 外部上拉；地址 0x68/0x69，
 * WHO_AM_I==0xEA。软 I2C 在电机运行时受 EMI 干扰——所有读带 NACK 重试。
 * 只做陀螺仪 Z 轴：偏航角速度(°/s) + 积分航向(°)，供本地自稳定环使用。
 * 参考：car2/tools/references/smart-transport-robot/01_.../bsp/bsp_c30d.c（已验证套路）
 */
#include "app_imu.h"
#include "stm32f4xx.h"

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
#define ICM_LSB_PER_DPS  65.5f       /* ±500dps 额定 65.5（课设实测 65.2，可调） */
#define ICM_Z_SIGN       (+1.0f)     /* Z 正方向=逆时针(ccw)；装反改 -1（手转板验证） */
#define IMU_STILL_LSB    150.0f      /* 静止窗口范围阈值(raw LSB)；一次 ~90° 转弯约 2250 LSB */
#define IMU_STILL_MS     400U        /* 静止持续该时长后提交新零偏 */

static uint8_t  imu_addr;            /* 0=未找到 */
static float    gyro_bias;
static float    yaw_now;
static float    rate_dps_now;
static uint32_t yaw_last_ms;
static uint32_t still_t0, still_n;
static float    still_acc, still_min, still_max;

volatile float    imu_dbg_bias_lsb;
volatile float    imu_dbg_yaw_deg;
volatile float    imu_dbg_rate_dps;
volatile uint8_t  imu_dbg_state;
volatile uint32_t imu_dbg_fail_cnt;
volatile uint8_t  imu_dbg_who;      /* 诊断: 0x68 处 WHO_AM_I 实读值 */
volatile uint8_t  imu_dbg_lines;    /* 诊断: 静息 (SCL<<1)|SDA, 0b11=两线均被上拉 */
volatile uint8_t  imu_dbg_who75;    /* 诊断: 0x68 处寄存器 0x75(MPU系 WHO_AM_I 位) */

/* [AI]  DWT 毫秒延时（boot 路径禁 HAL tick——R3 教训；本驱动自含, 不与 app_car 耦合） */
static void icm_delay_ms(uint32_t ms)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT;
    uint32_t span = ms * (SystemCoreClock / 1000U);
    while ((uint32_t)(DWT->CYCCNT - start) < span) { __NOP(); }
}

/* ---- 软 I2C（开漏 + 外部上拉；参照同板课设, 已被证明可用） ---- */
static void icm_dly(void)   { volatile uint32_t n = ICM_I2C_DLY; while (n--) { __NOP(); } }
static void icm_scl(int x)  { GPIOB->BSRR = x ? (1UL << ICM_SCL_PIN) : (1UL << (ICM_SCL_PIN + 16U)); }
static void icm_sda(int x)  { GPIOB->BSRR = x ? (1UL << ICM_SDA_PIN) : (1UL << (ICM_SDA_PIN + 16U)); }
static int  icm_sda_r(void) { return (GPIOB->IDR & (1UL << ICM_SDA_PIN)) ? 1 : 0; }

static void icm_i2c_start(void) { icm_sda(1); icm_scl(1); icm_dly(); icm_sda(0); icm_dly(); icm_scl(0); icm_dly(); }
static void icm_i2c_stop(void)  { icm_sda(0); icm_scl(1); icm_dly(); icm_sda(1); icm_dly(); }

static int icm_i2c_wr(uint8_t b)          /* 返回 ACK: 0=ack 1=nack */
{
    for (int i = 0; i < 8; ++i) {
        icm_sda((b & 0x80U) ? 1 : 0); b = (uint8_t)(b << 1);
        icm_dly(); icm_scl(1); icm_dly(); icm_scl(0); icm_dly();
    }
    icm_sda(1); icm_dly();                /* 释放等 ACK */
    icm_scl(1); icm_dly(); int ack = icm_sda_r(); icm_scl(0); icm_dly();
    return ack;
}

static uint8_t icm_i2c_rd(int ack)        /* ack=1: 主机继续 ACK */
{
    uint8_t b = 0;
    icm_sda(1);
    for (int i = 0; i < 8; ++i) {
        b = (uint8_t)(b << 1); icm_scl(1); icm_dly();
        if (icm_sda_r()) b |= 1U;
        icm_scl(0); icm_dly();
    }
    icm_sda(ack ? 0 : 1); icm_dly(); icm_scl(1); icm_dly(); icm_scl(0); icm_sda(1); icm_dly();
    return b;
}

static int icm_write(uint8_t reg, uint8_t val)
{
    icm_i2c_start();
    if (icm_i2c_wr((uint8_t)(imu_addr << 1))) { icm_i2c_stop(); return 1; }
    icm_i2c_wr(reg); icm_i2c_wr(val); icm_i2c_stop();
    return 0;
}

static int icm_read_n(uint8_t reg, uint8_t *buf, int n)   /* 0=ok; 电机 EMI 下靠重试 */
{
    for (int tries = 0; tries < 3; ++tries) {
        int nack;
        icm_i2c_start();
        nack  = icm_i2c_wr((uint8_t)(imu_addr << 1));
        nack |= icm_i2c_wr(reg);
        icm_i2c_start();
        nack |= icm_i2c_wr((uint8_t)((imu_addr << 1) | 1U));
        if (nack) { icm_i2c_stop(); continue; }
        for (int i = 0; i < n; ++i) buf[i] = icm_i2c_rd(i < n - 1);
        icm_i2c_stop();
        return 0;
    }
    return 1;
}

static uint8_t icm_read(uint8_t reg) { uint8_t v = 0; (void)icm_read_n(reg, &v, 1); return v; }
static void    icm_bank(uint8_t b)   { icm_write(ICM_REG_BANK_SEL, (uint8_t)(b << 4)); }

void imu_init(void)
{
    /* GPIO: PB10/PB11 通用输出+开漏+上拉 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;
    GPIOB->MODER = (GPIOB->MODER & ~((3UL << 20) | (3UL << 22))) | (1UL << 20) | (1UL << 22);
    GPIOB->OTYPER |= (1UL << ICM_SCL_PIN) | (1UL << ICM_SDA_PIN);
    GPIOB->PUPDR = (GPIOB->PUPDR & ~((3UL << 20) | (3UL << 22))) | (1UL << 20) | (1UL << 22);
    icm_scl(1); icm_sda(1);
    icm_dly();
    imu_dbg_lines = (uint8_t)((((GPIOB->IDR >> ICM_SCL_PIN) & 1U) << 1) |
                              ((GPIOB->IDR >> ICM_SDA_PIN) & 1U));   /* 诊断: 静息电平 */

    /* 地址探测（带重试：上电就绪有竞争，单次探测曾误判 0xFF——课设实测教训） */
    icm_delay_ms(50);
    imu_addr = 0U;
    for (int t = 0; t < 25; ++t) {
        imu_addr = 0x68U;
        imu_dbg_who = icm_read(ICM_WHO_AM_I);      /* 诊断: ICM 系 WHO_AM_I(0x00) */
        imu_dbg_who75 = icm_read(0x75U);           /* 诊断: MPU 系 WHO_AM_I(0x75) */
        if (imu_dbg_who == ICM_WHOAMI_VAL) break;
        imu_addr = 0x69U; if (icm_read(ICM_WHO_AM_I) == ICM_WHOAMI_VAL) break;
        imu_addr = 0U;
        icm_delay_ms(20);
    }
    if (imu_addr == 0U) { imu_dbg_state = 0U; return; }

    icm_bank(0);
    icm_write(ICM_PWR_MGMT_1, 0x80U); icm_delay_ms(100);  /* DEVICE_RESET */
    icm_bank(0);
    icm_write(ICM_PWR_MGMT_1, 0x01U);                     /* 唤醒, 自动时钟源 */
    icm_write(ICM_PWR_MGMT_2, 0x00U);                     /* 陀螺+加计全开 */
    icm_bank(2);
    icm_write(ICM_GYRO_SMPLRT_DIV, 0x07U);
    icm_write(ICM_GYRO_CONFIG_1, ICM_GYRO_CONFIG1_VAL);
    icm_bank(0);
    icm_delay_ms(50);

    /* 启动零偏：只在"读数范围足够平"的窗口上提交（上电抖动/电机起动会被范围检查拒掉） */
    {
        float acc = 0.0f;
        for (int t = 0; t < 15; ++t) {
            uint8_t b[2];
            float mn = 32767.0f, mx = -32768.0f;
            acc = 0.0f;
            for (int i = 0; i < 200; ++i) {
                (void)icm_read_n(ICM_GYRO_ZOUT_H, b, 2);
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
    if (imu_addr == 0U) return;

    uint8_t b[2];
    if (icm_read_n(ICM_GYRO_ZOUT_H, b, 2) != 0) {
        imu_dbg_fail_cnt++;
        imu_dbg_state = (uint8_t)(1U | 2U);      /* 在线但最近读失败 */
        yaw_last_ms = now_ms;                    /* 不积分；时间戳跟上防跳变 */
        return;
    }
    imu_dbg_state = 1U;
    float raw = (float)(int16_t)(((uint16_t)b[0] << 8) | b[1]);

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
    rate_dps_now = (raw - gyro_bias) / ICM_LSB_PER_DPS;
    yaw_now += ICM_Z_SIGN * rate_dps_now * dt;

    imu_dbg_bias_lsb = gyro_bias;
    imu_dbg_yaw_deg  = yaw_now;
    imu_dbg_rate_dps = rate_dps_now;
}

int   imu_ok(void)           { return imu_addr != 0U; }
float imu_yaw_deg(void)      { return yaw_now; }
float imu_yaw_rate_dps(void) { return rate_dps_now; }
void  imu_zero_yaw(void)     { yaw_now = 0.0f; }