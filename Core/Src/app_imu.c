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
#include "iwdg.h"
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

/* [AI]  R11: MPU6050 寄存器（无 bank） */
#define MPU_WHOAMI_REG   0x75U
#define MPU_WHOAMI_VAL   0x68U
#define MPU_PWR_MGMT_1   0x6BU
#define MPU_PWR_MGMT_2   0x6CU
#define MPU_SMPLRT_DIV   0x19U   /* 1kHz/(1+7)=125Hz 输出 */
#define MPU_CONFIG       0x1AU   /* DLPF_CFG=3: 44Hz 带宽 */
#define MPU_GYRO_CONFIG  0x1BU
#define MPU_GYRO_ZOUT_H  0x47U

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

/* [AI] R14: I2C 总线解锁——EMI 下从机可能把 SDA 拉死,普通重试永远失败;
   SCL 打钟≤9 次直到 SDA 被释放,再补一个 STOP 让总线回到空闲态 */
static void icm_bus_recover(void)
{
    for (int i = 0; i < 9 && icm_sda_r() == 0; ++i) {
        icm_scl(0); icm_dly();
        icm_scl(1); icm_dly();
    }
    icm_i2c_stop();
}

static int icm_write(uint8_t reg, uint8_t val)
{
    if (icm_sda_r() == 0) icm_bus_recover();   /* [AI] R14: SDA 被拉死先解锁 */
    icm_i2c_start();
    if (icm_i2c_wr((uint8_t)(imu_addr << 1))) { icm_i2c_stop(); return 1; }
    icm_i2c_wr(reg); icm_i2c_wr(val); icm_i2c_stop();
    return 0;
}

static int icm_read_n(uint8_t reg, uint8_t *buf, int n)   /* 0=ok; 电机 EMI 下靠重试 */
{
    for (int tries = 0; tries < 3; ++tries) {
        if (icm_sda_r() == 0) icm_bus_recover();           /* [AI] R14: SDA 被拉死先解锁 */
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
    last_successful_sample_ms = 0U;
    successful_sample_valid = 0U;

    /* [AI] R20 CubeMX 迁移：PB10/PB11 已由 MX_GPIO_Init 配置为开漏输出
       （gpio.c, OUTPUT_OD）；总线空闲电平靠外部 10k 上拉（板上有,见文件头），
       不再内置 GPIO 上拉——与 CubeMX 配置单点一致。 */
    icm_scl(1); icm_sda(1);
    icm_dly();
    imu_dbg_lines = (uint8_t)((((GPIOB->IDR >> ICM_SCL_PIN) & 1U) << 1) |
                              ((GPIOB->IDR >> ICM_SDA_PIN) & 1U));   /* 诊断: 静息电平 */

    /* [AI]  R11 双芯片探测（带重试：上电就绪有竞争，单次探测曾误判 0xFF——课设实测教训）
       同地址 0x68/0x69 上先认 ICM20948(0x00=0xEA) 再认 MPU6050(0x75=0x68)。 */
    icm_delay_ms(50);
    imu_addr = 0U;
    chip = CHIP_NONE;
    for (int t = 0; t < 25; ++t) {
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
        icm_bank(0);
        icm_write(ICM_PWR_MGMT_1, 0x80U); icm_delay_ms(100);  /* DEVICE_RESET */
        icm_bank(0);
        icm_write(ICM_PWR_MGMT_1, 0x01U);                     /* 唤醒, 自动时钟源 */
        icm_write(ICM_PWR_MGMT_2, 0x00U);                     /* 陀螺+加计全开 */
        icm_bank(2);
        icm_write(ICM_GYRO_SMPLRT_DIV, 0x07U);
        icm_write(ICM_GYRO_CONFIG_1, ICM_GYRO_CONFIG1_VAL);
        icm_bank(0);
    } else {
        zout_reg = MPU_GYRO_ZOUT_H;
        icm_write(MPU_PWR_MGMT_1, 0x80U); icm_delay_ms(100);  /* DEVICE_RESET */
        icm_write(MPU_PWR_MGMT_1, 0x01U);                     /* CLKSEL=1: 陀螺 PLL */
        icm_write(MPU_PWR_MGMT_2, 0x00U);                     /* 陀螺+加计全开 */
        icm_write(MPU_SMPLRT_DIV, 0x07U);
        icm_write(MPU_CONFIG, 0x03U);                         /* DLPF 44Hz */
        icm_write(MPU_GYRO_CONFIG, 0x08U);                    /* ±500dps */
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

    uint8_t b[2];
    if (icm_read_n(zout_reg, b, 2) != 0) {
        imu_dbg_fail_cnt++;
        imu_dbg_state = (uint8_t)(1U | 2U);      /* 在线但最近读失败 */
        return;
    }
    imu_dbg_state = 1U;
    last_successful_sample_ms = now_ms;
    successful_sample_valid = 1U;
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
