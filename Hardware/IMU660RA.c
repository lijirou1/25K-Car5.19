#include "IMU660RA.h"
#include "MyI2C.h"
#include "delay.h"

/* ---- 偏航角计算参数 ---- */
#define GYRO_SENSITIVITY  33.2f
#define DT                0.02f       // 采样周期 20ms

static int16_t gyro_z_bias = 0;
static float   Yaw = 0.0f;

uint8_t imu_init_ok = 0;

/* ---- 卡尔曼滤波器参数 ---- */
static float kalman_x = 0.0f;
static float kalman_P = 0.0f;
static float kalman_Q = 0.02f;
static float kalman_R = 0.5f;
static uint8_t kalman_first = 1;

#define IMU660RA_DEV_ADDR      0x69
#define IMU660RA_CHIP_ID       0x00
#define IMU660RA_PWR_CONF      0x7C
#define IMU660RA_PWR_CTRL      0x7D
#define IMU660RA_INIT_CTRL     0x59
#define IMU660RA_INIT_DATA     0x5E
#define IMU660RA_INT_STA       0x21
#define IMU660RA_ACC_ADDRESS   0x0C
#define IMU660RA_GYRO_ADDRESS  0x12
#define IMU660RA_ACC_CONF      0x40
#define IMU660RA_ACC_RANGE     0x41
#define IMU660RA_GYR_CONF      0x42
#define IMU660RA_GYR_RANGE     0x43
#define IMU660RA_ACC_SAMPLE    0x02
#define IMU660RA_GYR_SAMPLE    0x01

extern const unsigned char imu660ra_config_file[8192];

static void IMU660RA_WriteReg(uint8_t reg, uint8_t data)
{
    MyI2C_Start();
    MyI2C_SendByte(IMU660RA_DEV_ADDR << 1);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}
static uint8_t IMU660RA_ReadReg(uint8_t reg)
{
    uint8_t data;

    MyI2C_Start();
    MyI2C_SendByte(IMU660RA_DEV_ADDR << 1);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte((IMU660RA_DEV_ADDR << 1) | 0x01);
    MyI2C_ReceiveAck();
    data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return data;
}
static void IMU660RA_ReadRegs(uint8_t reg, uint8_t *data, uint16_t count)
{
    uint16_t i;

    MyI2C_Start();
    MyI2C_SendByte(IMU660RA_DEV_ADDR << 1);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte((IMU660RA_DEV_ADDR << 1) | 0x01);
    MyI2C_ReceiveAck();
    for(i = 0; i < count; i++)
    {
        data[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck(i < count - 1 ? 0 : 1);
    }
    MyI2C_Stop();
}
static void IMU660RA_WriteRegs(uint8_t reg, const unsigned char *data, uint16_t count)
{
    uint16_t i;

    MyI2C_Start();
    MyI2C_SendByte(IMU660RA_DEV_ADDR << 1);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();
    for(i = 0; i < count; i++)
    {
        MyI2C_SendByte(data[i]);
        MyI2C_ReceiveAck();
    }
    MyI2C_Stop();
}
uint8_t IMU660RA_GetID(void)
{
    return IMU660RA_ReadReg(IMU660RA_CHIP_ID);
}
// 获取当前偏航角（度）
uint8_t IMU660RA_Init(void)
{
    uint8_t state = 0;
    uint16_t timeout = 0;

    MyI2C_Init();
    Delay_ms(20);

    while(IMU660RA_GetID() != 0x24)
    {
        Delay_ms(1);
        if(++timeout > 255)
        {
            return 1;
        }
    }

    IMU660RA_WriteReg(IMU660RA_PWR_CONF, 0x00);
    Delay_ms(10);
    IMU660RA_WriteReg(IMU660RA_INIT_CTRL, 0x00);
    IMU660RA_WriteRegs(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));
    IMU660RA_WriteReg(IMU660RA_INIT_CTRL, 0x01);
    Delay_ms(20);

    if(IMU660RA_ReadReg(IMU660RA_INT_STA) != 1)
    {
        state = 1;
    }

    IMU660RA_WriteReg(IMU660RA_PWR_CTRL, 0x0E);
    IMU660RA_WriteReg(IMU660RA_ACC_CONF, 0xA7);
    IMU660RA_WriteReg(IMU660RA_GYR_CONF, 0xA9);
    IMU660RA_WriteReg(IMU660RA_ACC_RANGE, IMU660RA_ACC_SAMPLE);
    IMU660RA_WriteReg(IMU660RA_GYR_RANGE, IMU660RA_GYR_SAMPLE);

    imu_init_ok = (state == 0) ? 1 : 0;
    return state;
}
// 获取原始加速度和陀螺仪数据
void IMU660RA_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                      int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t acc[6];
    uint8_t gyro[6];

    IMU660RA_ReadRegs(IMU660RA_ACC_ADDRESS, acc, 6);
    IMU660RA_ReadRegs(IMU660RA_GYRO_ADDRESS, gyro, 6);

    *AccX = (int16_t)((uint16_t)acc[1] << 8 | acc[0]);
    *AccY = (int16_t)((uint16_t)acc[3] << 8 | acc[2]);
    *AccZ = (int16_t)((uint16_t)acc[5] << 8 | acc[4]);

    *GyroX = (int16_t)((uint16_t)gyro[1] << 8 | gyro[0]);
    *GyroY = (int16_t)((uint16_t)gyro[3] << 8 | gyro[2]);
    *GyroZ = (int16_t)((uint16_t)gyro[5] << 8 | gyro[4]);
}
/* ========== 卡尔曼滤波器 ========== 
 * @brief  初始化卡尔曼滤波器
 * @param  Q  过程噪声协方差，越大响应越快，越小越平滑
 * @param  R  测量噪声协方差，越大越平滑，越小响应越快
 */
void IMU660RA_Kalman_Init(float Q, float R)
{
    kalman_Q = Q;
    kalman_R = R;
    kalman_x = 0.0f;
    kalman_P = 1.0f;
    kalman_first = 1;
}

/**
 * @brief  自适应卡尔曼滤波更新
 * @param  raw_dps  原始角速度 (dps)
 * @retval 滤波后角速度
 *
 * 自适应：根据 |角速度| 动态调整 Q
 *   <5°/s→0.02(平滑)  5~30°/s→0.1(平衡)  ≥30°/s→0.5(快速跟踪)
 */
static void Kalman_SetAdaptiveQ(float raw_dps)
{
    float abs_dps = raw_dps;
    if(abs_dps < 0.0f) abs_dps = -abs_dps;

    if(abs_dps < 5.0f)
        kalman_Q = 0.02f;
    else if(abs_dps < 30.0f)
        kalman_Q = 0.1f;
    else
        kalman_Q = 0.5f;
}

static float Kalman_Update(float measurement)
{
    float K;

    if(kalman_first)
    {
        kalman_x = measurement;
        kalman_first = 0;
        return kalman_x;
    }

    kalman_P += kalman_Q;
    K = kalman_P / (kalman_P + kalman_R);
    kalman_x += K * (measurement - kalman_x);
    kalman_P *= (1.0f - K);

    return kalman_x;
}
/* ========== 陀螺仪偏航角处理 ========== */
void IMU660RA_CalibrateGyroZ(void)
{
    int i;
    int32_t sum_gz = 0;
    int16_t GX, GY, GZ;
    int16_t AX, AY, AZ;

    for(i = 0; i < 100; i++)
    {
        IMU660RA_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        sum_gz += GZ;
    }
    gyro_z_bias = (int16_t)(sum_gz / 100);

    IMU660RA_Kalman_Init(0.02f, 0.5f);
}

/* ---- ZUPT零速修正参数 ---- */
#define ZUPT_THRESHOLD_DPS    0.6f     // 静止判定阈值(°/s)，过滤电机振动
#define ZUPT_COUNT_THRESHOLD  5        // 连续判定次数(5×20ms=100ms)
static uint8_t zupt_count = 0;
static float  bias_correction = 0.0f;  // 零偏修正累积量
static uint8_t was_stationary = 0;     // 上一周期是否静止

/* ---- 急转弯冷却参数 ---- */
#define TURN_HIGH_SPEED_THRESHOLD  30.0f    // 急转弯判定阈值(°/s)
#define ZUPT_COOLDOWN_CYCLES      25        // 冷却周期(500ms内禁止ZUPT冻结)
static uint8_t turn_cooldown = 0;           // 冷却计数器(>0时禁用ZUPT)

/**
 * @brief  更新偏航角（卡尔曼滤波 + 急转弯冷却 + ZUPT零速修正）
 * @param  GZ  陀螺仪Z轴原始值
 *
 * 流程：减零偏 → 卡尔曼滤波 → 急转弯检测（冷却期内不冻结）→ ZUPT判定
 * ZUPT：连续低速判定静止后冻结角度，并用滤波值修正零偏漂移
 */
void IMU660RA_UpdateYaw_Filtered(int16_t GZ)
{
    float raw_dps;
    float filtered_dps;
    float abs_raw_dps;

    raw_dps = (float)(GZ - gyro_z_bias) / GYRO_SENSITIVITY;

    Kalman_SetAdaptiveQ(raw_dps);
    filtered_dps = Kalman_Update(raw_dps);

    abs_raw_dps = raw_dps;
    if(abs_raw_dps < 0.0f) abs_raw_dps = -abs_raw_dps;

    if(abs_raw_dps >= TURN_HIGH_SPEED_THRESHOLD)
    {
        turn_cooldown = ZUPT_COOLDOWN_CYCLES;
    }
    else if(turn_cooldown > 0)
    {
        turn_cooldown--;
    }

    if(turn_cooldown == 0)
    {
        if(filtered_dps > -ZUPT_THRESHOLD_DPS && filtered_dps < ZUPT_THRESHOLD_DPS)
        {
            if(zupt_count < ZUPT_COUNT_THRESHOLD)
                zupt_count++;

            if(zupt_count >= ZUPT_COUNT_THRESHOLD)
            {
                was_stationary = 1;

                bias_correction += filtered_dps * 0.1f;
                if(bias_correction > 0.25f)
                {
                    gyro_z_bias += 1;
                    bias_correction = 0.0f;
                }
                else if(bias_correction < -0.25f)
                {
                    gyro_z_bias -= 1;
                    bias_correction = 0.0f;
                }

                return;
            }
        }
        else
        {
            zupt_count = 0;
            if(was_stationary)
            {
                was_stationary = 0;
                bias_correction = 0.0f;
            }
        }
    }
    else
    {
        if(!(filtered_dps > -ZUPT_THRESHOLD_DPS && filtered_dps < ZUPT_THRESHOLD_DPS))
            zupt_count = 0;

        if(was_stationary)
        {
            was_stationary = 0;
            bias_correction = 0.0f;
        }
    }

    Yaw += filtered_dps * DT;

    if(Yaw >= 360.0f)  Yaw -= 360.0f;
    if(Yaw < 0.0f)     Yaw += 360.0f;
}

/**
 * @brief  更新偏航角（直接积分，无滤波）
 * @param  GZ  陀螺仪Z轴原始值
 */
void IMU660RA_UpdateYaw(int16_t GZ)
{
    Yaw += (float)(GZ - gyro_z_bias) / GYRO_SENSITIVITY * DT;

    if(Yaw >= 360.0f)  Yaw -= 360.0f;
    if(Yaw < 0.0f)     Yaw += 360.0f;
}

float IMU660RA_GetYaw(void)
{
    return Yaw;
}

int16_t IMU660RA_GetGyroZBias(void)
{
    return gyro_z_bias;
}
