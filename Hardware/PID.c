#include "PID.h"
#include "IMU660RA.h"
#include "delay.h"
#include "pwm.h"

// 车辆直行目标角、yaw角度偏移量（支持重置角度）
static float straight_target_angle = 0.0f;
static float yaw_offset = 0.0f;
extern int V_R;               // 右电机速度输出
extern int V_L;               // 左电机速度输出
// 直行控制使用的位置式PID结构体
typedef struct
{
    float target;
    float measure;
    float err;
    float last_err;
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float output;
} PID_TypeDef;

static PID_TypeDef PID_Str;        // 直行控制PID实例
static PID_TypeDef PID_Turn;       // 转弯控制PID实例

// 初始化PID参数（通用）
static void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->target = 0;
    pid->measure = 0;
    pid->err = 0;
    pid->last_err = 0;
    pid->integral = 0;
    pid->output = 0;
}
// 采样陀螺仪静态零偏，并重置直行PID
void IMU660ra_Calibrate(void)
{
    // 调用IMU660RA零偏校准（100次采样均值+卡尔曼初始化）
    IMU660RA_CalibrateGyroZ();
    yaw_offset = IMU660RA_GetYaw(); // 校准后当前yaw设为偏移基准

    PID_Init(&PID_Str, 1.3f, 0.0f, 12.0f);
    PID_Init(&PID_Turn, 1.0f, 0.0f, 0.5f);
}
// 位置式PID计算（通用，可指定PID实例）
static float PID_Calc_Generic(PID_TypeDef *pid, float measure, float target)
{
    pid->measure = measure;
    pid->target = target;

    pid->err = pid->target - pid->measure;

    // 积分限幅，避免长时间偏差导致积分饱和
    pid->integral += pid->err;
    if (pid->integral > 200)
        pid->integral = 200;
    if (pid->integral < -200)
        pid->integral = -200;

    // 位置式PID输出
    pid->output = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->last_err);

    pid->last_err = pid->err;
    return pid->output;
}
// 直行PID专用计算（保持原有行为）
static float PID_Calc(float measure, float target)
{
    return PID_Calc_Generic(&PID_Str, measure, target);
}
// 清空PID内部状态（通用）
static void PID_Clear(PID_TypeDef *pid)
{
    pid->err = 0;
    pid->last_err = 0;
    pid->integral = 0;
    pid->output = 0;
}
// 速度值限幅到 [-100, 100]
#define CLAMP_SPEED(v)  do { if ((v) > 100) (v) = 100; else if ((v) < -100) (v) = -100; } while(0)

//将角度差归一化到 -180° ~ +180°
static float Angle_Normalize(float diff)
{
    while (diff > 180.0f)
        diff -= 360.0f;
    while (diff < -180.0f)
        diff += 360.0f;
    return diff;
}
// 角速度已在主循环中每20ms更新，此函数仅作兼容声明，实际无操作
void Car_Update_Angle(void)
{
}
// 读取当前偏航角（基于IMU660RA，以校准时刻为基准偏移）
float Car_Get_Angle(void)
{
    float diff = IMU660RA_GetYaw() - yaw_offset;

    // 归一化到 -180° ~ +180°
    if (diff > 180.0f)
        diff -= 360.0f;
    if (diff < -180.0f)
        diff += 360.0f;

    return diff;
}
void Car_Set_Straight_Target(float target)
{
    straight_target_angle = target;
}
float Car_Get_Straight_Target(void)
{
    return straight_target_angle;
}
void Car_Lock_Current_Heading(void)
{
    straight_target_angle = Car_Get_Angle();
    PID_Clear(&PID_Str);
}
// 重新开始角度计算：以当前IMU yaw为偏移基准
void Car_Reset_Angle(void)
{
    yaw_offset = IMU660RA_GetYaw();
    straight_target_angle = 0.0f;
    PID_Clear(&PID_Str);
}

/**
 * @brief  内部直行核心：基于Yaw偏差计算修正，驱动电机
 * @param  speed     基础速度
 * @param  target    目标航向角（度）
 * @param  max_corr  最大修正量，与速度联动防过调
 */
static void Straight_Core(int speed, float target, int max_corr)
{
    float err = Angle_Normalize(target - Car_Get_Angle());
    int correction;

    if (err > -1.0f && err < 1.0f)
    {
        PID_Clear(&PID_Str);
        correction = 0;
    }
    else
    {
        correction = (int)PID_Calc(0, err);
        if (correction > max_corr) correction = max_corr;
        else if (correction < -max_corr) correction = -max_corr;
    }

    V_L = speed - correction;
    V_R = speed + correction;
    CLAMP_SPEED(V_L);
    CLAMP_SPEED(V_R);
    Set_PWM(V_R, V_L);
}

// 按锁定的目标航向直行
void Car_Go_Straight(int speed)
{
    Straight_Core(speed, straight_target_angle, 10);
}

// 按指定目标角度直行
void Car_Go_Straight_To_Target(int speed, float target_yaw)
{
    Straight_Core(speed, target_yaw, 10);
}

/**
 * @brief  使用PID闭环控制，使车体精确转到指定Yaw角度
 *
 *         工作原理：
 *         - 计算目标Yaw与当前Yaw的误差（归一化到-180~180）
 *         - 通过转弯PID（位置式）计算修正量
 *         - 修正量为正表示需要顺时针转 → 右轮正PWM、左轮负PWM
 *         - 修正量为负表示需要逆时针转 → 右轮负PWM、左轮正PWM
 *         - 到达目标角度后自动停止并返回1
 *
 * @param  target_yaw  目标偏航角（度，-180~180）
 * @param  speed       最大转弯速度 (0~100)，越大转弯越快但可能过冲
 * @retval 0  正在转弯中，需要继续调用
 * @retval 1  已到达目标角度，本次转弯完成
 */
uint8_t Car_Turn_To_Yaw(float target_yaw, int speed)
{
    float angle = Car_Get_Angle();
    float err = Angle_Normalize(target_yaw - angle);
    float correction;
    int pwm_magnitude;

    // 到达判断：偏差绝对值小于1度认为到位
    if (err > -1.0f && err < 1.0f)
    {
        PID_Clear(&PID_Turn);
        Set_PWM(0, 0);
        return 1;
    }

    // 使用转弯PID计算修正量
    correction = PID_Calc_Generic(&PID_Turn, err, 0.0f);
    if (correction < 0) correction = -correction;

    pwm_magnitude = (int)correction;
    if (pwm_magnitude > speed) pwm_magnitude = speed;
    if (pwm_magnitude < 15)    pwm_magnitude = 15;

    if (err > 0.0f)
    {
        V_R =  pwm_magnitude;
        V_L = -pwm_magnitude;
    }
    else
    {
        V_R = -pwm_magnitude;
        V_L =  pwm_magnitude;
    }

    CLAMP_SPEED(V_R);
    CLAMP_SPEED(V_L);
    Set_PWM(V_R, V_L);
    return 0;
}
