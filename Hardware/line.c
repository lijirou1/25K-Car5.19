/******************************************************************************
 * line.c - 循迹模块
 *
 * 功能：
 *   1. 8路红外传感器（D1~D8）检测黑线位置
 *   2. 使用增量式PID控制算法计算偏差输出
 *   3. 根据偏差调整左右轮PWM，实现沿黑线行驶
 *   4. Check_BlackLine()提供黑线检测状态
 *   5. Check_Corner()检测90°转弯顶点（所有传感器检测到黑线）
 *   6. Get_Line_Bias()返回最后已知的线位置偏差
 *
 * 硬件连接：
 *   - 红外传感器模块输出接至 MCU GPIO（D1 ~ D8）
 *   - 传感器检测到黑线（反射率低）时输出 1
 *   - 传感器检测到白底（反射率高）时输出 0
 *
 ******************************************************************************/

#include "line.h"
#include "pwm.h"
#include "sensor.h"
#include "PID.h"

/* 外部变量声明 -------------------------------------------------------------- */
extern int V_R;
extern int V_L;

#define CLAMP_SPEED(v)  do { if ((v) > 100) (v) = 100; else if ((v) < -100) (v) = -100; } while(0)

/* PID参数 */
static float line_kp = 1.9f;
static float line_ki = 0.0f;
static float line_kd = 7.5f;

static float line_integral = 0.0f;// PID积分项
static float line_last_error = 0.0f;// PID上次误差值

volatile uint8_t g_auto_turn_busy = 0;   // 全局转弯忙标志，供外部查询
void track_zhixian1(void)
{
    int base_speed = 23;
    int sum = 0, count = 0, error, correction;
    float output;
    static int8_t last_dir = 0;     // 上次有效偏差方向：+1=偏右，-1=偏左，0=未知

    // 读取8路传感器，计算加权和与数量
    // D8(左)~D1(右)，左负右正
    if (D8) { sum += -5; count++; }
    if (D7) { sum += -4; count++; }
    if (D6) { sum += -2; count++; }
    if (D5) { sum += -1; count++; }
    if (D4) { sum += +1; count++; }
    if (D3) { sum += +2; count++; }
    if (D2) { sum += +4; count++; }
    if (D1) { sum += +5; count++; }

    // 掉线处理：根据最后已知偏差方向决定旋转方向找线
    if (count == 0)
    {
        // D8=左 D1=右：上次偏右(线在右)→右旋找线；偏左(线在左)→左旋找线
        if (last_dir > 0)
            Set_PWM(25, -25);   // 右旋
        else if (last_dir < 0)
            Set_PWM(-25, 25);   // 左旋
        else
            Set_PWM(-25, 25);   // 未知方向，默认左旋
        return;
    }

    // 计算平均偏差并更新增量式PID
    error = sum / count;
    last_dir = (error > 0) ? 1 : (error < 0) ? -1 : last_dir;

    line_integral += (float)error;
    if (line_integral > 20)  line_integral =  20;
    if (line_integral < -20) line_integral = -20;

    output = line_kp * (float)error
           + line_ki * line_integral
           + line_kd * ((float)error - line_last_error);
    line_last_error = (float)error;

    correction = (int)output;
    if (correction > 20) correction = 20;
    if (correction < -20) correction = -20;

    V_R = base_speed + correction;
    V_L = base_speed - correction;
    CLAMP_SPEED(V_R);
    CLAMP_SPEED(V_L);
    Set_PWM(V_R, V_L);
}

/* ============ 直角转弯检测函数 ============ */

// 统计D1~D8中检测到黑线的传感器数量
uint8_t Count_BlackSensors(void)
{
    uint8_t cnt = 0;
    if (D1) cnt++; if (D2) cnt++; if (D3) cnt++; if (D4) cnt++;
    if (D5) cnt++; if (D6) cnt++; if (D7) cnt++; if (D8) cnt++;
    return cnt;
}

// 检测直角转弯：最右2路(D1+D2)或最左2路(D7+D8)同时检测到黑线即判定到达直角
// D1(右)~D8(左)
// 返回：1=右转，-1=左转，0=未到达直角
int8_t Check_Square_Corner(void)
{
    if (D1 && D2 && D3)       // 最右2路检测到黑线 → 前方是右直角，需右转
        return 1;
    else if (D7 && D8 && D6)  // 最左2路检测到黑线 → 前方是左直角，需左转
        return -1;

    return 0;           // 未到达直角
}

#define TURN_SPEED          12     // 转弯速度 (0~100)
#define TURN_COOLDOWN       5     // 转弯后冷却节拍数
#define APPROACH_TICKS      25     // 前冲补偿节拍数（传感器到车中心的距离）
#define APPROACH_SPEED      15     // 前冲补偿速度
#define TURN_TIMEOUT        300    // 转弯超时（节拍数，约2秒 @20ms）
#define APPROACH_TIMEOUT    100    // 前冲超时（节拍数，约2秒 @20ms）

// 自动直角转弯：检测到直角后自动转90°，需在主循环中周期性调用
// 返回：0=无动作/转弯中，1=右转完成，-1=左转完成
int8_t Auto_RightAngleTurn(void)
{
    static uint8_t busy = 0;
    static float target = 0.0f;
    static uint8_t target_valid = 0;     // 目标角度已计算标志
    static int8_t dir = 0;
    static uint8_t cooldown = 0;
    static uint8_t approach_timer = 0;
    static uint16_t timeout = 0;
    static uint8_t just_completed = 0;   // 刚完成转弯标志，防止重复计次

    // 冷却倒计时（完成后冷却，防止重复检测同一弯道）
    if (cooldown > 0)
    {
        cooldown--;
        return 0;
    }

    // 刚完成转弯：额外等待一圈，确保已经完全离开弯道区域
    if (just_completed)
    {
        just_completed = 0;
        return 0;
    }

    if (!busy)
    {
        int8_t corner = Check_Square_Corner();
        if (corner == 0) return 0;

        // 检测到直角，先前冲补偿让车中心到达顶点
        dir = corner;
        approach_timer = APPROACH_TICKS;
        busy = 1;
        g_auto_turn_busy = 1;
        return 0;
    }

    // 前冲补偿阶段：保持当前航向直行
    if (approach_timer > 0)
    {
        approach_timer--;
        Car_Go_Straight_To_Target(APPROACH_SPEED, Car_Get_Straight_Target());
        if (approach_timer == 0)
        {
            // 前冲结束，计算目标角度并立即进入转弯，避免空档
            float current = Car_Get_Angle();
            target = current + ((dir > 0) ?88.0f : -88.0f);
            while (target > 180.0f)  target -= 360.0f;
            while (target < -180.0f) target += 360.0f;
            target_valid = 1;
            timeout = TURN_TIMEOUT;
        }
        return 0;
    }

    // 转弯阶段
    if (target_valid)
    {
        Car_Go_Straight_To_Target(TURN_SPEED, target);

        float err = target - Car_Get_Angle();
        while (err > 180.0f)  err -= 360.0f;
        while (err < -180.0f) err += 360.0f;

        // 到达目标角度 或 超时 → 强制完成
        if ((err > -2.0f && err < 2.0f) || --timeout == 0)
        {
            busy = 0;
            g_auto_turn_busy = 0;
            target_valid = 0;
            target = 0.0f;
            cooldown = TURN_COOLDOWN;
            just_completed = 1;
            Car_Lock_Current_Heading();
            // 转弯完成时不要停车，让循迹自然接管
            return dir;
        }
    }
    return 0;
}

char Check_BlackLine(void)
{
    if (D1 || D2 || D3 || D4 || D5 || D6 || D7 || D8)
        return 1;
    return 0;
}

// 返回当前是否处于转弯进行中（包括前冲和转弯阶段）
uint8_t Is_Auto_Turning_Busy(void)
{
    return g_auto_turn_busy;
}
