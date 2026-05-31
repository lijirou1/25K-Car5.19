#include "stm32f10x.h"
#include "delay.h"
#include "IMU660RA.h"
#include "OLED.h"
#include "PID.h"
#include "timer.h"
#include "Key.h"
#include "pwm.h"
#include "gpio.h"

/* ==================== ȫ�ֱ��� ==================== */

// ����PWM�ٶ�ֵ�����ⲿģ��ʹ��
int V_R = 0;
int V_L = 0;
// �������б�־
static uint8_t car_running = 0;
// ����ѡ��
static uint8_t task_select = 1;
// OLED��ʾ��������ÿ10�����ڣ�200ms��ˢ��һ�Σ�����I2C�Կ��ƻ�·�ĸ���
static uint8_t oled_disp_counter = 0;

/* ==================== ��ʱ�����ߣ����ڶ�ʱֱ�еȣ� ==================== */
static uint16_t task_timer = 0;       // ��ʱ������ֵ����λ��20ms����
static uint8_t  timer_active = 0;     // ��ʱ���Ƿ񼤻�

// ������ʱ��������ms��20ms����ȡ��
static void Timer_Start(uint16_t ms)
{
    task_timer = (ms + 19) / 20;
    timer_active = 1;
}

// ��鶨ʱ���Ƿ��ڣ�ÿ20ms���ڵ���һ��
// ���� 1 = ʱ�䵽���Զ�ֹͣ��ʱ�������� 0 = ���ڼ�ʱ��
static uint8_t Timer_Check(void)
{
    if(!timer_active) return 0;
    if(task_timer > 0)
    {
        task_timer--;
        if(task_timer == 0)
        {
            timer_active = 0;
            return 1;
        }
    }
    return 0;
}

// ��ݺ꣺��ʱֱ�У���״̬���ظ����룩
#define TIMED_STRAIGHT(speed, yaw, ms, next_state)  do { \
    Car_Go_Straight_To_Target(speed, yaw);               \
    if(!timer_active) Timer_Start(ms);                   \
    if(Timer_Check()) now_state = next_state;             \
} while(0)

/* ==================== ״̬ö�� ==================== */
typedef enum {
    // ����1
    STATE1_Straight1, STATE1_Straight2, STATE1_Straight3, STATE1_STOP1,
    // ����2
    STATE2_Straight1, STATE2_Straight2, STATE2_Straight3,
    STATE2_Straight4, STATE2_Straight5, STATE2_STOP1,
    // ����3
    STATE3_Straight1,  STATE3_Straight2,  STATE3_Straight3,
    STATE3_Straight4,  STATE3_Straight5,  STATE3_Straight6,
    STATE3_Straight7,  STATE3_Straight8,  STATE3_Straight9,
    STATE3_Straight10, STATE3_Straight11, STATE3_STOP1
} CarState;

static CarState now_state;

/* ==================== �������� ==================== */
static void Key_Scan(void);
static void Car_Run_StateMachine(void);
static void OLED_DisplayYaw(void);

/* ==================== ������ ==================== */
int main(void)
{
    /* ---- ϵͳ��ʼ�� ---- */
    SystemInit();
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Task:1");
    OLED_ShowString(2, 1, "Yaw:");
    IMU660RA_Init();
    Gpio_Init();
    PWM_Init(7199, 0);
    Key_Init();
    Buzzer_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    IMU660ra_Calibrate();
    TIM2_Init();

    /* ---- ��ѭ�� ---- */
    while (1)
    {
        Key_Scan();//ɨ�谴��
        if (control_flag)
        {
            control_flag = 0;
            int16_t ax, ay, az, gx, gy, gz;
            Buzzer_Update(20);
            IMU660RA_GetData(&ax, &ay, &az, &gx, &gy, &gz);
            IMU660RA_UpdateYaw_Filtered(gz);
            if (car_running)
                Car_Run_StateMachine();
            else
                Set_PWM(0, 0);

            if (++oled_disp_counter >= 10)
            {
                oled_disp_counter = 0;
                OLED_DisplayYaw();
            }
        }
    }
}

/**
 * @brief  ��OLED����ʾƫ����
 */
static void OLED_DisplayYaw(void)
{
    OLED_ShowNum(2, 6, (uint16_t)IMU660RA_GetYaw(), 3);
}
/* ===================����ɨ��==================== */
static void Key_Scan(void)
{
    uint8_t key = Key_GetNum();
    if (key == 0) return;

    // ͨ�ó�ʼ������
    Car_Reset_Angle();
    IMU660ra_Calibrate();
    Car_Lock_Current_Heading();
    timer_active = 0;
    car_running = 1;

    task_select = key;
    OLED_Clear();
    OLED_ShowString(1, 1, "Task:");
    OLED_ShowNum(1, 6, key, 1);
    OLED_ShowString(2, 1, "Yaw:");

    switch (key)
    {
    case 1: now_state = STATE1_Straight1; break;
    case 2: now_state = STATE2_Straight1; break;
    case 3: Car_Set_Straight_Target(0.0f); now_state = STATE3_Straight1; break;
    }
}
/* =============����״̬��============= */
static void Car_Run_StateMachine(void)
{
    switch (task_select)
    {
    case 1:
        switch (now_state)
        {
        case STATE1_Straight1: TIMED_STRAIGHT(35,  1.0f,   3400, STATE1_Straight2); break;
        case STATE1_Straight2: TIMED_STRAIGHT(35, -90.0f,   500, STATE1_Straight3); break;
        case STATE1_Straight3: TIMED_STRAIGHT(35,  0.0f,  1000, STATE1_STOP1);      break;
        case STATE1_STOP1:
            Set_PWM(0, 0);
            car_running = 0;
            break;
        }
        break;

    case 2:
        switch (now_state)
        {
        case STATE2_Straight1: TIMED_STRAIGHT(35,  0.0f,   1700, STATE2_Straight2); break;
        case STATE2_Straight2: TIMED_STRAIGHT(35, -60.0f,   880, STATE2_Straight3); break;
        case STATE2_Straight3: TIMED_STRAIGHT(35,  60.0f,  1080, STATE2_Straight4); break;
        case STATE2_Straight4: TIMED_STRAIGHT(35, -60.0f,  1100, STATE2_Straight5); break;
        case STATE2_Straight5: TIMED_STRAIGHT(35,   0.0f,   819, STATE2_STOP1);     break;
        case STATE2_STOP1:
            Set_PWM(0, 0);
            car_running = 0;
            break;
        }
        break;

    case 3:
        switch (now_state)
        {
        case STATE3_Straight1:  TIMED_STRAIGHT(35,   1.0f,  1700, STATE3_Straight2);  break;
        case STATE3_Straight2:  TIMED_STRAIGHT(35, 300.0f,   370, STATE3_Straight3);  break;
        case STATE3_Straight3:  TIMED_STRAIGHT(35, 230.0f,   570, STATE3_Straight4);  break;
        case STATE3_Straight4:  TIMED_STRAIGHT(35, 120.0f,   770, STATE3_Straight5);  break;
        case STATE3_Straight5:  TIMED_STRAIGHT(35,  30.0f,   570, STATE3_Straight6);  break;
        case STATE3_Straight6:  TIMED_STRAIGHT(40,   1.0f,   674, STATE3_Straight7);  break;
        case STATE3_Straight7:  TIMED_STRAIGHT(35, 300.0f,   830, STATE3_Straight8);  break;
        case STATE3_Straight8:  TIMED_STRAIGHT(35,  40.0f,   570, STATE3_Straight9);  break;
        case STATE3_Straight9:  TIMED_STRAIGHT(35, 140.0f,   570, STATE3_Straight10); break;
        case STATE3_Straight10: TIMED_STRAIGHT(35, 240.0f,   700, STATE3_Straight11); break;
        case STATE3_Straight11: TIMED_STRAIGHT(40,   0.0f,  1600, STATE3_STOP1);      break;
        case STATE3_STOP1:
            Set_PWM(0, 0);
            car_running = 0;
            break;
        }
        break;
    }
}
