#include "stm32f10x.h"
#include "delay.h"
#include "IMU660RA.h"
#include "OLED.h"
#include "PID.h"
#include "timer.h"
#include "sensor.h"
#include "Key.h"
#include "pwm.h"
#include "line.h"
#include "stm32f10x_iwdg.h"
/* ==================== ?? ==================== */

// PWM?????
int V_R = 0;
int V_L = 0;
int16_t AX, AY, AZ, GX, GY, GZ;          // IMU??
uint16_t car_state = 0x0000;              // ??0x0000=??0x1000=
static uint8_t oled_disp_counter = 0;     // OLED?¼
static uint8_t total_laps = 1;            // 1?? (1~5)

/* ==================== ????ö ==================== */
typedef enum {
    SYS_MENU,
    SYS_TASK1_SETUP,
    SYS_TASK1_RUN,
    SYS_TASK2
} SysMode;
static SysMode sys_mode = SYS_MENU;

/* 1?? */
static uint8_t  square_edges = 0;     // ??

/* ====================  ==================== */
static void Key_Scan(void);
static void Task1_Run(void);
static void Show_Menu(void);
static void Show_Task1_Setup(void);
static void Show_Run_Info(void);

/* ==================== ??? ==================== */
static void SystemClock_Config(void)
{
    SystemInit();
}

/* ==================== ?? ==================== */
static void IWDG_Init(void)
{
    /* IWDG ? = LSI ~40kHz64?  ?625Hz */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);      // ? 64
    IWDG_SetReload(1250);                       // ?2?? (64*1250/400002.0s)
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/* ====================  ==================== */
int main(void)
{
    SystemClock_Config();
    OLED_Init();
    OLED_Clear();

    if (IMU660RA_Init() != 0)
    {
        OLED_ShowString(1, 1, "IMU Init Fail!");
        OLED_ShowString(2, 1, "Check Wiring!");
        while (1);  // IMU ????
    }

    Gpio_Init();
    PWM_Init(7199, 0);
    Key_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    IMU660ra_Calibrate();
    TIM2_Init();
    IWDG_Init();        // ?

    sys_mode = SYS_MENU;
    Show_Menu();

    while (1)
    {
        Key_Scan();
        IWDG_ReloadCounter();   // ?

        if (control_flag)
        {
            control_flag = 0;
            IMU660RA_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
            IMU660RA_UpdateYaw_Filtered(GZ);

            if (sys_mode == SYS_TASK1_RUN && car_state & 0x1000)
                Task1_Run();

            if (++oled_disp_counter >= 10)
            {
                oled_disp_counter = 0;
                if (sys_mode == SYS_TASK1_RUN)
                    Show_Run_Info();
            }
        }
    }
}

/* ==================== OLED ? ==================== */

// ?
static void Show_Menu(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "1.Task1 Square");
    OLED_ShowString(2, 1, "2.Task2 Empty");
}

// 1?ı
static void Show_Task1_Setup(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "Task1:Set Lap");
    OLED_ShowNum(2, 1, total_laps, 1);
    OLED_ShowString(2, 3, " Lap");
    OLED_ShowString(3, 1, "K1=Start K3=+/");
    OLED_ShowString(4, 1, "K2=Back");
}

// ??
static void Show_Run_Info(void)
{
    float yaw = IMU660RA_GetYaw();
    uint8_t lap = square_edges / 4 + 1;
    if (lap > total_laps) lap = total_laps;
    OLED_ShowString(1, 1, "Task1 Running");
    OLED_ShowNum(2, 1, lap, 1);
    OLED_ShowString(2, 3, "/");
    OLED_ShowNum(2, 5, total_laps, 1);
    OLED_ShowString(2, 7, " Lap");
    OLED_ShowNum(3, 1, (uint16_t)yaw, 3);
    OLED_ShowString(3, 5, "deg");
}

/* ==================== 1? ==================== */
static void Task1_Run(void)
{
    // ?????? busy ??
    int8_t ret = Auto_RightAngleTurn();

    if (ret == 0)
    {
        //  ??????????PWM? 
        if (!Is_Auto_Turning_Busy())
        {
            // ??  ?
            if (Check_BlackLine())
                track_zhixian1();
        }
    }
    else
    {
        // ?
        square_edges++;
        // ???
        if (square_edges >= total_laps * 4)
        {
            Set_PWM(0, 0);
            car_state = 0x0000;
            sys_mode = SYS_MENU;
            Show_Menu();
        }
    }
}

/* =================== ? ==================== */
static void Key_Scan(void)
{
    uint8_t key = Key_GetNum();
    if (key == 0) return;

    switch (sys_mode)
    {
        case SYS_MENU:
            if (key == 1)
            {
                // 1
                total_laps = 1;
                sys_mode = SYS_TASK1_SETUP;
                Show_Task1_Setup();
            }
            else if (key == 2)
            {
                // 2
                sys_mode = SYS_TASK2;
                OLED_Clear();
                OLED_ShowString(1, 1, "Task2:Coming");
                OLED_ShowString(2, 1, "Soon!");
            }
            break;

        case SYS_TASK1_SETUP:
            if (key == 1)
            {
                // ??1
                Car_Reset_Angle();
                IMU660ra_Calibrate();
                Car_Lock_Current_Heading();

                square_edges = 0;
                car_state = 0x1000;
                sys_mode = SYS_TASK1_RUN;
                OLED_Clear();
            }
            else if (key == 2)
            {
                // ??
                sys_mode = SYS_MENU;
                Show_Menu();
            }
            else if (key == 3)
            {
                // ?? 123451
                total_laps++;
                if (total_laps > 5) total_laps = 1;
                Show_Task1_Setup();
            }
            break;

        case SYS_TASK1_RUN:
            // ?????K4 ??
            if (key <= 4)
            {
                Set_PWM(0, 0);
                car_state = 0x0000;
                sys_mode = SYS_MENU;
                Show_Menu();
            }
            break;

        case SYS_TASK2:
            // 2??
            if (key <= 4)
            {
                sys_mode = SYS_MENU;
                Show_Menu();
            }
            break;
    }
}

