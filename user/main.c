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
/* ==================== 全局变量 ==================== */
int V_R = 0;
int V_L = 0;
int16_t AX, AY, AZ, GX, GY, GZ;          // IMU原始数据
uint16_t car_state = 0x0000;              // 车辆状态：0x0000=停止，0x1000=运行
static uint8_t oled_disp_counter = 0;     // OLED刷新计数器
static uint8_t total_laps = 1;            // 任务1设定圈数 (1~5)

/* ==================== 系统模式枚举 ==================== */
typedef enum {
    SYS_MENU,
    SYS_TASK1_SETUP,
    SYS_TASK1_RUN,
    SYS_TASK2
} SysMode;
static SysMode sys_mode = SYS_MENU;

/* 任务1状态变量 */
static uint8_t  square_edges = 0;     // 已完成的边数

/* ==================== 函数声明 ==================== */
static void SystemClock_Config(void);
static void Key_Scan(void);
static void Task1_Run(void);
static void Show_Menu(void);
static void Show_Task1_Setup(void);
static void Show_Run_Info(void);

/* ==================== 系统初始化 ==================== */
static void SystemClock_Config(void)
{
    SystemInit();
}

/* ==================== 独立看门狗初始化 ==================== */
static void IWDG_Init(void)
{
    /* IWDG 时钟 = LSI ~40kHz，64分频 → 约625Hz */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);      // 分频 64
    IWDG_SetReload(1250);                       // 约2秒超时 (64*1250/40000≈2.0s)
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/* ==================== 主函数 ==================== */
int main(void)
{
    SystemClock_Config();
    OLED_Init();
    OLED_Clear();

    if (IMU660RA_Init() != 0)
    {
        OLED_ShowString(1, 1, "IMU Init Fail!");
        OLED_ShowString(2, 1, "Check Wiring!");
        while (1);  // IMU 初始化失败，停机
    }

    Gpio_Init();
    PWM_Init(7199, 0);
    Key_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    IMU660ra_Calibrate();
    TIM2_Init();
    IWDG_Init();        // 启动独立看门狗

    sys_mode = SYS_MENU;
    Show_Menu();

    while (1)
    {
        Key_Scan();
        IWDG_ReloadCounter();   // 喂狗

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

/* ==================== OLED 显示函数 ==================== */

// 主菜单
static void Show_Menu(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "1.Task1 Square");
    OLED_ShowString(2, 1, "2.Task2 Empty");
}

// 任务1圈数设置界面
static void Show_Task1_Setup(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "Task1:Set Lap");
    OLED_ShowNum(2, 1, total_laps, 1);
    OLED_ShowString(2, 3, " Lap");
    OLED_ShowString(3, 1, "K1=Start K3=+/");
    OLED_ShowString(4, 1, "K2=Back");
}

// 运行中信息显示
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

/* ==================== 任务1：正方形循迹 ==================== */
static void Task1_Run(void)
{
    // 先检测直角转弯（转弯中会设 busy 标志，跳过后续循迹）
    int8_t ret = Auto_RightAngleTurn();

    if (ret == 0)
    {
        // ★ 关键修复：转弯进行中（前冲或转弯阶段）不执行循迹，避免PWM冲突 ★
        if (!Is_Auto_Turning_Busy())
        {
            // 空闲状态 → 正常循迹
            if (Check_BlackLine())
                track_zhixian1();
        }
    }
    else
    {
        // 转弯完成
        square_edges++;
        // 完成所有边数则停止
        if (square_edges >= total_laps * 4)
        {
            Set_PWM(0, 0);
            car_state = 0x0000;
            sys_mode = SYS_MENU;
            Show_Menu();
        }
    }
}

/* =================== 按键扫描 ==================== */
static void Key_Scan(void)
{
    uint8_t key = Key_GetNum();
    if (key == 0) return;

    switch (sys_mode)
    {
        case SYS_MENU:
            if (key == 1)
            {
                // 进入任务1设置
                total_laps = 1;
                sys_mode = SYS_TASK1_SETUP;
                Show_Task1_Setup();
            }
            else if (key == 2)
            {
                // 进入任务2（空任务）
                sys_mode = SYS_TASK2;
                OLED_Clear();
                OLED_ShowString(1, 1, "Task2:Coming");
                OLED_ShowString(2, 1, "Soon!");
            }
            break;

        case SYS_TASK1_SETUP:
            if (key == 1)
            {
                // 确认并启动任务1
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
                // 返回菜单
                sys_mode = SYS_MENU;
                Show_Menu();
            }
            else if (key == 3)
            {
                // 切换圈数 1→2→3→4→5→1
                total_laps++;
                if (total_laps > 5) total_laps = 1;
                Show_Task1_Setup();
            }
            break;

        case SYS_TASK1_RUN:
            // 运行中按任意键停止并回菜单（K4 紧急停止）
            if (key <= 4)
            {
                Set_PWM(0, 0);
                car_state = 0x0000;
                sys_mode = SYS_MENU;
                Show_Menu();
            }
            break;

        case SYS_TASK2:
            // 任务2中任意键返回菜单
            if (key <= 4)
            {
                sys_mode = SYS_MENU;
                Show_Menu();
            }
            break;
    }
}

