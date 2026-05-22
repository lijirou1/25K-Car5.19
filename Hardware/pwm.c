#include "pwm.h"

void Gpio_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;     // GPIO初始化结构体          
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  // 定时器基础配置结构体        
	TIM_OCInitTypeDef TIM_OCInitStructure;            // 定时器输出比较配置结构体  
	
	// 使能GPIOA和TIM1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
    // GPIO配置：PA8=TIM1_CH1（右轮PWM），PA9=TIM1_CH2（左轮PWM）
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;        
	// 初始化PA10和PA9
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9; 
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10; 
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr;                
	TIM_TimeBaseStructure.TIM_Prescaler = psc;           
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;           
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;             
	TIM_OCInitStructure.TIM_Pulse       = 0;                         
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;        
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   
	

	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	
	

}
void Set_PWM(int V_R, int V_L)// 设置左右轮PWM占空比，输入范围-100~100，正数前进，负数后退
{
	int PWMA, PWMB;
    PWMA =-V_L * 72;
    PWMB =-V_R * 72;

	if(PWMA>0){
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
		 GPIO_ResetBits(GPIOB, GPIO_Pin_14);
 
	}else{
		 GPIO_ResetBits(GPIOB, GPIO_Pin_13);	
		GPIO_SetBits(GPIOB, GPIO_Pin_14);	 
		PWMA=-PWMA;	

	}
	
	if(PWMB>0)
		{
		GPIO_SetBits(GPIOA, GPIO_Pin_8);    
		GPIO_ResetBits(GPIOB, GPIO_Pin_15); 
		
		}
	else
		{
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
	 
		PWMB=-PWMB;
		}

  TIM_SetCompare3(TIM1,PWMA);  
	TIM_SetCompare2(TIM1,PWMB);	
}
