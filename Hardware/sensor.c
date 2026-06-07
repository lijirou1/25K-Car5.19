//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其他用途
//////////////////////////////////////////////////////////////////////////////////
#include "sensor.h"

void SENSOR_GPIO_Config(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// 8路数字口：PA0~PA7 上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
	                              GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
// 获取X通道数字值（channel: 1~8），直接读取对应GPIO引脚
unsigned char digtal(unsigned char channel)
{
    if (channel < 1 || channel > 8) return 0;   // 参数保护
    return (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0 << (channel - 1)) == Bit_SET) ? 1 : 0;
}




