#ifndef	__LINE_H__
#define __LINE_H__

#include "stm32f10x.h"

void track_zhixian1(void);
char Check_BlackLine(void);

/* 直角转弯检测 */
uint8_t Count_BlackSensors(void);     // 统计D1~D8中检测到黑线的传感器数量
int8_t  Check_Square_Corner(void);    // 检测直角：返回1=右转，-1=左转，0=未到达
int8_t  Auto_RightAngleTurn(void);    // 自动直角转弯：返回1=右转完成，-1=左转完成，0=无动作/进行中
uint8_t Is_Auto_Turning_Busy(void);   // 返回1=正在转弯中（前冲或转弯阶段），0=空闲

#endif
