#ifndef FLIGHTMODE_H
#define FLIGHTMODE_H

#include "cpu.h"
#include "Battery.h"

typedef enum
{
    HeadLess = 0,           //无头飞行，起飞后保持起飞前航向，yaw控制回中自动回中。前后左右以大地坐标系为准
    HeadDirection,          //有头飞行，yaw控制飞机头转向并保持，前后左右以机体正向为标准
    FixedAltitude,          //定高模式，在有头飞行基础上，加入高度环并自动保持高度
    FixedPoint,             //定点模式，在定高模式基础上，加入位置环，自动保持当前位置，前后左右不断改变停止位置
    AutoReturn,             //自动返航，自动返回起飞地点
    Landing,                //降落模式，油门不可控制，保持最小输出量缓慢降落
    StandBy                 //停止模式，机器锁定
}FlightMode_e;

typedef struct
{
    FlightMode_e FlightMode;
}FlightModeInfo;


void FlightModeInit(void);
void FlightModeTask(void);

extern volatile FlightModeInfo  g_FlightModeCtrlMsg;

#endif


