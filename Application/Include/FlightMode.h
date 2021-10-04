#ifndef FLIGHTMODE_H
#define FLIGHTMODE_H

#include "cpu.h"
#include "Battery.h"
#include "PID.h"

typedef enum
{
    UserManual = 0,         //手动模式，开环油门，摇杆直接控制PWM输出
    FixedHeight,            //定高模式，高度环闭环控制油门，摇杆控制定高位置
    AutoTakeoff,            //自动起飞，自动起飞至一定高度切换定高
    AutoLanding,            //自动降落，自动降落至地面关闭油门
    ThroStandby             //停止模式，机器锁定
}ThrottleMode_e;

typedef enum
{
    HeadDirection = 0,      //有头飞行，yaw控制飞机头转向并保持，前后左右以机体正向为标准
    HeadLess,               //无头飞行，起飞后保持起飞前航向，yaw控制回中自动回中。前后左右以大地坐标系为准
    FixedPoint,             //定点模式，自动保持当前位置，前后左右不断改变停止位置
    PointCruise,            //航点模式，制定航点，自动巡航
    DirecStandBy            //停止模式，机器锁定
}DirectionMode_e;

typedef struct
{
    ThrottleMode_e ThrottleMode;
    DirectionMode_e DirectionMode;
}FlightModeInfo;


void FlightModeInit(void);
void FlightModeTask(void);
void FlightWorking(void);
void FlightStandBy(void);

extern volatile FlightModeInfo  g_FlightModeCtrlMsg;

#endif


