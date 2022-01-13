#ifndef DPORT_H
#define DPORT_H

#include "cpu.h"

#define L_PLUG_CHECK_TRIG       0
#define L_PLUG_CHECK_NTRIG      (!L_PLUG_CHECK_TRIG)
#define L_KEY_TKOF_TRIG         0
#define L_KEY_TKOF_NTRIG        (!L_KEY_TKOF_TRIG)
#define L_KEY_1_TRIG            0
#define L_KEY_1_NTRIG           (!L_KEY_1_TRIG)
#define L_KEY_2_TRIG            0
#define L_KEY_2_NTRIG           (!L_KEY_2_TRIG)
#define L_LOCK_MOTOR_TRIG       0
#define L_LOCK_MOTOR_NTRIG      (!L_LOCK_MOTOR_TRIG)
#define L_LOCK_ROCKER_TRIG      0
#define L_LOCK_ROCKER_NTRIG     (!L_LOCK_ROCKER_TRIG)
#define R_PLUG_CHECK_TRIG       0
#define R_PLUG_CHECK_NTRIG      (!R_PLUG_CHECK_TRIG)
#define R_SW_HEIGHT_TRIG        0
#define R_SW_HEIGHT_NTRIG       (!R_SW_HEIGHT_TRIG)
#define R_SW_DRIEC_TRIG         0
#define R_SW_DRIEC_NTRIG        (!R_SW_DRIEC_TRIG)
#define R_KEY_MODE_TRIG         0
#define R_KEY_MODE_NTRIG        (!R_KEY_MODE_TRIG)
#define R_KEY_1_TRIG            0
#define R_KEY_1_NTRIG           (!R_KEY_1_TRIG)
#define R_KEY_2_TRIG            0
#define R_KEY_2_NTRIG           (!R_KEY_2_TRIG)

#define TRIGGER_INTERVAL_TIME   500     //MS    按键不松手触发间隔

#define TRIGGER_SHORT_TIME      20      //MS
#define TRIGGER_LONG_TIME       2000    //MS


typedef enum
{
    DPortInvalid = 0,
    DPortValid = 1
}DPortLevel_e;

typedef enum
{
    L_PLUG_CHECK = 0,
    L_KEY_TKOF,
    L_KEY_1,
    L_KEY_2,
    L_UNLOCK_MOTOR,
    L_LOCK_ROCKER,
    R_PLUG_CHECK,
    R_SW_HEIGHT,
    R_SW_DRIEC,
    R_KEY_MODE,
    R_KEY_1,
    R_KEY_2,
    AllDPort
}DPortChannel_e;

typedef enum
{
    UserManual = 0,         //手动模式，开环油门，摇杆直接控制PWM输出
    FixedHeight,            //定高模式，高度环闭环控制油门，摇杆控制定高位置
    AutoTakeoff,            //自动起飞，自动起飞至一定高度切换定高
    AutoLanding,            //自动降落，自动降落至地面关闭油门
    AllHeightMode
}HeightMode_e;

typedef enum
{
    HeadDirection = 0,      //有头飞行，yaw控制飞机头转向并保持，前后左右以机体正向为标准
    HeadLess,               //无头飞行，起飞后保持起飞前航向，yaw控制回中自动回中。前后左右以大地坐标系为准
    FixedPoint,             //定点模式，自动保持当前位置，前后左右不断改变停止位置
    PointCruise,            //航点模式，制定航点，自动巡航
    AllDirectMode
}DirectionMode_e;

typedef struct
{
    struct
    {
        INT8U TriggerShort : 1;
        INT8U TriggerLong : 1;
        INT8U Activity : 1;
        INT8U Reserve : 5;
    }Status;
    struct
    {
        INT8U AccumuStart  :1;
        INT8U AccumuEnd  :1;
        INT8U LastIOStatus  : 1;
        INT8U NeedWait : 1;
        INT8U Reserved  : 5;
        INT32U Count;
        INT32U Wait;
        INT32U LevelValid;
        INT32U LevelInValid;
    }IO;
}PortStatus_t;

typedef struct
{
    PortStatus_t DPort[AllDPort];
    
    HeightMode_e  HeightMode;           //高度模式
    HeightMode_e  SelectHeight;         //当前选中项（在模式摇杆置中的时候锁存到当前模式）
    
    DirectionMode_e DirectionMode;      //方向模式
    DirectionMode_e SelectDirection;    //当前选中项（在模式摇杆置中的时候锁存到当前模式）
    
    INT32U TaskTimer;
    INT8U TaskStage;
}DPortInfo_t;


void DigitalPortInit(void);
void DigitalPortTask(void);

extern volatile DPortInfo_t g_DPortCtrlMsg;
#endif

