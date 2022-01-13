#ifndef ROCKER_H
#define ROCKER_H

#include "ADC.h"

#define ROCKER_MAX_VALUE    2000
#define ROCKER_MID_VALUE    0
#define ROCKER_MIN_VALUE    -2000
#define ROCKER_POS_RANGE    (ROCKER_MAX_VALUE - ROCKER_MID_VALUE)
#define ROCKER_NEG_RANGE    (ROCKER_MID_VALUE - ROCKER_MIN_VALUE)
#define ROCKER_ALL_RANGE    (ROCKER_POS_RANGE + ROCKER_NEG_RANGE)

#define ROCKER_DEAD_BAND    100

typedef struct
{
    INT16U Volt;    //MV        原始电压
    INT16U Max;     //MV        电压最大值
    INT16U Mid;     //MV        电压中点值
    INT16U Min;     //MV        电压最小值
    FP32 Rate;      //Float     摇杆占比0%~100%
    INT16S Value;   //Integer   转换结果
}RockerVal_t;

typedef struct
{
    
    struct
    {
        INT8U LeftRockerMode : 1;       //0 自动回中    1 不回中
        INT8U RightRockerMode : 1;
        INT8U Reserve : 6;
    }Status;
    
    RockerVal_t Throttle;
    RockerVal_t Yaw;
    RockerVal_t Roll;
    RockerVal_t Pitch;

    INT8U TaskStage;
    INT32U TaskTimer;
    
}RockerInfo_t;

void RockerInit(void);
void RockerTask(void);
void SetRockerMode(INT8U Index, BOOLEAN Status);

#endif
