#ifndef HEIGHT_H
#define HEIGHT_H

#include "includes.h"
#include "BMP280.h"

typedef struct
{
    struct
    {
        INT8U HeightIsStable : 1;
        INT8U Reserve : 7;
    }Status;
    
    FP32 Altitude;      //气压计原始数据   m
    
    FP32 Velocity;      //融合后的速度    cm/s
    FP32 Height;        //融合后的相对高度    cm
}HeightInfo;

void HeightInit(void);
void HeightTask(void);

extern volatile HeightInfo  g_HeightCtrlMsg;

#endif

