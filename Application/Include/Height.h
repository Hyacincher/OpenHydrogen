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
    
    FP32 Altitude;      //气压计原始数据
    
    FP32 Position;      //数据融合后的高度
    FP32 Velocity;      //Z轴运动速度
}HeightInfo;

void HeightInit(void);
void HeightTask(void);

extern volatile HeightInfo  g_HeightCtrlMsg;

#endif

