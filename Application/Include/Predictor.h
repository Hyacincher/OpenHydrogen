#ifndef PREDICTOR_H
#define PREDICTOR_H

#include "includes.h"
#include "Height.h"
#include "Attitude.h"

#define GRAVITY_CMSS    980.665f
#define GRAVITY_MSS     9.80665f

#define PREDICTOR_UPDATE_RATE   RATE_500_HZ     //姿态解算速率
#define PREDICTOR_UPDATE_DT     (1.0/PREDICTOR_UPDATE_RATE)

#define WEIGHT_Z_BARO   0.28f       //气压计修正权重  0.28
#define WEIGHT_ACC_BIAS 0       //加速度计修正权重0.01

typedef struct
{
    struct
    {
        INT8U HeightCaliIsCplt : 1;
        INT8U GravityCaliIsCplt : 1;
        INT8U Reserve : 6;
    }Status;
    
    struct
    {
        FP32 Height;                    //cm
        
        FP32 AccelerationNEU[IMUAxisAll];       //去除重力
        FP32 AccelerationBias[IMUAxisAll];
    }RawData;
    
    struct
    {
        FP32 Position[IMUAxisAll];      //预估出来的相对位置
        FP32 Velocity[IMUAxisAll];      //预估出来的位移速度
    }EstimateData;
    
}PredictorInfo;


void PredictorInit(void);
void PredictorTask(void);


extern volatile PredictorInfo g_PredictorCtrlMsg;

#endif

