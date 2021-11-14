#ifndef PREDICTOR2_H
#define PREDICTOR2_H

#include "includes.h"
#include "Height.h"
#include "Attitude.h"

#define GRAVITY_CMSS    980.665f
#define GRAVITY_MSS     9.80665f

#define PREDICTOR_UPDATE_DT     (1.0/PREDICTOR_UPDATE_RATE)
#define PREDICTOR_UPDATE_RATE   RATE_500_HZ     //��̬��������

#define WEIGHT_Z_BARO   0.75f       //��ѹ������Ȩ��  0.28
#define WEIGHT_ACC_BIAS 0.02f       //���ٶȼ�����Ȩ��0.01

#define TIME_CONTANST_Z 5.0f
#define K_ACC_Z (5.0f / (TIME_CONTANST_Z * TIME_CONTANST_Z * TIME_CONTANST_Z))
#define K_VEL_Z (3.0f / (TIME_CONTANST_Z * TIME_CONTANST_Z))
#define K_POS_Z (3.0f / TIME_CONTANST_Z)

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
        FP32 Height;                            //cm
        
        FP32 AccelerationNEU[IMUAxisAll];       //ȥ������
        FP32 AccelerationBias[IMUAxisAll];      //����ƫ��
    }RawData;
    
    struct
    {
        FP32 Acceleration[IMUAxisAll];  //����֮��ļ��ٶ�
        FP32 Position[IMUAxisAll];      //Ԥ�����������λ��
        FP32 Velocity[IMUAxisAll];      //Ԥ��������λ���ٶ�
    }EstiData;
    
}PredictorInfo;


void PredictorInit(void);
void PredictorTask(void);


extern volatile PredictorInfo g_PredictorCtrlMsg;
extern FP32 Vel;
extern FP32 Pos;
#endif

