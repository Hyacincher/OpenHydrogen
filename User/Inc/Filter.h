#ifndef FILTER_H
#define FILTER_H

#include "string.h"
#include "includes.h"
#include "Mymath.h"

#define BIQUAD_BANDWIDTH 1.9f     		/* bandwidth in octaves */
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/

typedef struct
{
    //滤波参数
    volatile FP32 dt;   //卡尔曼采样时间
    volatile FP32 P[2][2];
    volatile FP32 Pdot[4];
    volatile FP32 Q_angle;//角度数据置信度,陀螺仪协方差
    volatile FP32 Q_gyro;     //角速度数据置信度，陀螺仪飘移噪声协方差
    volatile FP32 R_angle;    //加速度计协方差
    volatile INT8U   C_0;
    volatile FP32 q_bias,angle_err; //q_bias为陀螺仪飘移
    volatile FP32 PCt_0,PCt_1,E;
    volatile FP32 K_0,K_1,t_0,t_1;
    volatile FP32 kalman_filter_angle, kalman_filter_angle_dot;
}KalmanInfo;

typedef struct
{
    float b0, b1, b2, a1, a2;
    float d1, d2;
}BiquadFilterInfo;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;

void KalmanFilter(volatile INT16U *Input, INT16U Len, INT16U *Output, FP32 Q, FP32 R);
FP32 KalmanFusion(KalmanInfo* Kalman, FP32 Angle_m, FP32 Gyro_m);
void LowPassFilter(FP32 Ratio, FP32 *Input, FP32 *FilterVal);
void Medfilt1(INT16U *Wave, INT16U Len, INT16U K);
void MoveAveFilter(INT16U *Wave, INT16U Len, INT16U K);

FP32 BiquadLPFFilter(BiquadFilterInfo *Filter, FP32 Input);
void BiquadFilterInit(BiquadFilterInfo *filter, INT16U samplingFreq, INT16U filterFreq, FP32 Q, biquadFilterType_e filterType);
void BiquadFilterInitLPF(BiquadFilterInfo *filter, INT16U samplingFreq, INT16U filterFreq);
#endif // FILTER_H


