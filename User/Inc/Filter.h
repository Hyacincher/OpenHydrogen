#ifndef FILTER_H
#define FILTER_H

#include "string.h"
#include "includes.h"
#include "Mymath.h"

#define BIQUAD_BANDWIDTH 1.9f     		/* bandwidth in octaves */
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/

typedef struct
{
    //�˲�����
    volatile FP32 dt;   //����������ʱ��
    volatile FP32 P[2][2];
    volatile FP32 Pdot[4];
    volatile FP32 Q_angle;//�Ƕ��������Ŷ�,������Э����
    volatile FP32 Q_gyro;     //���ٶ��������Ŷȣ�������Ʈ������Э����
    volatile FP32 R_angle;    //���ٶȼ�Э����
    volatile INT8U   C_0;
    volatile FP32 q_bias,angle_err; //q_biasΪ������Ʈ��
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


