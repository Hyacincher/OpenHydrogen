#ifndef FFT_H
#define FFT_H

#include "includes.h"

/*数据矫正临界区间*/
#define NEG_CRITICAL_PI   -3.0415926535     //相位平均下边界
#define POS_CRITICAL_PI   3.0415926535      //相位平均上边界

/*Goerttzel算法*/
#define SIGNAL_FREQ     5000        //signal frequency

#define K_COEF      (SIGNAL_FREQ*SAMPLE_POINT_SIZE/SAMPLE_RATE) //Goertzel coefficent - MUST be integer, or quality fall

#define SCALING     ((FP32)SAMPLE_POINT_SIZE / 2.0)     //scaling the data
#define OMEGA       ((2.0 * PI * K_COEF) / SAMPLE_POINT_SIZE)

#define INT_COEF    1024    //sin/cos Amp

void FFT(FP32 *pr , FP32 *pi , INT32U n , FP32 *fr , FP32 *fi);
FP32 GetAmplitude(FP32 fr, FP32 fi);
FP32 GetPhase(FP32 fr, FP32 fi);
INT16U FindMaxMag(FP32 *Mag,INT16U Len);
void apFFT(FP32 *pr, FP32 *pi , INT32U n , FP32 *fr , FP32 *fi);
FP32 CalculateDiffPhase(FP32 StartPhase, FP32 EndPhase);
FP32 CalculateDistance(INT32U Freq, FP32 DisPhase);
FP32 CalculateAvePhase(FP32 *Phase, INT16U Len, INT8U AveMode);


FP32 Goertzel(INT16U* data);
void InitGoertzel(void);

#endif // FFT_H


