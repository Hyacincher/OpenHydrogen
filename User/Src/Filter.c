#include "Filter.h"

static INT16U GetArrayNumber(INT16S Number, INT16U Max);

/*
**函数信息      KalmanFilter
**功能描述      卡尔曼滤波
**输入参数      Input       输入数组
                Len         数组长度
                Output      输出缓冲
                Q           卡尔曼参数Q      Q越小越信赖模型
                R           卡尔曼参数R
**输出参数      none
*/
void KalmanFilter(volatile INT16U *Input, INT16U Len, INT16U *Output, FP32 Q, FP32 R)
{
    FP32 x_last = 0;
	FP32 p_last = 0.8;
	FP32 kg;
	FP32 x_mid;
	FP32 x_now;
	FP32 p_mid;
	FP32 p_now;
    INT16U ii;

    x_last = (FP32)Input[0];

    for(ii = 0 ; ii < Len ; ii++)
    {
        x_mid = x_last;
        p_mid = p_last + Q;
        kg = p_mid / (p_mid + R);
        x_now = x_mid + kg*((FP32)Input[ii] - x_mid);//估计出的最有值
        p_now = (1 - kg)*p_mid;//最优值对应的协方差

        p_last = p_now;
        x_last = x_now;

        Output[ii] = (INT16U)x_now;
    }
}


/*
*函数名：   Kalman_filter
*功  能：   卡尔曼数据融合
*输入参数： *Kalman          卡尔曼句柄
            Angle_m          数据1
            Gyro_m           数据2
*返回参数： none
*/
FP32 KalmanFusion(KalmanInfo* Kalman, FP32 Angle_m, FP32 Gyro_m)
{
    Kalman->kalman_filter_angle += (Gyro_m - Kalman->q_bias) * Kalman->dt;    //卡尔曼预测方程，认为每次飘移相同，

    Kalman->Pdot[0]=Kalman->Q_angle - Kalman->P[0][1] - Kalman->P[1][0];
    Kalman->Pdot[1]= -(Kalman->P[1][1]);
    Kalman->Pdot[2]= -(Kalman->P[1][1]);
    Kalman->Pdot[3]= Kalman->Q_gyro;

    Kalman->P[0][0] += Kalman->Pdot[0] * Kalman->dt;
    Kalman->P[0][1] += Kalman->Pdot[1] * Kalman->dt;
    Kalman->P[1][0] += Kalman->Pdot[2] * Kalman->dt;
    Kalman->P[1][1] += Kalman->Pdot[3] * Kalman->dt;

    Kalman->PCt_0 = Kalman->C_0 * Kalman->P[0][0];     //矩阵乘法中间变量
    Kalman->PCt_1 = Kalman->C_0 * Kalman->P[1][0];

    Kalman->E = Kalman->R_angle + Kalman->C_0 * Kalman->PCt_0;     //分母

    Kalman->K_0 = Kalman->PCt_0 / Kalman->E;   //增益值
    Kalman->K_1 = Kalman->PCt_1 / Kalman->E;

    Kalman->angle_err = Angle_m - Kalman->kalman_filter_angle;    
    Kalman->kalman_filter_angle += Kalman->K_0 * Kalman->angle_err; //对状态的卡尔曼估计，最优角度
    Kalman->q_bias += Kalman->K_1 * Kalman->angle_err;
    Kalman->kalman_filter_angle_dot = Gyro_m-Kalman->q_bias;//最优角速度

    Kalman->t_0 = Kalman->PCt_0;     //矩阵计算中间变量
    Kalman->t_1 = Kalman->C_0 * Kalman->P[0][1];

    Kalman->P[0][0] -= Kalman->K_0 * Kalman->t_0;
    Kalman->P[0][1] -= Kalman->K_0 * Kalman->t_1;
    Kalman->P[1][0] -= Kalman->K_1 * Kalman->t_0;
    Kalman->P[1][1] -= Kalman->K_1 * Kalman->t_1;
    
    return Kalman->kalman_filter_angle;
}


/*
**函数信息      GetArrayNumber
**功能描述      获取数组的下标
**输入参数      Number      任意下标
                Max         数组长度
**输出参数      正确的下标
*/
static INT16U GetArrayNumber(INT16S Number, INT16U Max)
{
    if(Number > Max)
    {
        return (Number - Max - 1);
    }
    else if(Number < 0)
    {
        return (Number + Max + 1);
    }
    else
    {
        return Number;
    }
}    


/*
**函数信息      LowPassFilter
**功能描述      低通滤波器
**输入参数      Ratio       低通比率
                Input       输入数组
                FilterVal   输出滤波值
**输出参数      none
*/
void LowPassFilter(FP32 Ratio, FP32 *Input, FP32 *FilterVal)
{
    *FilterVal = (Ratio * (*Input)) + ((1-Ratio) * (*FilterVal));
}


/*
**函数信息      MoveAveFilter
**功能描述      移动平均窗
**输入参数      Wave        输入波形
                Len         波形长度
                K           窗宽度
**输出参数      none
*/
void MoveAveFilter(INT16U *Wave, INT16U Len, INT16U K)
{
    FP32 Ave;
    FP32 MoveAve[64];
    INT32U ii;
    
    //移动平均
    Ave = 0;
    for(ii = 0 ; ii < K ; ii++)
    {
        Ave += Wave[ii];     
    }
    Ave /= K;
    MoveAve[K - 1] = Ave;
    
    for(ii = K ; ii < Len ; ii++)
    {
        Ave = Ave - (Wave[ii - K] / K) + (Wave[ii] / K);
        MoveAve[ii] = Ave;
    }
    for(ii = 0 ; ii < K ; ii++)
    {
        Ave = Ave - (Wave[Len - K + ii] / K) + (Wave[ii] / K);
        MoveAve[ii] = Ave;        
    }
    
    for(ii = 0 ; ii < Len ; ii++)
    {
        Wave[ii] = (INT16U)(MoveAve[ii]);
    }    
}


/*
**函数信息      Medfilt1
**功能描述      移动中值窗
**输入参数      Wave        输入波形
                Len         波形长度
                K           窗宽度
**输出参数      none
*/
void Medfilt1(INT16U *Wave, INT16U Len, INT16U K)
{
    INT16U MedWindow[(2 * 5)+ 1];
    INT16U ii;
    INT16S jj;
    
    for(ii = 0 ; ii < Len ; ii++)
    {
        for(jj = -K ; jj <= K ; jj++)//取得+-K个点数据
        {
            MedWindow[jj + K] = Wave[GetArrayNumber(ii + jj, Len - 1)];            
        }
        BubbleSortINT16(MedWindow, (2 * K)+ 1);//排序
        Wave[ii] = MedWindow[K];//取中间点
    }
}

//二阶低通滤波器
FP32 BiquadLPFFilter(BiquadFilterInfo *Filter, FP32 Input)
{
    const FP32 result = Filter->b0 * Input + Filter->d1;
    Filter->d1 = Filter->b1 * Input - Filter->a1 * result + Filter->d2;
    Filter->d2 = Filter->b2 * Input - Filter->a2 * result;
    return result;
}

//二阶滤波器
void BiquadFilterInit(BiquadFilterInfo *filter, INT16U samplingFreq, INT16U filterFreq, FP32 Q, biquadFilterType_e filterType)
{
    // Check for Nyquist frequency and if it's not possible to initialize filter as requested - set to no filtering at all
    if (filterFreq < (samplingFreq / 2)) {
        // setup variables
        const FP32 sampleRate = samplingFreq;
        const FP32 omega = 2.0f * PI * ((FP32)filterFreq) / sampleRate;
        const FP32 sn = MySinApprox(omega);
        const FP32 cs = MyCosApprox(omega);
        const FP32 alpha = sn / (2 * Q);

        FP32 b0, b1, b2;
        switch (filterType) {
        case FILTER_LPF:
            b0 = (1 - cs) / 2;
            b1 = 1 - cs;
            b2 = (1 - cs) / 2;
            break;
        case FILTER_NOTCH:
            b0 =  1;
            b1 = -2 * cs;
            b2 =  1;
            break;
        }
        const FP32 a0 =  1 + alpha;
        const FP32 a1 = -2 * cs;
        const FP32 a2 =  1 - alpha;

        // precompute the coefficients
        filter->b0 = b0 / a0;
        filter->b1 = b1 / a0;
        filter->b2 = b2 / a0;
        filter->a1 = a1 / a0;
        filter->a2 = a2 / a0;
    }
    else {
        // Not possible to filter frequencies above Nyquist frequency - passthrough
        filter->b0 = 1.0f;
        filter->b1 = 0.0f;
        filter->b2 = 0.0f;
        filter->a1 = 0.0f;
        filter->a2 = 0.0f;
    }

    // zero initial samples
    filter->d1 = filter->d2 = 0;
}

//二阶低通滤波器
void BiquadFilterInitLPF(BiquadFilterInfo *filter, INT16U samplingFreq, INT16U filterFreq)
{
    BiquadFilterInit(filter, samplingFreq, filterFreq, BIQUAD_Q, FILTER_LPF);
}