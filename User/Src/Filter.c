#include "Filter.h"

static INT16U GetArrayNumber(INT16S Number, INT16U Max);

/*
**������Ϣ      KalmanFilter
**��������      �������˲�
**�������      Input       ��������
                Len         ���鳤��
                Output      �������
                Q           ����������Q      QԽСԽ����ģ��
                R           ����������R
**�������      none
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
        x_now = x_mid + kg*((FP32)Input[ii] - x_mid);//���Ƴ�������ֵ
        p_now = (1 - kg)*p_mid;//����ֵ��Ӧ��Э����

        p_last = p_now;
        x_last = x_now;

        Output[ii] = (INT16U)x_now;
    }
}


/*
*��������   Kalman_filter
*��  �ܣ�   �����������ں�
*��������� *Kalman          ���������
            Angle_m          ����1
            Gyro_m           ����2
*���ز����� none
*/
FP32 KalmanFusion(KalmanInfo* Kalman, FP32 Angle_m, FP32 Gyro_m)
{
    Kalman->kalman_filter_angle += (Gyro_m - Kalman->q_bias) * Kalman->dt;    //������Ԥ�ⷽ�̣���Ϊÿ��Ʈ����ͬ��

    Kalman->Pdot[0]=Kalman->Q_angle - Kalman->P[0][1] - Kalman->P[1][0];
    Kalman->Pdot[1]= -(Kalman->P[1][1]);
    Kalman->Pdot[2]= -(Kalman->P[1][1]);
    Kalman->Pdot[3]= Kalman->Q_gyro;

    Kalman->P[0][0] += Kalman->Pdot[0] * Kalman->dt;
    Kalman->P[0][1] += Kalman->Pdot[1] * Kalman->dt;
    Kalman->P[1][0] += Kalman->Pdot[2] * Kalman->dt;
    Kalman->P[1][1] += Kalman->Pdot[3] * Kalman->dt;

    Kalman->PCt_0 = Kalman->C_0 * Kalman->P[0][0];     //����˷��м����
    Kalman->PCt_1 = Kalman->C_0 * Kalman->P[1][0];

    Kalman->E = Kalman->R_angle + Kalman->C_0 * Kalman->PCt_0;     //��ĸ

    Kalman->K_0 = Kalman->PCt_0 / Kalman->E;   //����ֵ
    Kalman->K_1 = Kalman->PCt_1 / Kalman->E;

    Kalman->angle_err = Angle_m - Kalman->kalman_filter_angle;    
    Kalman->kalman_filter_angle += Kalman->K_0 * Kalman->angle_err; //��״̬�Ŀ��������ƣ����ŽǶ�
    Kalman->q_bias += Kalman->K_1 * Kalman->angle_err;
    Kalman->kalman_filter_angle_dot = Gyro_m-Kalman->q_bias;//���Ž��ٶ�

    Kalman->t_0 = Kalman->PCt_0;     //��������м����
    Kalman->t_1 = Kalman->C_0 * Kalman->P[0][1];

    Kalman->P[0][0] -= Kalman->K_0 * Kalman->t_0;
    Kalman->P[0][1] -= Kalman->K_0 * Kalman->t_1;
    Kalman->P[1][0] -= Kalman->K_1 * Kalman->t_0;
    Kalman->P[1][1] -= Kalman->K_1 * Kalman->t_1;
    
    return Kalman->kalman_filter_angle;
}


/*
**������Ϣ      GetArrayNumber
**��������      ��ȡ������±�
**�������      Number      �����±�
                Max         ���鳤��
**�������      ��ȷ���±�
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
**������Ϣ      LowPassFilter
**��������      ��ͨ�˲���
**�������      Ratio       ��ͨ����
                Input       ��������
                FilterVal   ����˲�ֵ
**�������      none
*/
void LowPassFilter(FP32 Ratio, FP32 *Input, FP32 *FilterVal)
{
    *FilterVal = (Ratio * (*Input)) + ((1-Ratio) * (*FilterVal));
}


/*
**������Ϣ      MoveAveFilter
**��������      �ƶ�ƽ����
**�������      Wave        ���벨��
                Len         ���γ���
                K           �����
**�������      none
*/
void MoveAveFilter(INT16U *Wave, INT16U Len, INT16U K)
{
    FP32 Ave;
    FP32 MoveAve[64];
    INT32U ii;
    
    //�ƶ�ƽ��
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
**������Ϣ      Medfilt1
**��������      �ƶ���ֵ��
**�������      Wave        ���벨��
                Len         ���γ���
                K           �����
**�������      none
*/
void Medfilt1(INT16U *Wave, INT16U Len, INT16U K)
{
    INT16U MedWindow[(2 * 5)+ 1];
    INT16U ii;
    INT16S jj;
    
    for(ii = 0 ; ii < Len ; ii++)
    {
        for(jj = -K ; jj <= K ; jj++)//ȡ��+-K��������
        {
            MedWindow[jj + K] = Wave[GetArrayNumber(ii + jj, Len - 1)];            
        }
        BubbleSortINT16(MedWindow, (2 * K)+ 1);//����
        Wave[ii] = MedWindow[K];//ȡ�м��
    }
}

//���׵�ͨ�˲���
FP32 BiquadLPFFilter(BiquadFilterInfo *Filter, FP32 Input)
{
    const FP32 result = Filter->b0 * Input + Filter->d1;
    Filter->d1 = Filter->b1 * Input - Filter->a1 * result + Filter->d2;
    Filter->d2 = Filter->b2 * Input - Filter->a2 * result;
    return result;
}

//�����˲���
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

//���׵�ͨ�˲���
void BiquadFilterInitLPF(BiquadFilterInfo *filter, INT16U samplingFreq, INT16U filterFreq)
{
    BiquadFilterInit(filter, samplingFreq, filterFreq, BIQUAD_Q, FILTER_LPF);
}