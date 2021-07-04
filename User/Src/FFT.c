#include "FFT.h"


/*
*��������FFT
*��  �ܣ����ٸ���Ҷ�任
*���������
    FP32 pr[n]	���n�����������ʵ����������ɢ����Ҷ�任����
    FP32 pi[n]	���n������������鲿
    FP32 fr[n]	������ɢ����Ҷ�任��n��ʵ��
    FP32 fi[n]	������ɢ����Ҷ�任��n���鲿
    int n	��������
*���ز�����None
*/
void FFT(FP32 *pr , FP32 *pi , INT32U n , FP32 *fr , FP32 *fi)
{
    INT32S it,m,is,i,j,nv,l0;
    FP32 k = (log(n) / log(2));
    FP32 p,q,s,vr,vi,poddr,poddi;

    for (it=0; it<=n-1; it++)  //��pr[0]��pi[0]ѭ����ֵ��fr[]��fi[]
    {
		m=it;
		is=0;
		for(i=0; i<=k-1; i++)
        {
			j=m/2;
			is=2*is+(m-2*j);
			m=j;
		}
        fr[it]=pr[is];
        fi[it]=pi[is];
    }

    pr[0]=1.0;
    pi[0]=0.0;
    p=6.283185306/(1.0*n);
    pr[1]=cos(p); //��w=e^-j2pi/n��ŷ����ʽ��ʾ
    pi[1]=-sin(p);

    for (i=2; i<=n-1; i++)  //����pr[]
    {
		p=pr[i-1]*pr[1];
		q=pi[i-1]*pi[1];
		s=(pr[i-1]+pi[i-1])*(pr[1]+pi[1]);
		pr[i]=p-q; pi[i]=s-p-q;
    }
    for (it=0; it<=n-2; it=it+2)
    {
		vr=fr[it];
		vi=fi[it];
		fr[it]=vr+fr[it+1];
		fi[it]=vi+fi[it+1];
		fr[it+1]=vr-fr[it+1];
		fi[it+1]=vi-fi[it+1];
    }
	m=n/2;
	nv=2;
    for (l0=k-2; l0>=0; l0--) //��������
    {
		m=m/2;
		nv=2*nv;
        for (it=0; it<=(m-1)*nv; it=it+nv)
          for (j=0; j<=(nv/2)-1; j++)
            {
				p=pr[m*j]*fr[it+j+nv/2];
				q=pi[m*j]*fi[it+j+nv/2];
				s=pr[m*j]+pi[m*j];
				s=s*(fr[it+j+nv/2]+fi[it+j+nv/2]);
				poddr=p-q;
				poddi=s-p-q;
				fr[it+j+nv/2]=fr[it+j]-poddr;
				fi[it+j+nv/2]=fi[it+j]-poddi;
				fr[it+j]=fr[it+j]+poddr;
				fi[it+j]=fi[it+j]+poddi;
            }
    }

    return;
}


/*
*��������GetAmplitude
*��  �ܣ�ȡ�ø���Ҷ�仯����ķ�ֵ
*���������
    FP32 amplitude    ���渵��Ҷ�任�ķ�ֵ
    int    n            ��������
    FP32 fr[n]	    ��ɢ����Ҷ�任��n��ʵ��
    FP32 fi[n]	    ��ɢ����Ҷ�任��n���鲿
*���ز�����None
*/
FP32 GetAmplitude(FP32 fr, FP32 fi)
{
    return sqrt(fr*fr+fi*fi);  //��ֵ����
}

/*
*��������GetPhase
*��  �ܣ�ȡ�ø���Ҷ�仯�������λ
*���������
    FP32 phase        ���渵��Ҷ�任����λ
    int    n            ��������
    FP32 fr[n]	    ��ɢ����Ҷ�任��n��ʵ��
    FP32 fi[n]	    ��ɢ����Ҷ�任��n���鲿
*���ز�����None
*/
FP32 GetPhase(FP32 fr, FP32 fi)
{
    FP32 phase;
    
    //phase = atan(fi/fr);  
    phase = atan2(fi , fr);  
    
    return phase;
}


/*
**������Ϣ      FindMaxMag
**��������      Ѱ�����ķ�ֵ��
**�������      Mag     ��ֵ����
                Len     ���鳤��
**�������      ����ֵλ��
*/
INT16U FindMaxMag(FP32 *Mag,INT16U Len)
{
    INT16U ii;
    INT16U Max = 0;
    FP32 Value = 0;
    
    for(ii = 1 ; ii < Len ; ii++)
    {
        if(Mag[ii] > Value)
        {
            Value = Mag[ii];
            Max = ii;
        }
    }
    return Max;
}

/*
*��������apFFT
*��  �ܣ�ȫ��λ����Ҷ�任
*���������
        FP32 *pr        ʵ������
        FP32 *pi        �鲿����
        INT32U n        �����������������������������
        FP32 *fr        ���ʵ��
        FP32 *fi        ���ʵ��
*���ز�����None
*/
void apFFT(FP32 *pr , FP32 *pi , INT32U n , FP32 *fr , FP32 *fi)
{
//    INT16U ii;
//    FP64 Result[64];
//	
//    for(ii = 0 ; ii < 63 ; ii++)
//    {
//        Result[ii] = (FP64)pr[ii];
//    }
//    
//    for(ii = 0 ; ii < 63 ; ii++)
//    {
//        Result[ii] *= ConvHamming[ii];
//    }
//	
//	pr[0] = (Result[31] * 10000);
//	
//    for(ii = 1 ; ii < 32 ; ii++)
//    {
//		pr[ii] = ((Result[31 + ii] + Result[ii - 1]) * 10000);
//    }
//	
//    FFT(pr, pi, 32, fr, fi);
}


/*
**������Ϣ      CalculateDiffPhase
**��������      �����ĩ��λ��
**�������      StartPhase      ����λ
                EndPhase        ĩ��λ
**�������      ��λ��
*/
FP32 CalculateDiffPhase(FP32 StartPhase, FP32 EndPhase)
{
    FP32 Start,End;
    FP32 DisPhase;
    
    Start = StartPhase;
    End = EndPhase;
    
    if(StartPhase < 0)
    {
        Start = (PI * 2) + Start;
    }

    if(EndPhase < 0)
    {
        End = (PI * 2) + End;
    }
    
    if(Start > End)
    {
        DisPhase = Start - End;
    }
    else
    {
        DisPhase = (PI * 2) - End + Start;
    }
    
    return DisPhase;
}


/*
**������Ϣ      CalculateAvePhase
**��������      ������λ��ƽ��
**�������      Phase       ��λ����
                Len         ���鳤��
                AveMode     ƽ����ʽ
                    1ֱ��ƽ��
                    2ȥ����������ƽ��
                    3ȥ��������ȡ��ֵ��Ȼ�����������ֵ���е�ͨ�˲�
**�������      ��λƽ��
*/
FP32 CalculateAvePhase(FP32 *Phase, INT16U Len, INT8U AveMode)
{
    INT8U Neg,Pos,ii;
    FP32 Average = 0;
    
    if(Len == 1)
    {
        return Phase[0];
    }
    
    Neg = 0;
    Pos = 0;
    for(ii = 0 ; ii < Len ; ii++)
    {
        if((Phase[ii] < NEG_CRITICAL_PI) || (Phase[ii] > POS_CRITICAL_PI))
        {
            if(Phase[ii] > 0)
            {
                Pos++;
            }
            else
            {
                Neg++;
            }
        }
    }
    if((Pos == 0) || (Neg == 0))//ȫΪ���򸺣����ݲ����ٽ磬���߾���0����
    {
        for(ii = 0 ; ii < Len ; ii++)
        {
            Average += Phase[ii];
        }
    }
    else
    {
        if(Neg > Pos)
        {
            for(ii = 0 ; ii < Len ; ii++)
            {
                if(Phase[ii] > 0)
                {
                    Phase[ii] = -PI;
                }
                Average += Phase[ii];
            }
        }
        else
        {
            for(ii = 0 ; ii < Len ; ii++)
            {
                if(Phase[ii] < 0)
                {
                    Phase[ii] = PI;
                }
                Average += Phase[ii];
            }        
        }        
    }
    
    if(AveMode == 0)//ֱ��ƽ��
    {
        Average /= Len;        
    }
    else if(AveMode == 1)//ȥ��������ƽ��
    {
        Average = 0;
        BubbleSortFP32(Phase, Len);
        
        if(Len < 3)
        {
            for(ii = 0 ; ii < Len ; ii++)
            {
                Average += Phase[ii];
            }
            Average /= Len;             
        }
        else
        {
            for(ii = 1 ; ii < Len - 1 ; ii++)
            {
                Average += Phase[ii];
            }
            Average /= Len - 2;            
        }
    }
    else if(AveMode == 2)//ȡ��ֵ����ͨ(�ų�������)
    {
        Average = 0;
        BubbleSortFP32(Phase, Len);
        
        if((Len % 2) == 0)//˫��
        {
            Average += Phase[Len / 2];
            Average += Phase[(Len / 2) - 1];   
            Average /= 2;
        }
        else
        {
            Average = Phase[(Len / 2)];
        }
        if(Len > 3)
        {
            for(ii = 1 ; ii < Len - 1 ; ii++)
            {
                LowPassFilter(0.04, &Phase[ii], &Average);
            }            
        }
    }
    
    return Average;
}


/*
**������Ϣ      CalculateDistance
**��������      �������
**�������      Freq        ���Ƶ��
                DisPhase    ��λ��
**�������      ����
*/
FP32 CalculateDistance(INT32U Freq, FP32 DisPhase)
{
    FP32 Distance = 0; 

    while(DisPhase > (PI2))
    {
        DisPhase -= (PI2);
    }
    Distance = (0.5 * 300000000 * DisPhase) / (PI2 * Freq);   
    
    return Distance;
}


FP32 Goertzel(INT16U* data)
{
//    volatile INT16U ii;
//    volatile INT64S real = 0.0;
//    volatile INT64S imag = 0.0;
//    volatile FP32 Phase;
//    
//    real = 0;
//    imag = 0;
//    
//    for(ii = 0 ; ii < SAMPLE_POINT_SIZE ; ii++)
//    {
//        real += ((INT64S)(g_SinBuff[ii]) * (INT64S)(data[ii]));
//        imag += ((INT64S)(g_CosBuff[ii]) * (INT64S)(data[ii]));
//    }

//    real = real / (INT64S)SCALING;
//    imag = imag / (INT64S)SCALING;

//    Phase = GetPhase((FP32)real, (FP32)imag);

//    return Phase;
    return 0;
}

void InitGoertzel(void)
{
    INT16U ii;
    
//    for(ii = 0 ; ii < SAMPLE_POINT_SIZE ; ii++)
//    {
//      g_SinBuff[ii] = (INT32S)(sin(ii * OMEGA) * INT_COEF);
//      g_CosBuff[ii] = (INT32S)(cos(ii * OMEGA) * INT_COEF);
//    }
}
