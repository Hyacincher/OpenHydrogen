#include "FFT.h"


/*
*函数名：FFT
*功  能：快速傅里叶变换
*输入参数：
    FP32 pr[n]	存放n个采样输入的实部，返回离散傅里叶变换的摸
    FP32 pi[n]	存放n个采样输入的虚部
    FP32 fr[n]	返回离散傅里叶变换的n个实部
    FP32 fi[n]	返回离散傅里叶变换的n个虚部
    int n	采样点数
*返回参数：None
*/
void FFT(FP32 *pr , FP32 *pi , INT32U n , FP32 *fr , FP32 *fi)
{
    INT32S it,m,is,i,j,nv,l0;
    FP32 k = (log(n) / log(2));
    FP32 p,q,s,vr,vi,poddr,poddi;

    for (it=0; it<=n-1; it++)  //将pr[0]和pi[0]循环赋值给fr[]和fi[]
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
    pr[1]=cos(p); //将w=e^-j2pi/n用欧拉公式表示
    pi[1]=-sin(p);

    for (i=2; i<=n-1; i++)  //计算pr[]
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
    for (l0=k-2; l0>=0; l0--) //蝴蝶操作
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
*函数名：GetAmplitude
*功  能：取得傅里叶变化结果的幅值
*输入参数：
    FP32 amplitude    储存傅里叶变换的幅值
    int    n            采样点数
    FP32 fr[n]	    离散傅里叶变换的n个实部
    FP32 fi[n]	    离散傅里叶变换的n个虚部
*返回参数：None
*/
FP32 GetAmplitude(FP32 fr, FP32 fi)
{
    return sqrt(fr*fr+fi*fi);  //幅值计算
}

/*
*函数名：GetPhase
*功  能：取得傅里叶变化结果的相位
*输入参数：
    FP32 phase        储存傅里叶变换的相位
    int    n            采样点数
    FP32 fr[n]	    离散傅里叶变换的n个实部
    FP32 fi[n]	    离散傅里叶变换的n个虚部
*返回参数：None
*/
FP32 GetPhase(FP32 fr, FP32 fi)
{
    FP32 phase;
    
    //phase = atan(fi/fr);  
    phase = atan2(fi , fr);  
    
    return phase;
}


/*
**函数信息      FindMaxMag
**功能描述      寻找最大的幅值点
**输入参数      Mag     幅值数组
                Len     数组长度
**输出参数      最大幅值位置
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
*函数名：apFFT
*功  能：全相位傅里叶变换
*输入参数：
        FP32 *pr        实部输入
        FP32 *pi        虚部输入
        INT32U n        输出点数（输入点数是输出的两倍）
        FP32 *fr        输出实部
        FP32 *fi        输出实部
*返回参数：None
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
**函数信息      CalculateDiffPhase
**功能描述      计算初末相位差
**输入参数      StartPhase      初相位
                EndPhase        末相位
**输出参数      相位差
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
**函数信息      CalculateAvePhase
**功能描述      计算相位的平均
**输入参数      Phase       相位数组
                Len         数组长度
                AveMode     平滑方式
                    1直接平均
                    2去掉最高最低求平均
                    3去掉最高最低取中值，然后除开最高最低值进行低通滤波
**输出参数      相位平均
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
    if((Pos == 0) || (Neg == 0))//全为正或负，数据不在临界，或者就在0附近
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
    
    if(AveMode == 0)//直接平均
    {
        Average /= Len;        
    }
    else if(AveMode == 1)//去除最高最低平均
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
    else if(AveMode == 2)//取中值并低通(排除最高最低)
    {
        Average = 0;
        BubbleSortFP32(Phase, Len);
        
        if((Len % 2) == 0)//双数
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
**函数信息      CalculateDistance
**功能描述      计算距离
**输入参数      Freq        测尺频率
                DisPhase    相位差
**输出参数      距离
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
