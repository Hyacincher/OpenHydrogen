#include "PID.h"

PIDInfo g_PIDCtrlMsg[PID_NUM];

void PIDInit(PIDInfo *PID, FP32 Kp, FP32 Ki, FP32 Kd, FP32 ILimit, FP32 IThreshold, FP32 IStop, FP32 OutputLimit, FP32 dt, BOOLEAN EnDerivFilter, FP32 CutoffFreq)
{
    memset(PID, 0, sizeof(PIDInfo));
    
	PID->desired   = 0;
	PID->error     = 0;
	PID->prevError = 0;	
	PID->integ     = 0;
	PID->deriv     = 0;
	PID->kp 	   = Kp;
	PID->ki        = Ki;
	PID->kd        = Kd;
	PID->outP      = 0;
	PID->outI      = 0;
	PID->outD      = 0;
	PID->iLimit    = ILimit;
	PID->outputLimit = OutputLimit;
	PID->dt        = dt;
	PID->enableDFilter = EnDerivFilter;
    PID->iThreshold = IThreshold;
    PID->iStop = IStop;
	if (PID->enableDFilter)
	{
		BiquadFilterInitLPF(&PID->dFilter, (1.0f/dt), CutoffFreq);
	}
}

FP32 PIDUpdate(PIDInfo* PID, FP32 error)//error = desired - measured 
{
	FP32 output = 0.0f;
	
	PID->error = error;

    if(MyFP32Abs(error) < PID->iThreshold)
    {//利用积分累计门限防止积分饱和效应
        PID->integ += PID->error * PID->dt;
    }
    else
    {
        PID->integ = 0;
    }

    if(MyFP32Abs(error) < PID->iStop)
    {//err达到目标值附近小范围取消积分累计
        PID->integ = 0;
    }
    
	//积分限幅
	if (PID->iLimit != 0)
	{
        PID->integ = MyConstrainF(PID->integ, -PID->iLimit, PID->iLimit);
	}
    
	PID->deriv = (PID->error - PID->prevError) / PID->dt;
	if (PID->enableDFilter)
	{
		PID->deriv = BiquadLPFFilter(&PID->dFilter, PID->deriv);
	}
	
	PID->outP = PID->kp * PID->error;
	PID->outI = PID->ki * PID->integ;
	PID->outD = PID->kd * PID->deriv;

	output = PID->outP + PID->outI + PID->outD;
    
	//输出限幅
	if (PID->outputLimit != 0)
	{
		output = MyConstrainF(output, -PID->outputLimit, PID->outputLimit);
	}

    
	PID->prevError = PID->error;

	return output;
}

void PIDReset(PIDInfo* PID)
{
	PID->error = 0;
	PID->prevError = 0;
	PID->integ = 0;
	PID->deriv = 0;
}

void PIDResetIntegral(PIDInfo* PID)
{
	PID->integ = 0;
}

void PIDSetIntegral(PIDInfo* PID, FP32 integ)
{
	PID->integ = integ;
}