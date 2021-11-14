#ifndef PID_H
#define PID_H

#include "includes.h"
#include "string.h"
#include "Mymath.h"

#define PID_UPDATE_RATE (1.0/ATTITUDE_UPDATE_RATE)

enum PIDIndex
{
	RATE_ROLL = 0,
	RATE_PITCH,
	RATE_YAW,
	ANGLE_ROLL,
	ANGLE_PITCH,
	ANGLE_YAW,
	VELOCITY_Z,     //Z轴速度环
	POSITION_Z,      //Z轴高度环
	VELOCITY_XY,
	POSITION_XY,
	PID_NUM    
};

typedef struct
{
	FP32 desired;		//< set point
	FP32 error;        //< error
	FP32 prevError;    //< previous error
    
	FP32 integ;        //< integral
	FP32 deriv;        //< derivative
    
	FP32 kp;           //< proportional gain
	FP32 ki;           //< integral gain
	FP32 kd;           //< derivative gain
    
	FP32 outP;         //< proportional output (debugging)
	FP32 outI;         //< integral output (debugging)
	FP32 outD;         //< derivative output (debugging)
    
	FP32 iLimit;       //< integral limit
    
	FP32 outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
	FP32 dt;           //< delta-time dt
	BOOLEAN enableDFilter; //< filter for D term enable flag
	BiquadFilterInfo dFilter;  //< filter for D term
}PIDInfo;

typedef struct
{
    FP32 Kp;
    FP32 Ki;
    FP32 Kd;
}PIDPara_t;

void PIDInit(PIDInfo *PID, FP32 Kp, FP32 Ki, FP32 Kd, FP32 ILimit, FP32 OutputLimit, FP32 dt, BOOLEAN EnDerivFilter, FP32 CutoffFreq);
FP32 PIDUpdate(PIDInfo* PID, FP32 error);
void PIDReset(PIDInfo* PID);
void PIDResetIntegral(PIDInfo* PID);
void PIDSetIntegral(PIDInfo* PID, FP32 integ);

extern PIDInfo g_PIDCtrlMsg[PID_NUM];

#endif // PID_H_INCLUDED
