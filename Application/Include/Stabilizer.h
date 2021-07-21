#ifndef STABILIZER_H
#define STABILIZER_H

#include "includes.h"
#include "PID.h"
#include "cpu.h"
#include "Attitude.h"
#include "Motor.h"

/*--------------------外环-------------------------*/

/*角度PID输出限幅（单位：deg/s）*/
#define PID_ANGLE_ROLL_OUTPUT_LIMIT    		300.0
#define PID_ANGLE_PITCH_OUTPUT_LIMIT   		300.0
#define PID_ANGLE_YAW_OUTPUT_LIMIT     		300.0

/*-----------------------------------------------*/


/*--------------------内环-------------------------*/

/*角速度PID积分限幅（单位：deg/s）*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT		150.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT	150.0
#define PID_RATE_YAW_INTEGRATION_LIMIT		100.0

/*角速度PID积分开始门限（单位：deg/s）*/
#define PID_RATE_ROLL_INTEGRATION_THRESHOLD		120.0
#define PID_RATE_PITCH_INTEGRATION_THRESHOLD	120.0
#define PID_RATE_YAW_INTEGRATION_THRESHOLD		120.0

/*角速度PID输出限幅（单位：油门值）*/
#define PID_RATE_ROLL_OUTPUT_LIMIT			350.0   //350
#define PID_RATE_PITCH_OUTPUT_LIMIT			350.0   //350
#define PID_RATE_YAW_OUTPUT_LIMIT			300.0

/*--------------------内环-------------------------*/

//角速度PID D项低通截止频率（单位Hz）
#define PID_RATE_LPF_CUTOFF_FREQ			80.0

#define MAIN_LOOP_RATE 			RATE_500_HZ				//主循环速率
#define MAIN_LOOP_DT			(1.0/MAIN_LOOP_RATE)	

#define ATTITUDE_ESTIMAT_RATE	RATE_500_HZ				//姿态解算速率
#define ATTITUDE_ESTIMAT_DT		(1.0/ATTITUDE_ESTIMAT_RATE)

#define RATE_PID_RATE			MAIN_LOOP_RATE 			//角速度环PID速率（和主循环速率一致）
#define RATE_PID_DT				(1.0/RATE_PID_RATE)

#define ANGLE_PID_RATE			ATTITUDE_ESTIMAT_RATE 	//角度环PID速率（和姿态解算速率一致）
#define ANGLE_PID_DT			(1.0/ANGLE_PID_RATE)

typedef struct	
{
	INT8U Version;                  /*软件版本号*/
	PIDPara_t pid[PID_NUM];       /*PID参数*/
} ConfigPara_t;

static ConfigPara_t ControlParaDefault=
{
	.Version = 00,		/*软件版本号*/

	.pid = 
	{
		[RATE_ROLL]   = {1020, 150, 240},   //手感还行，可能需要给遥控做低通
		[RATE_PITCH]  = {1020, 150, 240},
		[RATE_YAW]    = {0, 0, 0},
		[ANGLE_ROLL]  = {6000, 0, 0},
		[ANGLE_PITCH] = {6000, 0, 0},
		[ANGLE_YAW]   = {6000, 0, 0},
		[VELOCITY_Z]  = {0, 0, 0},
		[POSHOLD_Z]   = {0, 0, 0},
		[VELOCITY_XY] = {0, 0, 0},
		[POSHOLD_XY]  = {0, 0, 0},
	}
};

typedef struct
{
    FP32 SetRoll;
    FP32 SetPitch;
    FP32 SetYaw;

    FP32 AngleOutRoll;
    FP32 AngleOutPitch;
    FP32 AngleOutYaw;
    
    FP32 RateOutRoll;
    FP32 RateOutPitch;
    FP32 RateOutYaw;
    
    FP32 ThrustOut;
}StabilizerInfo;

void StabilizerInit(void);
void StabilizerTask(void);
void UpdateSetpoint(FP32 SetRoll, FP32 SetPitch, FP32 SetYaw, FP32 Throttle);

extern StabilizerInfo  g_StabiliCtrlMsg;

#endif


