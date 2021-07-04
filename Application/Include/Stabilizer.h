#ifndef STABILIZER_H
#define STABILIZER_H

#include "includes.h"
#include "PID.h"
#include "cpu.h"
#include "Attitude.h"
#include "Motor.h"

/*���ٶ�PID�����޷�����λ��deg/s��*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT		200.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT	200.0
#define PID_RATE_YAW_INTEGRATION_LIMIT		100.0

/*���ٶ�PID����޷�����λ������ֵ��*/
#define PID_RATE_ROLL_OUTPUT_LIMIT			500.0
#define PID_RATE_PITCH_OUTPUT_LIMIT			500.0
#define PID_RATE_YAW_OUTPUT_LIMIT			300.0

/*�Ƕ�PID����޷�����λ��deg/s��*/
#define PID_ANGLE_ROLL_OUTPUT_LIMIT    		300.0
#define PID_ANGLE_PITCH_OUTPUT_LIMIT   		300.0
#define PID_ANGLE_YAW_OUTPUT_LIMIT     		150.0

//Z���ٶ�PID�����޷�����λcm/s��
#define PID_VZ_INTEGRATION_LIMIT 			500.0
//Z���ٶ�PID����޷�����λ����ֵ��
#define PID_VZ_OUTPUT_LIMIT					800.0

//XY���ٶ�PID�����޷�����λcm/s��
#define PID_VXY_INTEGRATION_LIMIT			300.0
//XY���ٶ�PID����޷�����λdeg��
#define PID_VXY_OUTPUT_LIMIT				30.0

//XYZλ��PID����޷�����λcm/s��
#define PID_POS_OUTPUT_LIMIT				150.0

//���ٶ�PID D���ͨ��ֹƵ�ʣ���λHz��
#define PID_RATE_LPF_CUTOFF_FREQ			80.0

//Z���ٶ�PID D���ͨ��ֹƵ�ʣ���λHz��
#define PID_VZ_LPF_CUTOFF_FREQ				15.0

#define MAIN_LOOP_RATE 			RATE_500_HZ				//��ѭ������
#define MAIN_LOOP_DT			(1.0/MAIN_LOOP_RATE)	

#define ATTITUDE_ESTIMAT_RATE	RATE_500_HZ				//��̬��������
#define ATTITUDE_ESTIMAT_DT		(1.0/ATTITUDE_ESTIMAT_RATE)

#define RATE_PID_RATE			MAIN_LOOP_RATE 			//���ٶȻ�PID���ʣ�����ѭ������һ�£�
#define RATE_PID_DT				(1.0/RATE_PID_RATE)

#define ANGLE_PID_RATE			ATTITUDE_ESTIMAT_RATE 	//�ǶȻ�PID���ʣ�����̬��������һ�£�
#define ANGLE_PID_DT			(1.0/ANGLE_PID_RATE)

typedef struct	
{
	INT8U Version;                  /*����汾��*/
	PIDPara_t pid[PID_NUM];       /*PID����*/
} ConfigPara_t;

static ConfigPara_t ControlParaDefault=
{
	.Version = 00,		/*����汾��*/

	.pid = 
	{
		[RATE_ROLL]   = {85, 900, 18},
		[RATE_PITCH]  = {90, 1000, 18},
		[RATE_YAW]    = {120, 800, 0},
		[ANGLE_ROLL]  = {600, 0, 0},
		[ANGLE_PITCH] = {600, 0, 0},
		[ANGLE_YAW]   = {600, 0, 0},
		[VELOCITY_Z]  = {150, 200, 50},
		[POSHOLD_Z]   = {45, 0, 0},
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

extern StabilizerInfo  g_StabiliCtrlMsg;
#endif


