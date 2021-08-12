#ifndef STABILIZER_H
#define STABILIZER_H

#include "includes.h"
#include "PID.h"
#include "cpu.h"
#include "Attitude.h"
#include "Motor.h"
#include "FlightMode.h"

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
#define PID_RATE_YAW_INTEGRATION_LIMIT		150.0

/*角速度PID积分开始门限（单位：deg/s）*/
#define PID_RATE_ROLL_INTEGRATION_THRESHOLD		90.0
#define PID_RATE_PITCH_INTEGRATION_THRESHOLD	90.0
#define PID_RATE_YAW_INTEGRATION_THRESHOLD		180.0

/*角速度PID积分停止门限（单位：deg/s）*/
#define PID_RATE_ROLL_INTEGRATION_STOP          5.0
#define PID_RATE_PITCH_INTEGRATION_STOP         5.0
#define PID_RATE_YAW_INTEGRATION_STOP           10.0

/*角速度PID输出限幅（单位：油门值）*/
#define PID_RATE_ROLL_OUTPUT_LIMIT			500.0   //350
#define PID_RATE_PITCH_OUTPUT_LIMIT			500.0   //350
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

#define LANDING_THROTTLE        3000

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
		[RATE_ROLL]   = {2700, 165, 375},   //1300 380 340  //2600-80er   2900-50er   3200-40er   3800-43er
		[RATE_PITCH]  = {2700, 0, 375},   //1450 380 320
		[RATE_YAW]    = {1400, 0, 340},   //1400 350 340
		[ANGLE_ROLL]  = {6000, 0, 0},
		[ANGLE_PITCH] = {6000, 0, 0},
		[ANGLE_YAW]   = {6000, 0, 0},
		[VELOCITY_Z]  = {0, 0, 0},
		[POSHOLD_Z]   = {0, 0, 0},
		[VELOCITY_XY] = {0, 0, 0},
		[POSHOLD_XY]  = {0, 0, 0},
        /*内环积分问题
        1、积分饱和：假如机器长期处于一边，机器累积积分满后，如果开启电机，会瞬间失衡
        2、积分消除慢：假如积分堆满了，但是此时err突然收拢，会导致积分来不及清除，造成一定角度的偏差
        
        调整思想：
        1、利用PD获得0度状态的稳定，将0度状态调制稳态误差很小（+-1.5°）基本上就差不多了。
            PD合适的标准还有就是快速打杆不会产生过大的过冲，且能快速响应
        2、利用I减小倾斜状态的稳态误差，这个误差来源于四轴的重力不平衡性质。尽量减少这个量就够了，并且保证
            不产生高频积分震荡即可，实测怎么都有点误差，无法保证全姿态角度跟随。
        */
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


