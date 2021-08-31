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
#define PID_RATE_ROLL_INTEGRATION_LIMIT		150.0   //150
#define PID_RATE_PITCH_INTEGRATION_LIMIT	150.0   //150
#define PID_RATE_YAW_INTEGRATION_LIMIT		150.0   //150

/*角速度PID输出限幅（单位：油门值）*/
#define PID_RATE_ROLL_OUTPUT_LIMIT			500.0   //500
#define PID_RATE_PITCH_OUTPUT_LIMIT			500.0   //500
#define PID_RATE_YAW_OUTPUT_LIMIT			300.0   //300

/*------------------通用参数-------------------------*/

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
		[RATE_ROLL]   = {2600, 2300, 0},   //单环1600 1400 345   //串级 2600 2300 0
		[RATE_PITCH]  = {2600, 2300, 0},   //单环1600 1300 345
		[RATE_YAW]    = {2100, 1900, 100},   //单环1600 800 340
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
        目前解决办法：设置反向积分拦截门限，当控制器冲过目标设定值，并到达设定阈值时，直接清除当前积分
                    （前提是积分与当前过冲方向一致）
        副作用：有时快速向一个方向猛打杆，偶然触发清除，此时这次打杆会过冲的厉害（原本的消除过冲累积被清除了），
                大多数时候能正常累积反向抵消积分
        
        存在问题：
        1、如果快速打杆至两边边界，会严重过冲，PID参数暂时无法解决这个问题，可能是惯性引起的，
            或许需要遥控器低通来解决
        2、航向角顺逆时针不平衡，逆时针力矩更大
        
        调整思想：
        1、利用PD获得0度状态的稳定，将0度状态调制稳态误差很小（+-1.5°）基本上就差不多了。
            PD合适的标准还有就是快速打杆不会产生过大的过冲，且能快速响应
        2、利用I减小倾斜状态的稳态误差，这个误差来源于四轴的重力不平衡性质。尽量减少这个量就够了，并且保证
            不产生高频积分震荡即可
        -----------------------------以上都是单环的经验--------------------------------------
        串级调整思想：
        内环：
        1、P的调整思想为角速度能大概跟随目标速度全过程，且目标快速变化时，不会存在太大过冲。能够达到
        快速跟随即可
        2、I的思想为能够及时修正角度偏差且回中后积分消除过程不存在太大过程
        3、D的思想为极限打杆（如垂直起飞），回中立即停止运动，不会存在大幅度过冲。需配合油门限制使用
        */
	}
};

typedef struct
{
    struct
    {
        INT8U UpdateOriginYaw : 1;
        INT8U Reserve : 7;
    }Status;
    
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
    
    FP32 TakeOffYaw;       //起飞的朝向
    FP32 HoverYaw;         //当前机头朝向
}StabilizerInfo;

void StabilizerInit(void);
void StabilizerTask(void);
void UpdateSetpoint(FP32 SetRoll, FP32 SetPitch, FP32 SetYaw, FP32 Throttle);
void SetOriginYaw(void);

extern StabilizerInfo  g_StabiliCtrlMsg;

#endif


