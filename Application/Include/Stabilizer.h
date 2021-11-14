#ifndef STABILIZER_H
#define STABILIZER_H

#include "includes.h"
#include "PID.h"
#include "cpu.h"
#include "Attitude.h"
#include "Height.h"
#include "Motor.h"
#include "FlightMode.h"
#include "Predictor2.h"

/*--------------------外环-------------------------*/
/*角度PID输出限幅（单位：deg/s）*/
#define PID_ANGLE_ROLL_OUTPUT_LIMIT    		300.0
#define PID_ANGLE_PITCH_OUTPUT_LIMIT   		300.0
#define PID_ANGLE_YAW_OUTPUT_LIMIT     		150.0

/*--------------------内环-------------------------*/
/*角速度PID积分限幅（单位：deg/s）*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT		150.0   //150
#define PID_RATE_PITCH_INTEGRATION_LIMIT	150.0   //150
#define PID_RATE_YAW_INTEGRATION_LIMIT		100.0   //100

/*角速度PID输出限幅（单位：油门值）*/
#define PID_RATE_ROLL_OUTPUT_LIMIT			400.0   //400
#define PID_RATE_PITCH_OUTPUT_LIMIT			400.0   //400
#define PID_RATE_YAW_OUTPUT_LIMIT			400.0   //300

/*--------------------高度位置环----------------------*/
#define PID_POS_Z_OUTPUT_LIMIT              150.0   //cm/s

/*--------------------高度速度环----------------------*/
#define PID_VEL_Z_INTEGRATION_LIMIT         200.0   //cm/s
#define PID_VEL_Z_OUTPUT_LIMIT              400    //油门

#define PID_VEL_Z_LPF_CUTOFF_FREQ           15.0    //hz

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

#define POSITION_PID_RATE       MAIN_LOOP_RATE
#define POSITION_PID_DT         (1.0/POSITION_PID_RATE)

#define VELOCITY_PID_RATE       MAIN_LOOP_RATE
#define VELOCITY_PID_DT         (1.0/VELOCITY_PID_RATE)

#define MIN_THROTTLE_VALUE      0.10                    //默认在地面起飞时才会出现的最小油门量
#define MANUAL_THROTTLE_BASE    2800
#define MANUAL_THROTTLE_DIFF    800
#define MANUAL_THROTTLE_LANDING 3000

#define FIXEDF_THROTTLE_BASE    2700
#define FIXEDF_THROTTLE_DIFF    800

#define MAX_RISE_VELOCITY       150.0f      //cm/s
#define MAX_FALL_VELOCITY       110.0f      //cm/s  

typedef struct	
{
	INT8U Version;                  /*软件版本号*/
	PIDPara_t pid[PID_NUM];       /*PID参数*/
} ConfigPara_t;

typedef struct
{
    struct
    {
        INT8U UpdateOriginYaw : 1;
        INT8U FixedHeightIsInit : 1;
        INT8U UpdateFixedHeight : 1;
        INT8U Reserve : 6;
    }Status;
    
    FP32 SetRoll;
    FP32 SetPitch;
    FP32 SetYaw;
    FP32 SetThrottleRate;
    
    FP32 AngleOutRoll;
    FP32 AngleOutPitch;
    FP32 AngleOutYaw;
    FP32 HeightZOut;
    
    FP32 RateOutRoll;
    FP32 RateOutPitch;
    FP32 RateOutYaw;
    FP32 VelocityZOut;
    
    FP32 ThrottleBase;      //定高油门基础（根据遥控器等比例更新）
    FP32 ThrottleOut;       //油门输出
    
    FP32 TakeOffYaw;       //起飞的朝向
    FP32 HoverYaw;         //当前机头朝向
    
    FP32 TakeOffHeight;
    
}StabilizerInfo;

static ConfigPara_t ControlParaDefault=
{
	.Version = 00,		/*软件版本号*/

	.pid = 
	{
		[RATE_ROLL]   = {1800, 1850, 2},   //1800  1850   10      
		[RATE_PITCH]  = {1850, 1850, 2},   //1800  1850   10
		[RATE_YAW]    = {2600, 1000, 2},   //3500  1800   10
		[ANGLE_ROLL]  = {6000, 0, 0},
		[ANGLE_PITCH] = {6000, 0, 0},
		[ANGLE_YAW]   = {6000, 0, 0},
		[VELOCITY_Z]  = {3000, 500, 0},
		[POSITION_Z]  = {3000, 0, 0},
		[VELOCITY_XY] = {0, 0, 0},
		[POSITION_XY] = {0, 0, 0},
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
        横滚俯仰：
        1、P的调整思想为角速度能大概跟随目标速度全过程，且目标快速变化时，不会存在太大过冲。能够达到
        快速跟随即可
        2、I的思想为能够及时修正角度偏差且回中后积分消除过程不存在太大过冲（考虑用上放积分饱和策略）
        3、D的思想为极限打杆（如垂直起飞），回中立即停止运动，不会存在大幅度过冲。需配合油门限制使用
        
        航向：
        1、P会增大减小回中的油门量，以适量过冲的P为宜
        2、无论什么P，都无法解决打杆的偏差，也就是目标角速度始终不能等于实际角速度，但是P可以合理控制回中的力道
        3、D需要适量添加让回中曲线完美贴合目标值
        4、暂不适用I
        
        
        */
	}
};

void StabilizerInit(void);
void StabilizerTask(void);
void UpdateSetpoint(FP32 SetRoll, FP32 SetPitch, FP32 SetYaw, FP32 Throttle);
void SetOriginFlyPara(void);
BOOLEAN TakeoffCheck(void);

extern StabilizerInfo  g_StabiliCtrlMsg;

#endif


