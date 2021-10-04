#include "Stabilizer.h"

StabilizerInfo  g_StabiliCtrlMsg;

static void ThrottleControl(void);
static void AttitudeControl(void);

static void StabilizerUpdate(void);

static void FixedHeightCtrl(void);
static void HeadLessCtrl(void);
static void LeadingCtrl(void);
static void HeadDirectionCal(void);

extern FP32 GetThrottleRate(void);

void StabilizerInit(void)
{
    INT16U ii;
    PIDPara_t PIDZoom[PID_NUM];

    MotorInit();
    
    memset(&g_StabiliCtrlMsg, 0, sizeof(g_StabiliCtrlMsg));
    
    for(ii = 0 ; ii < PID_NUM ; ii++)
    {
		PIDZoom[ii].Kp = ControlParaDefault.pid[ii].Kp / 1000.0f;
		PIDZoom[ii].Ki = ControlParaDefault.pid[ii].Ki / 1000.0f;
		PIDZoom[ii].Kd = ControlParaDefault.pid[ii].Kd / 1000.0f;
    }
    
    //角度PID（roll\pitch\yaw）
	PIDInit(&g_PIDCtrlMsg[ANGLE_ROLL], PIDZoom[ANGLE_ROLL].Kp, PIDZoom[ANGLE_ROLL].Ki, PIDZoom[ANGLE_ROLL].Kd, 
		0, PID_ANGLE_ROLL_OUTPUT_LIMIT, ANGLE_PID_DT, 0, 0);
	PIDInit(&g_PIDCtrlMsg[ANGLE_PITCH], PIDZoom[ANGLE_PITCH].Kp, PIDZoom[ANGLE_PITCH].Ki, PIDZoom[ANGLE_PITCH].Kd, 
		0, PID_ANGLE_PITCH_OUTPUT_LIMIT, ANGLE_PID_DT, 0, 0);
    PIDInit(&g_PIDCtrlMsg[ANGLE_YAW], PIDZoom[ANGLE_YAW].Kp, PIDZoom[ANGLE_YAW].Ki, PIDZoom[ANGLE_YAW].Kd, 
		0, PID_ANGLE_YAW_OUTPUT_LIMIT, ANGLE_PID_DT, 0, 0);
    
	//角速度PID（roll\pitch\yaw）
	PIDInit(&g_PIDCtrlMsg[RATE_ROLL], PIDZoom[RATE_ROLL].Kp, PIDZoom[RATE_ROLL].Ki, PIDZoom[RATE_ROLL].Kd, 
		PID_RATE_ROLL_INTEGRATION_LIMIT, PID_RATE_ROLL_OUTPUT_LIMIT, RATE_PID_DT, 1, PID_RATE_LPF_CUTOFF_FREQ);
    
	PIDInit(&g_PIDCtrlMsg[RATE_PITCH], PIDZoom[RATE_PITCH].Kp, PIDZoom[RATE_PITCH].Ki, PIDZoom[RATE_PITCH].Kd, 
		PID_RATE_PITCH_INTEGRATION_LIMIT, PID_RATE_PITCH_OUTPUT_LIMIT, RATE_PID_DT, 1, PID_RATE_LPF_CUTOFF_FREQ);
    
    PIDInit(&g_PIDCtrlMsg[RATE_YAW], PIDZoom[RATE_YAW].Kp, PIDZoom[RATE_YAW].Ki, PIDZoom[RATE_YAW].Kd, 
		PID_RATE_YAW_INTEGRATION_LIMIT, PID_RATE_YAW_OUTPUT_LIMIT, RATE_PID_DT, 1, PID_RATE_LPF_CUTOFF_FREQ);
}

void StabilizerTask(void)
{
    static INT64U s_SystemTime = 0;
    static INT8U s_StabilizerStage = 0;
    
    switch(s_StabilizerStage)
    {
        case 0:
            s_SystemTime = g_SysTickTime;
            
            ThrottleControl();
        
            AttitudeControl();
        
            StabilizerUpdate();
        
            s_StabilizerStage++;
            break;
        case 1:
            if(g_SysTickTime - s_SystemTime >= 2)
            {
                s_StabilizerStage++;
            }
            break;
        case 2:
            s_StabilizerStage = 0;
            break;
    }
}

//更新姿态设定值（degree）
void UpdateSetpoint(FP32 SetRoll, FP32 SetPitch, FP32 SetYaw, FP32 Throttle)
{
    g_StabiliCtrlMsg.SetRoll = SetRoll;
    g_StabiliCtrlMsg.SetPitch = SetPitch;
    g_StabiliCtrlMsg.SetYaw = SetYaw;
    g_StabiliCtrlMsg.SetThrottleRate = Throttle;
}

void SetOriginYaw(void)
{
    g_StabiliCtrlMsg.TakeOffYaw = g_AttitudeCtrlMsg.Yaw;
    g_StabiliCtrlMsg.HoverYaw = g_AttitudeCtrlMsg.Yaw;
}

static void ThrottleControl(void)
{
    switch(g_FlightModeCtrlMsg.ThrottleMode)
    {
        case UserManual:
            g_StabiliCtrlMsg.ThrottleOut = (g_StabiliCtrlMsg.SetThrottleRate * MANUAL_THROTTLE_DIFF) + MANUAL_THROTTLE_BASE;
            break;
        case FixedHeight:
            
            g_StabiliCtrlMsg.ThrottleOut = (g_StabiliCtrlMsg.SetThrottleRate * MANUAL_THROTTLE_DIFF) + MANUAL_THROTTLE_BASE;
            break;
        case AutoTakeoff:
            g_StabiliCtrlMsg.ThrottleOut = (g_StabiliCtrlMsg.SetThrottleRate * MANUAL_THROTTLE_DIFF) + MANUAL_THROTTLE_BASE;
            break;
        case AutoLanding:
            g_StabiliCtrlMsg.ThrottleOut = MANUAL_THROTTLE_LANDING;
            break;
        case ThroStandby:
            g_StabiliCtrlMsg.ThrottleOut = 0;
            break;
        default:
            g_FlightModeCtrlMsg.ThrottleMode = ThroStandby;
            g_StabiliCtrlMsg.ThrottleOut = 0;
            break;
    }
}

static void AttitudeControl(void)
{
    switch(g_FlightModeCtrlMsg.DirectionMode)
    {
        case HeadDirection:
            HeadDirectionCal();
            break;
        case HeadLess:
            HeadLessCtrl();
            break;
        case FixedPoint:
            HeadDirectionCal();
            break;
        case PointCruise:
            HeadDirectionCal();
            break;
        case DirecStandBy:
            g_StabiliCtrlMsg.RateOutRoll = 0;
            g_StabiliCtrlMsg.RateOutPitch = 0;
            g_StabiliCtrlMsg.RateOutYaw = 0;
            break;
        default:
            g_FlightModeCtrlMsg.DirectionMode = DirecStandBy;
            g_StabiliCtrlMsg.RateOutRoll = 0;
            g_StabiliCtrlMsg.RateOutPitch = 0;
            g_StabiliCtrlMsg.RateOutYaw = 0;
            break;
    }
}

static void StabilizerUpdate(void)
{
    FP32 MotorOut1,MotorOut2,MotorOut3,MotorOut4;
    INT16U ii;
    
    MotorOut1 = g_StabiliCtrlMsg.ThrottleOut + g_StabiliCtrlMsg.RateOutRoll - g_StabiliCtrlMsg.RateOutPitch - g_StabiliCtrlMsg.RateOutYaw;
    MotorOut2 = g_StabiliCtrlMsg.ThrottleOut + g_StabiliCtrlMsg.RateOutRoll + g_StabiliCtrlMsg.RateOutPitch + g_StabiliCtrlMsg.RateOutYaw;
    MotorOut3 = g_StabiliCtrlMsg.ThrottleOut - g_StabiliCtrlMsg.RateOutRoll + g_StabiliCtrlMsg.RateOutPitch - g_StabiliCtrlMsg.RateOutYaw;
    MotorOut4 = g_StabiliCtrlMsg.ThrottleOut - g_StabiliCtrlMsg.RateOutRoll - g_StabiliCtrlMsg.RateOutPitch + g_StabiliCtrlMsg.RateOutYaw;

    MotorOut1 = MyConstrainF(MotorOut1, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
    MotorOut2 = MyConstrainF(MotorOut2, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
    MotorOut3 = MyConstrainF(MotorOut3, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
    MotorOut4 = MyConstrainF(MotorOut4, MOTOR_OUT_MIN, MOTOR_OUT_MAX);

    MotorSetDuty(Motor1, (INT16U)MotorOut1);
    MotorSetDuty(Motor2, (INT16U)MotorOut2);
    MotorSetDuty(Motor3, (INT16U)MotorOut3);
    MotorSetDuty(Motor4, (INT16U)MotorOut4);
}

static void FixedHeightCtrl(void)
{
    FP32 Error;
    
    if(g_StabiliCtrlMsg.Status.FixedHeightIsInit == 0)
    {
        g_StabiliCtrlMsg.Status.FixedHeightIsInit = 1;
        
        if(g_StabiliCtrlMsg.SetThrottleRate < MIN_THROTTLE_VALUE)   //小油门量
        {
            PIDSetIntegral(&g_PIDCtrlMsg[VELOCITY_Z], -400);        //利用积分让速度环输出缓慢收敛，防止油门突然过冲
        }
        else
        {
            PIDResetIntegral(&g_PIDCtrlMsg[VELOCITY_Z]);
        }
        
        g_StabiliCtrlMsg.TakeOffHeight = g_HeightCtrlMsg.Altitude;
    }
    
    if(g_StabiliCtrlMsg.SetThrottleRate == 0.5)
    {//摇杆回中
        if(g_StabiliCtrlMsg.Status.UpdateFixedHeight)
        {//高度锁定，更新锁定高度
            g_StabiliCtrlMsg.Status.UpdateFixedHeight = 0;
            g_StabiliCtrlMsg.TakeOffHeight = g_HeightCtrlMsg.Altitude;
        }
        Error = g_StabiliCtrlMsg.TakeOffHeight - g_HeightCtrlMsg.Altitude;
        g_StabiliCtrlMsg.HeightZOut = PIDUpdate(&g_PIDCtrlMsg[HEIGHT_Z], Error);
    }
    else if(g_StabiliCtrlMsg.SetThrottleRate < 0.5)
    {
        //下降
        g_StabiliCtrlMsg.HeightZOut = (g_StabiliCtrlMsg.SetThrottleRate * 2) * MAX_FALL_VELOCITY;
        g_StabiliCtrlMsg.Status.UpdateFixedHeight = 1;
    }
    else if(g_StabiliCtrlMsg.SetThrottleRate > 0.5)
    {
        //上升
        g_StabiliCtrlMsg.HeightZOut = ((g_StabiliCtrlMsg.SetThrottleRate - 0.5) * 2) * MAX_RISE_VELOCITY;
        g_StabiliCtrlMsg.Status.UpdateFixedHeight = 1;
    }
    
    //g_StabiliCtrlMsg.VelocityZOut = PIDUpdate(&g_PIDCtrlMsg[VELOCITY_Z], Error);
}

static void HeadLessCtrl(void)
{
    FP32 Error;
    INT16U ii;
    
    //求取横滚、俯仰向量分量
    FP32 RadDiff = -(g_AttitudeCtrlMsg.Yaw - g_StabiliCtrlMsg.TakeOffYaw) * PI / 180.0f;    //取负号是因为我的电机反着装的
    FP32 CosDiff = MyCosApprox(RadDiff);
    FP32 SinDiff = MySinApprox(RadDiff);
    FP32 SetPitch = (g_StabiliCtrlMsg.SetPitch * CosDiff) + (g_StabiliCtrlMsg.SetRoll * SinDiff);
    FP32 SetRoll = (g_StabiliCtrlMsg.SetRoll * CosDiff) - (g_StabiliCtrlMsg.SetPitch * SinDiff);

    //外环（角度）
    g_StabiliCtrlMsg.AngleOutRoll = PIDUpdate(&g_PIDCtrlMsg[ANGLE_ROLL], SetRoll - g_AttitudeCtrlMsg.Roll);
    g_StabiliCtrlMsg.AngleOutPitch = PIDUpdate(&g_PIDCtrlMsg[ANGLE_PITCH], SetPitch - g_AttitudeCtrlMsg.Pitch);
    if(g_StabiliCtrlMsg.SetYaw == 0)
    {//摇杆回中
        if(g_StabiliCtrlMsg.Status.UpdateOriginYaw)
        {//从偏航锁定，更新目标航向角
            g_StabiliCtrlMsg.Status.UpdateOriginYaw = 0;
            g_StabiliCtrlMsg.HoverYaw = g_AttitudeCtrlMsg.Yaw;
        }
        Error = g_StabiliCtrlMsg.HoverYaw - g_AttitudeCtrlMsg.Yaw;
        if (Error >= +180)Error -= 360;
        if (Error <= -180)Error += 360;
        g_StabiliCtrlMsg.AngleOutYaw = PIDUpdate(&g_PIDCtrlMsg[ANGLE_YAW], Error);
    }
    else
    {//偏航
        g_StabiliCtrlMsg.AngleOutYaw = g_StabiliCtrlMsg.SetYaw;
        g_StabiliCtrlMsg.Status.UpdateOriginYaw = 1;
    }

    //内环（角速度）
    if(GetThrottleRate() < MIN_THROTTLE_VALUE)
    {//地面油门量清除积分
        for(ii = RATE_ROLL ; ii <= RATE_YAW ; ii++)
        {
            PIDResetIntegral(&g_PIDCtrlMsg[ii]);
        }
    }
    g_StabiliCtrlMsg.RateOutRoll = PIDUpdate(&g_PIDCtrlMsg[RATE_ROLL], g_StabiliCtrlMsg.AngleOutRoll - RADIANS_TO_DEGREES(g_AttitudeCtrlMsg.NormailGyro[IMUAxisX]));
    g_StabiliCtrlMsg.RateOutPitch = PIDUpdate(&g_PIDCtrlMsg[RATE_PITCH], g_StabiliCtrlMsg.AngleOutPitch - RADIANS_TO_DEGREES(g_AttitudeCtrlMsg.NormailGyro[IMUAxisY]));
    g_StabiliCtrlMsg.RateOutYaw = PIDUpdate(&g_PIDCtrlMsg[RATE_YAW], g_StabiliCtrlMsg.AngleOutYaw - RADIANS_TO_DEGREES(g_AttitudeCtrlMsg.NormailGyro[IMUAxisZ]));
}

static void HeadDirectionCal(void)
{
    FP32 Error;
    INT16U ii;
    
    //外环（角度）
    g_StabiliCtrlMsg.AngleOutRoll = PIDUpdate(&g_PIDCtrlMsg[ANGLE_ROLL], g_StabiliCtrlMsg.SetRoll - g_AttitudeCtrlMsg.Roll);
    g_StabiliCtrlMsg.AngleOutPitch = PIDUpdate(&g_PIDCtrlMsg[ANGLE_PITCH], g_StabiliCtrlMsg.SetPitch - g_AttitudeCtrlMsg.Pitch);
    if(g_StabiliCtrlMsg.SetYaw == 0)
    {//摇杆回中
        if(g_StabiliCtrlMsg.Status.UpdateOriginYaw)
        {//从偏航锁定，更新目标航向角
            g_StabiliCtrlMsg.Status.UpdateOriginYaw = 0;
            g_StabiliCtrlMsg.HoverYaw = g_AttitudeCtrlMsg.Yaw;
        }
        Error = g_StabiliCtrlMsg.HoverYaw - g_AttitudeCtrlMsg.Yaw;
        if (Error >= +180)Error -= 360;
        if (Error <= -180)Error += 360;
        g_StabiliCtrlMsg.AngleOutYaw = PIDUpdate(&g_PIDCtrlMsg[ANGLE_YAW], Error);
    }
    else
    {//偏航
        g_StabiliCtrlMsg.AngleOutYaw = g_StabiliCtrlMsg.SetYaw;
        g_StabiliCtrlMsg.Status.UpdateOriginYaw = 1;
    }

    //内环（角速度）
    if(GetThrottleRate() < MIN_THROTTLE_VALUE)
    {//地面油门量清除积分
        for(ii = RATE_ROLL ; ii <= RATE_YAW ; ii++)
        {
            PIDResetIntegral(&g_PIDCtrlMsg[ii]);
        }
    }
    g_StabiliCtrlMsg.RateOutRoll = PIDUpdate(&g_PIDCtrlMsg[RATE_ROLL], g_StabiliCtrlMsg.AngleOutRoll - RADIANS_TO_DEGREES(g_AttitudeCtrlMsg.NormailGyro[IMUAxisX]));
    g_StabiliCtrlMsg.RateOutPitch = PIDUpdate(&g_PIDCtrlMsg[RATE_PITCH], g_StabiliCtrlMsg.AngleOutPitch - RADIANS_TO_DEGREES(g_AttitudeCtrlMsg.NormailGyro[IMUAxisY]));
    g_StabiliCtrlMsg.RateOutYaw = PIDUpdate(&g_PIDCtrlMsg[RATE_YAW], g_StabiliCtrlMsg.AngleOutYaw - RADIANS_TO_DEGREES(g_AttitudeCtrlMsg.NormailGyro[IMUAxisZ])); 
}




