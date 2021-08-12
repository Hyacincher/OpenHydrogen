#include "Stabilizer.h"

StabilizerInfo  g_StabiliCtrlMsg;

static void ManualCtrlThread(void);
static void LandingThread(void);

void StabilizerInit(void)
{
    INT16U ii;
    PIDPara_t PIDZoom[PID_NUM];

    MotorInit();
    
    memset(&g_StabiliCtrlMsg, 0, sizeof(g_StabiliCtrlMsg));
    
    g_StabiliCtrlMsg.ThrustOut = 2500;  //调试用
    
    for(ii = 0 ; ii < PID_NUM ; ii++)
    {
		PIDZoom[ii].Kp = ControlParaDefault.pid[ii].Kp / 1000.0f;
		PIDZoom[ii].Ki = ControlParaDefault.pid[ii].Ki / 1000.0f;
		PIDZoom[ii].Kd = ControlParaDefault.pid[ii].Kd / 1000.0f;
    }
    
    //角度PID（roll\pitch\yaw）
	PIDInit(&g_PIDCtrlMsg[ANGLE_ROLL], PIDZoom[ANGLE_ROLL].Kp, PIDZoom[ANGLE_ROLL].Ki, PIDZoom[ANGLE_ROLL].Kd, 
		0, 0, 0, PID_ANGLE_ROLL_OUTPUT_LIMIT, ANGLE_PID_DT, 0, 0);
	PIDInit(&g_PIDCtrlMsg[ANGLE_PITCH], PIDZoom[ANGLE_PITCH].Kp, PIDZoom[ANGLE_PITCH].Ki, PIDZoom[ANGLE_PITCH].Kd, 
		0, 0, 0, PID_ANGLE_PITCH_OUTPUT_LIMIT, ANGLE_PID_DT, 0, 0);
    PIDInit(&g_PIDCtrlMsg[ANGLE_YAW], PIDZoom[ANGLE_YAW].Kp, PIDZoom[ANGLE_YAW].Ki, PIDZoom[ANGLE_YAW].Kd, 
		0, 0, 0, PID_ANGLE_YAW_OUTPUT_LIMIT, ANGLE_PID_DT, 0, 0);
    
	//角速度PID（roll\pitch\yaw）
	PIDInit(&g_PIDCtrlMsg[RATE_ROLL], PIDZoom[RATE_ROLL].Kp, PIDZoom[RATE_ROLL].Ki, PIDZoom[RATE_ROLL].Kd, 
		PID_RATE_ROLL_INTEGRATION_LIMIT, PID_RATE_ROLL_INTEGRATION_THRESHOLD, PID_RATE_ROLL_INTEGRATION_STOP, 
        PID_RATE_ROLL_OUTPUT_LIMIT, RATE_PID_DT, 1, PID_RATE_LPF_CUTOFF_FREQ);
    
	PIDInit(&g_PIDCtrlMsg[RATE_PITCH], PIDZoom[RATE_PITCH].Kp, PIDZoom[RATE_PITCH].Ki, PIDZoom[RATE_PITCH].Kd, 
		PID_RATE_PITCH_INTEGRATION_LIMIT, PID_RATE_PITCH_INTEGRATION_THRESHOLD, PID_RATE_PITCH_INTEGRATION_STOP, 
        PID_RATE_PITCH_OUTPUT_LIMIT, RATE_PID_DT, 1, PID_RATE_LPF_CUTOFF_FREQ);
    
    PIDInit(&g_PIDCtrlMsg[RATE_YAW], PIDZoom[RATE_YAW].Kp, PIDZoom[RATE_YAW].Ki, PIDZoom[RATE_YAW].Kd, 
		PID_RATE_YAW_INTEGRATION_LIMIT, PID_RATE_YAW_INTEGRATION_THRESHOLD, PID_RATE_YAW_INTEGRATION_STOP, 
        PID_RATE_YAW_OUTPUT_LIMIT, RATE_PID_DT, 1, PID_RATE_LPF_CUTOFF_FREQ);
}

void StabilizerTask(void)
{
    static INT64U s_SystemTime = 0;
    static INT8U s_StabilizerStage = 0;
    
    switch(s_StabilizerStage)
    {
        case 0:
            s_SystemTime = g_SysTickTime;

            //外环（角度）
            g_StabiliCtrlMsg.AngleOutRoll = PIDUpdate(&g_PIDCtrlMsg[ANGLE_ROLL], g_StabiliCtrlMsg.SetRoll - g_AttitudeCtrlMsg.Roll);
            g_StabiliCtrlMsg.AngleOutPitch = PIDUpdate(&g_PIDCtrlMsg[ANGLE_PITCH], g_StabiliCtrlMsg.SetPitch - g_AttitudeCtrlMsg.Pitch);
            g_StabiliCtrlMsg.AngleOutYaw = PIDUpdate(&g_PIDCtrlMsg[ANGLE_YAW], g_StabiliCtrlMsg.SetYaw - g_AttitudeCtrlMsg.Yaw);
        
            //内环（角速度）
            g_StabiliCtrlMsg.RateOutRoll = PIDUpdate(&g_PIDCtrlMsg[RATE_ROLL], g_StabiliCtrlMsg.AngleOutRoll - g_AttitudeCtrlMsg.NormailGyro[IMU_AXIS_X]);
            g_StabiliCtrlMsg.RateOutPitch = PIDUpdate(&g_PIDCtrlMsg[RATE_PITCH], g_StabiliCtrlMsg.AngleOutPitch - g_AttitudeCtrlMsg.NormailGyro[IMU_AXIS_Y]);
            g_StabiliCtrlMsg.RateOutYaw = PIDUpdate(&g_PIDCtrlMsg[RATE_YAW], g_StabiliCtrlMsg.AngleOutYaw - g_AttitudeCtrlMsg.NormailGyro[IMU_AXIS_Z]);
            
            switch(g_FlightModeCtrlMsg.FlightMode)
            {
                case ManualCtrl:
                    ManualCtrlThread();
                    break;
                case FixedAltitude:
                    break;
                case FixedPoint:
                    break;
                case AutoReturn:
                    break;
                case Landing:
                    LandingThread();
                    break;
                case StandBy:
                    MotorLock();
                    break;
                default:
                    g_FlightModeCtrlMsg.FlightMode = StandBy;
                    break;
            }
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
    g_StabiliCtrlMsg.ThrustOut = Throttle;
}

static void ManualCtrlThread(void)
{
    FP32 MotorOut1,MotorOut2,MotorOut3,MotorOut4;
    
    if(GetMotorUnLock())
    {
        //解锁状态
        MotorOut1 = g_StabiliCtrlMsg.ThrustOut + g_StabiliCtrlMsg.RateOutRoll - g_StabiliCtrlMsg.RateOutPitch - g_StabiliCtrlMsg.RateOutYaw;
        MotorOut2 = g_StabiliCtrlMsg.ThrustOut + g_StabiliCtrlMsg.RateOutRoll + g_StabiliCtrlMsg.RateOutPitch + g_StabiliCtrlMsg.RateOutYaw;
        MotorOut3 = g_StabiliCtrlMsg.ThrustOut - g_StabiliCtrlMsg.RateOutRoll + g_StabiliCtrlMsg.RateOutPitch - g_StabiliCtrlMsg.RateOutYaw;
        MotorOut4 = g_StabiliCtrlMsg.ThrustOut - g_StabiliCtrlMsg.RateOutRoll - g_StabiliCtrlMsg.RateOutPitch + g_StabiliCtrlMsg.RateOutYaw;
    
        MotorOut1 = MyConstrainF(MotorOut1, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
        MotorOut2 = MyConstrainF(MotorOut2, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
        MotorOut3 = MyConstrainF(MotorOut3, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
        MotorOut4 = MyConstrainF(MotorOut4, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
    }
    else
    {
        MotorOut1 = MOTOR_OUT_MIN;
        MotorOut2 = MOTOR_OUT_MIN;
        MotorOut3 = MOTOR_OUT_MIN;
        MotorOut4 = MOTOR_OUT_MIN;
    }

    MotorSetDuty(Motor1, (INT16U)MotorOut1);
    MotorSetDuty(Motor2, (INT16U)MotorOut2);
    MotorSetDuty(Motor3, (INT16U)MotorOut3);
    MotorSetDuty(Motor4, (INT16U)MotorOut4);    
}

static void LandingThread(void)
{
    FP32 MotorOut1,MotorOut2,MotorOut3,MotorOut4;
    
    if(GetMotorUnLock())
    {
        //解锁状态
        MotorOut1 = LANDING_THROTTLE + g_StabiliCtrlMsg.RateOutRoll - g_StabiliCtrlMsg.RateOutPitch - g_StabiliCtrlMsg.RateOutYaw;
        MotorOut2 = LANDING_THROTTLE + g_StabiliCtrlMsg.RateOutRoll + g_StabiliCtrlMsg.RateOutPitch + g_StabiliCtrlMsg.RateOutYaw;
        MotorOut3 = LANDING_THROTTLE - g_StabiliCtrlMsg.RateOutRoll + g_StabiliCtrlMsg.RateOutPitch - g_StabiliCtrlMsg.RateOutYaw;
        MotorOut4 = LANDING_THROTTLE - g_StabiliCtrlMsg.RateOutRoll - g_StabiliCtrlMsg.RateOutPitch + g_StabiliCtrlMsg.RateOutYaw;
    
        MotorOut1 = MyConstrainF(MotorOut1, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
        MotorOut2 = MyConstrainF(MotorOut2, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
        MotorOut3 = MyConstrainF(MotorOut3, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
        MotorOut4 = MyConstrainF(MotorOut4, MOTOR_OUT_MIN, MOTOR_OUT_MAX);
    }
    else
    {
        MotorOut1 = MOTOR_OUT_MIN;
        MotorOut2 = MOTOR_OUT_MIN;
        MotorOut3 = MOTOR_OUT_MIN;
        MotorOut4 = MOTOR_OUT_MIN;
    }

    MotorSetDuty(Motor1, (INT16U)MotorOut1);
    MotorSetDuty(Motor2, (INT16U)MotorOut2);
    MotorSetDuty(Motor3, (INT16U)MotorOut3);
    MotorSetDuty(Motor4, (INT16U)MotorOut4);    
}