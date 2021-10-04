#include "FlightMode.h"

volatile FlightModeInfo  g_FlightModeCtrlMsg;

extern void MotorUnLock(void);
extern void MotorLock(void);
extern void SetOriginYaw(void);
extern BOOLEAN TakeoffCheck(void);

void FlightModeInit(void)
{
    memset((void *)&g_FlightModeCtrlMsg, 0, sizeof(g_FlightModeCtrlMsg));
    g_FlightModeCtrlMsg.ThrottleMode = ThroStandby;
    g_FlightModeCtrlMsg.DirectionMode = DirecStandBy;
}

void FlightModeTask(void)
{
    static INT64U s_SystemTime = 0;
    
    if(g_SysTickTime - s_SystemTime > 0)
    {
        s_SystemTime = g_SysTickTime;
        if(g_BatteryCtrlMsg.Battery == UnVolt)
        {
            g_FlightModeCtrlMsg.ThrottleMode = AutoLanding;
        }
        else if(g_BatteryCtrlMsg.Battery == ShutDownVolt)
        {
            g_FlightModeCtrlMsg.ThrottleMode = ThroStandby;
            g_FlightModeCtrlMsg.DirectionMode = DirecStandBy;
        }
    }
}

void FlightWorking(void)
{
    INT16U ii;
    
    if(GetMotorUnLock() == 0)
    {
        //机器没有在飞行途中
        
        if(TakeoffCheck())//起飞角度检查
        {
            g_FlightModeCtrlMsg.ThrottleMode = UserManual;      //调试用
            g_FlightModeCtrlMsg.DirectionMode = HeadDirection;
            
            SetOriginYaw();
            for(ii = 0 ; ii < PID_NUM ; ii++)
            {
                PIDReset(&g_PIDCtrlMsg[ii]);
            }
            MotorUnLock();        
        }
        else
        {
            g_FlightModeCtrlMsg.ThrottleMode = ThroStandby;
            g_FlightModeCtrlMsg.DirectionMode = DirecStandBy;        
        }
    }
}

void FlightStandBy(void)
{
    g_FlightModeCtrlMsg.ThrottleMode = ThroStandby;
    g_FlightModeCtrlMsg.DirectionMode = DirecStandBy;
    MotorLock();
}

