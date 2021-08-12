#include "FlightMode.h"

volatile FlightModeInfo  g_FlightModeCtrlMsg;

void FlightModeInit(void)
{
    memset((void *)&g_FlightModeCtrlMsg, 0, sizeof(g_FlightModeCtrlMsg));
}

void FlightModeTask(void)
{
    static INT64U s_SystemTime = 0;
    
    if(g_SysTickTime - s_SystemTime > 0)
    {
        s_SystemTime = g_SysTickTime;
        if(g_BatteryCtrlMsg.Battery == UnVolt)
        {
            g_FlightModeCtrlMsg.FlightMode = Landing;
        }
        else if(g_BatteryCtrlMsg.Battery == ShutDownVolt)
        {
            g_FlightModeCtrlMsg.FlightMode = StandBy;
        }
        else
        {
            g_FlightModeCtrlMsg.FlightMode = ManualCtrl;
        }
    }
}
