#include "SysStatus.h"

void SysStatusInit(void)
{
    
}

void SysStatusTask(void)
{
    static INT64U s_SystemTime= 0;
    
    if(g_SysTickTime - s_SystemTime > 499)
    {
        LED_RUN_TOGGLE();
        s_SystemTime = g_SysTickTime;
    }
}
