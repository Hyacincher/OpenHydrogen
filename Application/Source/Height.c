#include "Height.h"

volatile HeightInfo  g_HeightCtrlMsg;

static void CalculateAltitude(void);

void HeightInit(void)
{
    memset((void *)&g_HeightCtrlMsg, 0, sizeof(g_HeightCtrlMsg));
    
    BMP280Init();
}

void HeightTask(void)
{
    static INT64U s_SystemTime = 0;
    static INT8U s_HeightStage = 0;
    
    switch(s_HeightStage)
    {
        case 0:
            s_SystemTime = g_SysTickTime;
            CalculateAltitude();  //500hz
            s_HeightStage++;
            break;
        case 1:
            if(g_SysTickTime - s_SystemTime >= 2)
            {
                s_HeightStage++;
            }
            break;
        case 2:
            s_HeightStage = 0;
            break;
        default:
            s_HeightStage = 0;
            break;
    }
}

//解算高度信息，当前气压计单传感器，第二阶段会加上TOF组成双传感器
static void CalculateAltitude(void)
{
    BMP280Update();
    
    if(g_BMPCtrlMsg.Status.IsStable)
    {
        g_HeightCtrlMsg.Altitude = g_BMPCtrlMsg.Altitude;
        g_HeightCtrlMsg.Status.HeightIsStable = 1;
    }
    else
    {
        g_HeightCtrlMsg.Altitude = 0;
        g_HeightCtrlMsg.Status.HeightIsStable = 0;
    }
}