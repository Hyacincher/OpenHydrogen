#include "Power.h"

volatile PowerInfo_t    g_PowerCtrlMsg;

static void UpdateBatteryVolt(void);

void PowerInit(void)
{
    memset((void *)&g_PowerCtrlMsg, 0, sizeof(g_PowerCtrlMsg));
}

void PowerTask(void)
{
    
    switch(g_PowerCtrlMsg.TaskStage)
    {
        case 0:
            g_PowerCtrlMsg.TaskTimer = g_SystemTime;
            g_PowerCtrlMsg.TaskStage++;
            break;
        case 1:
            if((g_SystemTime - g_PowerCtrlMsg.TaskTimer) >= 200)
            {
                INT8U Str[20];
                
                UpdateBatteryVolt();
                sprintf((char *)Str, "Battery:%dmV", g_PowerCtrlMsg.BatteryVolt);
                LCD_ShowString(1, Str, Font_16_32, LCD_COLOR_BLACK, LCD_COLOR_WHITE);
                g_PowerCtrlMsg.TaskStage++;
            }
            break;
        case 2:
            g_PowerCtrlMsg.TaskStage = 0;
            break;
        default:
            g_PowerCtrlMsg.TaskStage = 0;
            g_PowerCtrlMsg.TaskTimer = g_SystemTime;
            break;
    }
}

static void UpdateBatteryVolt(void)
{
    g_PowerCtrlMsg.BatteryVolt = g_ADC1Volt[BAT_VOLT] * BATTERY_VOLT_MUTI;
}
