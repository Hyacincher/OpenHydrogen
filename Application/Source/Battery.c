#include "Battery.h"

BatteryInfo g_BatteryCtrlMsg;

static void UpdateBattery(void);

void BatteryInit(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    g_BatteryCtrlMsg.BatteryVolt = 12.6;
    g_BatteryCtrlMsg.Battery = HighVolt;
}

void BatteryTask(void)
{
    UpdateBattery();
}

static void UpdateBattery(void)
{
    static INT64U s_SystemTime = 0;
    FP32 Data;
    
    if(g_SysTickTime - s_SystemTime >= 50)
    {
        s_SystemTime = g_SysTickTime;
        
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 5);    //等待转换完成，第二个参数表示超时时间，单位ms   
        
        if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
        {
            Data = HAL_ADC_GetValue(&hadc1);
            Data = Data * 3.3 / 1024;
        }

        LowPassFilter(0.3, &Data, &g_BatteryCtrlMsg.BatteryVolt);
        
        if(g_BatteryCtrlMsg.BatteryVolt > HIGHVOLT_UP)
        {
            g_BatteryCtrlMsg.Battery = HighVolt;
        }
        else if((g_BatteryCtrlMsg.BatteryVolt < HIGHVOLT_DOWN) && (g_BatteryCtrlMsg.BatteryVolt > MIDVOLT_UP))
        {
            g_BatteryCtrlMsg.Battery = MidVolt;
        }
        else if((g_BatteryCtrlMsg.BatteryVolt < MIDVOLT_DOWN) && (g_BatteryCtrlMsg.BatteryVolt > LOWVOLT_UP))
        {
            g_BatteryCtrlMsg.Battery = LowVolt;
        }
        else if((g_BatteryCtrlMsg.BatteryVolt < LOWVOLT_DOWN) && (g_BatteryCtrlMsg.BatteryVolt > UNVOLT_UP))
        {
            g_BatteryCtrlMsg.Battery = UnVolt;
        }
        else if(g_BatteryCtrlMsg.BatteryVolt < UNVOLT_DOWN)
        {
            g_BatteryCtrlMsg.Battery = ShutDownVolt;
        }
    }
}

