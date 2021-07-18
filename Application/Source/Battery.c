#include "Battery.h"

BatteryInfo BatteryCtrlMsg;

static void UpdateBattery(void);
static void BatteryProtect(void);

void BatteryInit(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
}

void BatteryTask(void)
{
    UpdateBattery();
    BatteryProtect();
}

static void UpdateBattery(void)
{
    static INT64U s_SystemTime = 0;
    INT32U Data;
    
    if(g_SysTickTime - s_SystemTime >= 50)
    {
        s_SystemTime = g_SysTickTime;
        
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 5);    //等待转换完成，第二个参数表示超时时间，单位ms   
        
        if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
        {
            Data = HAL_ADC_GetValue(&hadc1);
        }
                
        BatteryCtrlMsg.BatteryVolt = Data * 3.3 / 1024;
        
        if((FP64)BatteryCtrlMsg.BatteryVolt < UNVOLT_DOWN)
        {
            BatteryCtrlMsg.State.UnVoltState = 1;
        }
        if((FP64)BatteryCtrlMsg.BatteryVolt > UNVOLT_UP)
        {
            BatteryCtrlMsg.State.UnVoltState = 0;
        }
        
        if((FP64)BatteryCtrlMsg.BatteryVolt < SHUTDOWNVOLT)
        {
            BatteryCtrlMsg.State.PowerState = 0;
        }
        else
        {
            BatteryCtrlMsg.State.PowerState = 1;
        }        
    }
}

static void BatteryProtect(void)
{
    if(BatteryCtrlMsg.State.PowerState == 0)
    {
        MotorLock();
            //LED_FAULT_ON();
            //Battery Dangerous
    }
}

