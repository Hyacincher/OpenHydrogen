#include "Battery.h"

BatteryInfo BatteryCtrlMsg;

void BatteryInit(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
}

void UpdateBattery(BatteryInfo *Battery)
{
    INT32U Data;
    
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 5);    //等待转换完成，第二个参数表示超时时间，单位ms   
    
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
    {
        Data = HAL_ADC_GetValue(&hadc1);
    }
            
    Battery->BatteryVolt = Data * 3.3 / 1024;
    
    if((FP64)Battery->BatteryVolt < UNVOLT_DOWN)
    {
        Battery->State.UnVoltState = 1;
    }
    if((FP64)Battery->BatteryVolt > UNVOLT_UP)
    {
        Battery->State.UnVoltState = 0;
    }
    
    if((FP64)Battery->BatteryVolt < SHUTDOWNVOLT)
    {
        Battery->State.PowerState = 0;
    }
    else
    {
        Battery->State.PowerState = 1;
    }
}

void BatteryProtect(BatteryInfo *Battery)
{
    if(Battery->State.PowerState == 0)
    {
        MotorLock();
        while(1)
        {
            LED_FAULT_ON();
            //Battery Dangerous
        }
    }
}

