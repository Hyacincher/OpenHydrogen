#include "main.h"

/*
1、ANO间断输出状态，上位机更新异常，数据一直在发，不知道是不是上位机的问题
2、ANO设定PID功能异常
3、飞行会出现YAW一直自传的情况

*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HardWareInit();
    
    HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);
    
    AttitudeInit();
    StabilizerInit();
    BatteryInit();
    RemoteCtrlInit();
    DbgPrintInit();
    FlightModeInit();
    
    while (1)
    {
        AttitudeTask();
        
        StabilizerTask();
        
        DbgPrintTask();
        
        BatteryTask();
        
        RemoteCtrlTask();
        
        FlightModeTask();
    }
}