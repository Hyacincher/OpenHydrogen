#include "main.h"

/*
1、电机大负载的情况下，NRF可能会断线

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