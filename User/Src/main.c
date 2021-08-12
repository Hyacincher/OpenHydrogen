#include "main.h"


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