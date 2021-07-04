#include "main.h"


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HardWareInit();
    
    HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);
    
    MPU6000Init();
    BMP280Init();
    LSM303DInit();
    NRFInit();
    LORA433Init();
    
    AttitudeInit();
    StabilizerInit();
    MotorInit();
    BatteryInit();
    DbgPrintInit();
    
    UART_Start_Receive_IT(&huart4, (INT8U *)g_DbgPrintCtrlMsg.ReceiveBuff, RECEIVE_BUFF_LEN);
    
    while (1)
    {
        BMP280Update();
        
        
        AttitudeTask();
        StabilizerTask();
        
        DbgPrintTask();
        
        BatteryTask();
        
        MotorTask();
        //HAL_Delay(2);
    }
}