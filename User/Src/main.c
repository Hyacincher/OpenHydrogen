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
    
    while (1)
    {
        MPU6000pdate();
        BMP280Update();
        LSM303DUpdate();
        HAL_Delay(1);
    }
}