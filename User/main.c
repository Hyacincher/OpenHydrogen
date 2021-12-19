#include "main.h"

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{  
    HardwareInit();
    PowerInit();
    CommnuniInit();
    
    LCD_ShowString(0, (INT8U *)"OpenHydrogen", Font_16_32, LCD_COLOR_BLACK, LCD_COLOR_WHITE);
    
    while (1)
    {
        PowerTask();
        CommnuniTask();
    }
}
