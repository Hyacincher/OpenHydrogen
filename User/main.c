#include "main.h"

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    INT16U Color[40*40];
    
    HardwareInit();
    PowerInit();
    CommnuniInit();
    
    lv_init();
    lv_port_disp_init();
    LVGLDemoInit();
    
    while (1)
    {
        PowerTask();
        CommnuniTask();
        lv_task_handler();
    }
}
