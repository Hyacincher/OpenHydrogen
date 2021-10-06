#include "main.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * HyFlightctrl飞控源码
 * @Hyacincher
 * 创建日期:2020/2/1
 * 开源协议:GPL3.0
 * 设计参考来源：ATKFlight、Pixhawk、APM、网络开源资料
 * All rights reserved
********************************************************************************/

/*
1、电机大负载的情况下，NRF可能会断线（遥控器那边的问题）

1、启动电机时BMP280会出现大的数值阶跃，可能是地线干扰导致，需检查
2、XY预估数据偏差很大（还没做矫正）
*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HardWareInit();

    AttitudeInit();
    HeightInit();
    StabilizerInit();
    BatteryInit();
    RemoteCtrlInit();
    DbgPrintInit();
    FlightModeInit();
    SysStatusInit();
    PredictorInit();
    
    IWDG_INIT(); 
    
    while (1)
    {
        AttitudeTask();
        
        HeightTask();
        
        StabilizerTask();
        
        DbgPrintTask();
        
        BatteryTask();
        
        RemoteCtrlTask();
        
        FlightModeTask();
        
        SysStatusTask();
        
        PredictorTask();
        
        IWDG_FEED();
    }
}