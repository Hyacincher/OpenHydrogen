#include "main.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * Hydrogenflight飞控源码
 * @Hyacincher
 * 创建日期:2020/2/1
 * 开源协议:GPL3.0
 * 设计参考来源：ATKFlight、Pixhawk、APM、网络开源资料
 * All rights reserved
********************************************************************************/

/*
1、电机大负载的情况下，NRF可能会断线（遥控器那边的问题）
2、地线隔离没有做好，电机启动有200mv纹波（非电机原因，PWM启动就有很大纹波）


1、BMP280启动有时候会失败，数据读出正常，矫正稳定算法有问题（同PWM启动问题）
2、启动配置时钟计算有问题，重新检查
4、XY预估数据偏差很大（还没做矫正）
5、坐标系参考https://blog.csdn.net/qq_27690393/article/details/94394879
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