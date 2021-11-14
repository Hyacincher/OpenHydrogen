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

2、启动电机时BMP280会出现大的数值阶跃，可能是地线干扰导致，需检查
经检查发现是LORA与电机PWM线互相干扰，控制线屏蔽没做好，形成天线与LORA互相干扰。
下一板子四层板信号线走中间，外面地平面包裹提升板子抗干扰性能。
PWM信号按差分线规格走。
接口以外线用锡纸包裹，换屏蔽双绞线。
**电机堵转越厉害，气压计偏离越远
**1号电调有问题，接上就造成气压计漂移

2、XY预估数据偏差很大（还没做矫正）

3、航向角无力，无法恒定方向，检查PID参数

4、起飞自动积分往一边偏，容易翻车。暂时无法装脚架

5、定高内环积分不够，在稳态误差震荡

6、航向力度远远不够，横滚打杆会造成飞机航向偏移，微风也会导致飞机自转。
考虑关闭积分，大比例

7、定高力道不够，大角度推俯仰时会造成高度大幅度下降，航向调好后室内精调
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