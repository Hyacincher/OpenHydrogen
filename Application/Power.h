#ifndef POWER_H
#define POWER_H

#include "ADC.h"
#include "ltdc.h"

#define BATTERY_VOLT_MUTI   4
#define HIGH_BATTERY_UP
#define HIGH_BATTERY_DOWN
#define MID_BATTERY_UP
#define MID_BATTERY_DOWN
#define LOW_BATTERY_UP
#define LOW_BATTERY_DOWN

typedef struct
{
    struct
    {
        INT8U PowerOn : 1;
        INT8U Reserve : 7;
    }Status;
    INT16U BatteryVolt;
    
    INT8U TaskStage;
    INT32U TaskTimer;
    
}PowerInfo_t;

void PowerInit(void);
void PowerTask(void);


#endif

