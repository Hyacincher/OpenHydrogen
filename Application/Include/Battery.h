#ifndef BATTERY_H
#define BATTERY_H

#include "stm32f4xx_hal.h"
#include "cpu.h"

#define UNVOLT_UP       11
#define UNVOLT_DOWN     10.8
#define SHUTDOWNVOLT    10.5

typedef struct
{
    struct
    {
        INT8U PowerState : 1;
        INT8U UnVoltState : 1;
    }State;
    
    FP32 BatteryVolt;
    
}BatteryInfo;


void BatteryInit(void);
void UpdateBattery(BatteryInfo *Battery);
void BatteryProtect(BatteryInfo *Battery);

extern BatteryInfo BatteryCtrlMsg;

#endif


