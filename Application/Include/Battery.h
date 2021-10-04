#ifndef BATTERY_H
#define BATTERY_H

#include "stm32f4xx_hal.h"
#include "cpu.h"
#include "Motor.h"

#define HIGHVOLT_UP     12.30f
#define HIGHVOLT_DOWN   12.10f

#define MIDVOLT_UP      11.75f
#define MIDVOLT_DOWN    11.55f

#define LOWVOLT_UP      11.20f
#define LOWVOLT_DOWN    11.00f

#define UNVOLT_UP       10.70f
#define UNVOLT_DOWN     10.50f

typedef enum
{
    HighVolt = 0,
    MidVolt,
    LowVolt,
    UnVolt,
    ShutDownVolt
}BatteryState_e;

typedef struct
{
    FP32 BatteryVolt;
    BatteryState_e Battery;
    
}BatteryInfo;


void BatteryInit(void);
void BatteryTask(void);

extern BatteryInfo g_BatteryCtrlMsg;

#endif


