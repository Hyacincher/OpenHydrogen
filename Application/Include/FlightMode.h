#ifndef FLIGHTMODE_H
#define FLIGHTMODE_H

#include "cpu.h"
#include "Battery.h"

typedef enum
{
    ManualCtrl = 0,
    FixedAltitude,
    FixedPoint,
    AutoReturn,
    Landing,
    StandBy
}FlightMode_e;

typedef struct
{
    FlightMode_e FlightMode;
}FlightModeInfo;


void FlightModeInit(void);
void FlightModeTask(void);

extern volatile FlightModeInfo  g_FlightModeCtrlMsg;

#endif


