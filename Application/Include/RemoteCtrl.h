#ifndef REMOTECTRL_H
#define REMOTECTRL_H

#include "includes.h"
#include "NRF24L01.h"

#define ROCKER_VAL_BASE     1000
#define ROCKER_VAL_DIFF     1000

#define MAX_ROLL_DEGREE     60.0f
#define MAX_PITCH_DEGREE    60.0f
#define MAX_THROTTLE_DIFF   2000

typedef struct
{
    INT16U RockerRoll;
    INT16U RockerPitch;
    INT16U RockerYaw;
    INT16U RockerThrottle;
    
}RemoteCtrlInfo;


void RemoteCtrlInit(void);
void RemoteCtrlTask(void);
void UpdateRocker(INT16U Roll, INT16U Pitch, INT16U Yaw, INT16U Throttle);

extern RemoteCtrlInfo  g_RemoteCtrlMsg;


#endif

