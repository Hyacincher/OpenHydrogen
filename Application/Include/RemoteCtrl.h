#ifndef REMOTECTRL_H
#define REMOTECTRL_H

#include "includes.h"
#include "NRF24L01.h"
#include "PID.h"

#define ROCKER_VAL_BASE     1000
#define ROCKER_VAL_DIFF     1000

#define MAX_ROLL_DEGREE     60.0f       //+-30deg
#define MAX_PITCH_DEGREE    60.0f       //+-30deg
#define MAX_YAW_RATE        400.0f      //+-200deg/s

#define BASE_THROTTLE       2800
#define MAX_THROTTLE_DIFF   400

#define MAX_DISCONNECT_TIME 500

/*Channel 0~31, two bytes per data*/
#define RC_SYSTEM1_CHANNEL  0
#define RC_SYSTEM2_CHANNEL  2
#define RC_MODE1_CHANNEL 4
#define RC_MODE2_CHANNEL 6
#define RC_ROLL_CHANNEL     8
#define RC_THROTTLE_CHANNEL 10
#define RC_YAW_CHANNEL      12
#define RC_PITCH_CHANNEL    14
#define RC_KEY1_CHANNEL     16
#define RC_KEY2_CHANNEL     18

#define RC_CHECK_CHANNEL    31

/*KEY1*/
#define RC_KEY_LOCK_SHIFT       0
#define RC_KEY_UNLOCK_SHIFT     1

typedef struct
{
    struct
    {
        INT8U Connect : 1;
        INT8U Reserve : 7;
    }Status;
    
    INT16U RockerRoll;
    INT16U RockerPitch;
    INT16U RockerYaw;
    INT16U RockerThrottle;
    
    INT32U DisConnectTime;
    INT64U ErrCode;
}RemoteCtrlInfo;


void RemoteCtrlInit(void);
void RemoteCtrlTask(void);
void UpdateRocker(INT16U Roll, INT16U Pitch, INT16U Yaw, INT16U Throttle);

extern volatile RemoteCtrlInfo  g_RemoteCtrlMsg;


#endif

