#include "RemoteCtrl.h"

RemoteCtrlInfo  g_RemoteCtrlMsg;

static void RockerToAltitude(void);

extern void UpdateSetpoint(FP32 SetRoll, FP32 SetPitch, FP32 SetYaw, FP32 Throttle);

void RemoteCtrlInit(void)
{
    memset(&g_RemoteCtrlMsg, 0, sizeof(g_RemoteCtrlMsg));
    g_RemoteCtrlMsg.RockerRoll = 1500;
    g_RemoteCtrlMsg.RockerPitch = 1500;
    g_RemoteCtrlMsg.RockerYaw = 1500;
    g_RemoteCtrlMsg.RockerThrottle = 1000;
    
    NRFInit();    
}

void RemoteCtrlTask(void)
{
    NRFReceiveData();
    RockerToAltitude();
}

static void RockerToAltitude(void)
{
    FP32 Roll,Pitch,Yaw,Throttle;
    
    Roll = (FP32)(g_RemoteCtrlMsg.RockerRoll - ROCKER_VAL_BASE) / ROCKER_VAL_DIFF;
    Pitch = (FP32)(g_RemoteCtrlMsg.RockerPitch - ROCKER_VAL_BASE) / ROCKER_VAL_DIFF;
    Yaw = (FP32)(g_RemoteCtrlMsg.RockerYaw - ROCKER_VAL_BASE) / ROCKER_VAL_DIFF;
    Throttle = (FP32)(g_RemoteCtrlMsg.RockerThrottle - ROCKER_VAL_BASE) / ROCKER_VAL_DIFF;
    
    Roll -= 0.5;
    Pitch -= 0.5;
    Yaw -= 0.5;
    
    Roll *= MAX_ROLL_DEGREE;
    Pitch *= MAX_PITCH_DEGREE;
    Throttle = (Throttle * MAX_THROTTLE_DIFF) + 2000;
    
    UpdateSetpoint(Roll, Pitch, 0, Throttle);
}

void UpdateRocker(INT16U Roll, INT16U Pitch, INT16U Yaw, INT16U Throttle)
{
    g_RemoteCtrlMsg.RockerRoll = Roll;
    g_RemoteCtrlMsg.RockerPitch = Pitch;
    g_RemoteCtrlMsg.RockerYaw = Yaw;
    g_RemoteCtrlMsg.RockerThrottle = Throttle;
}
