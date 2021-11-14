#include "RemoteCtrl.h"

volatile RemoteCtrlInfo  g_RemoteCtrlMsg;

static void ReceiveToRocker(void);
static void RockerToAltitude(void);
static void ReceiveKeyDeal(void);
static INT8U GetSum(INT8U *Buff, INT16U Lenth);

extern void UpdateSetpoint(FP32 SetRoll, FP32 SetPitch, FP32 SetYaw, FP32 Throttle);
extern void FlightWorking(void);
extern void FlightStandBy(void);

void RemoteCtrlInit(void)
{
    memset((void *)&g_RemoteCtrlMsg, 0, sizeof(g_RemoteCtrlMsg));
    g_RemoteCtrlMsg.RockerRoll = 1500;
    g_RemoteCtrlMsg.RockerPitch = 1500;
    g_RemoteCtrlMsg.RockerYaw = 1500;
    g_RemoteCtrlMsg.RockerThrottle = 1000;
    
    NRFInit();    
}

void RemoteCtrlTask(void)
{
    static INT64U s_SystemTime= 0;
    
    if(g_SysTickTime - s_SystemTime >= 1)
    {
        s_SystemTime = g_SysTickTime;
        
        NRFReceiveData();
        ReceiveToRocker();
        RockerToAltitude();
        ReceiveKeyDeal();
    }
}

static void ReceiveToRocker(void)
{
    INT16U ReceiveData;
    INT8U Sum;
    INT8U Key;
    INT16U ii;
    
    if(g_NRFCtrlMsg.State.RxFlag)
    {
        g_NRFCtrlMsg.State.RxFlag = 0;

        if((g_NRFCtrlMsg.RxBuff[RC_SYSTEM1_CHANNEL] != 0xAA) || \
            (g_NRFCtrlMsg.RxBuff[RC_SYSTEM1_CHANNEL + 1] != 0xAA))
        {
            g_RemoteCtrlMsg.ErrCode++;
            g_RemoteCtrlMsg.DisConnectTime++;
            return;
        }
        
        Sum = GetSum(g_NRFCtrlMsg.RxBuff, 32 - 1);
        
        if(Sum != g_NRFCtrlMsg.RxBuff[RC_CHECK_CHANNEL])
        {
            g_RemoteCtrlMsg.ErrCode++;
            g_RemoteCtrlMsg.DisConnectTime++;
            return;
        }
        
        g_RemoteCtrlMsg.Status.Connect = 1;
        ReceiveData = (INT16U)g_NRFCtrlMsg.RxBuff[RC_ROLL_CHANNEL] << 8;
        ReceiveData |= (INT8U)g_NRFCtrlMsg.RxBuff[RC_ROLL_CHANNEL+1];
        ReceiveData = MyConstrainINT16U(ReceiveData, ROCKER_VAL_BASE, ROCKER_VAL_BASE + ROCKER_VAL_DIFF);
        g_RemoteCtrlMsg.RockerRoll = ReceiveData;
        
        ReceiveData = (INT16U)g_NRFCtrlMsg.RxBuff[RC_THROTTLE_CHANNEL] << 8;
        ReceiveData |= (INT8U)g_NRFCtrlMsg.RxBuff[RC_THROTTLE_CHANNEL+1];
        ReceiveData = MyConstrainINT16U(ReceiveData, ROCKER_VAL_BASE, ROCKER_VAL_BASE + ROCKER_VAL_DIFF);
        g_RemoteCtrlMsg.RockerThrottle = ReceiveData;
        
        ReceiveData = (INT16U)g_NRFCtrlMsg.RxBuff[RC_YAW_CHANNEL] << 8;
        ReceiveData |= (INT8U)g_NRFCtrlMsg.RxBuff[RC_YAW_CHANNEL+1];
        ReceiveData = MyConstrainINT16U(ReceiveData, ROCKER_VAL_BASE, ROCKER_VAL_BASE + ROCKER_VAL_DIFF);
        g_RemoteCtrlMsg.RockerYaw = ReceiveData;
        
        ReceiveData = (INT16U)g_NRFCtrlMsg.RxBuff[RC_PITCH_CHANNEL] << 8;
        ReceiveData |= (INT8U)g_NRFCtrlMsg.RxBuff[RC_PITCH_CHANNEL+1];
        ReceiveData = MyConstrainINT16U(ReceiveData, ROCKER_VAL_BASE, ROCKER_VAL_BASE + ROCKER_VAL_DIFF);
        g_RemoteCtrlMsg.RockerPitch = ReceiveData;
        
        Key = g_NRFCtrlMsg.RxBuff[RC_KEY1_CHANNEL];
        if(Key & (1 << RC_KEY_LOCK_SHIFT))
        {
            g_RemoteCtrlMsg.Status.LockPress = 1;
        }
        else
        {
            g_RemoteCtrlMsg.Status.LockPress = 0;
        }
        if(Key & (1 << RC_KEY_UNLOCK_SHIFT))
        {
            g_RemoteCtrlMsg.Status.UnLockPress = 1;
        }
        else
        {
            g_RemoteCtrlMsg.Status.UnLockPress = 0;
        }
        
        g_RemoteCtrlMsg.DisConnectTime = 0;
    }
    else
    {
        g_RemoteCtrlMsg.DisConnectTime++;
    }
    
    if(g_RemoteCtrlMsg.DisConnectTime > MAX_DISCONNECT_TIME)
    {
        g_RemoteCtrlMsg.Status.Connect = 0;
        g_RemoteCtrlMsg.RockerRoll = ROCKER_VAL_BASE + (ROCKER_VAL_DIFF * 0.5);
        g_RemoteCtrlMsg.RockerYaw = ROCKER_VAL_BASE + (ROCKER_VAL_DIFF * 0.5);
        g_RemoteCtrlMsg.RockerPitch = ROCKER_VAL_BASE + (ROCKER_VAL_DIFF * 0.5);
        g_RemoteCtrlMsg.RockerThrottle = ROCKER_VAL_BASE + (ROCKER_VAL_DIFF * 0.5);
        
        g_RemoteCtrlMsg.Status.LockPress = 0;
        g_RemoteCtrlMsg.Status.UnLockPress = 0;
    }
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
    Pitch *= -MAX_PITCH_DEGREE;
    Yaw *= MAX_YAW_RATE;
    
    UpdateSetpoint(Roll, Pitch, Yaw, Throttle);
}

static void ReceiveKeyDeal(void)
{
    static INT8U s_KeyCtrl = 0;
    static INT64U s_StartTime;
    
    switch(s_KeyCtrl)
    {
        case 0:
            if(g_RemoteCtrlMsg.Status.LockPress)
            {
                s_StartTime = g_SysTickTime;
                s_KeyCtrl = 10;
            }
            if(g_RemoteCtrlMsg.Status.UnLockPress)
            {
                s_StartTime = g_SysTickTime;
                s_KeyCtrl = 20;
            }           
            break;
        case 10:
            if(g_RemoteCtrlMsg.Status.LockPress == 0)
            {
                s_KeyCtrl++;
                break;
            }
            if(g_SysTickTime - s_StartTime > KEY_LOCK_PRESS_TIME)
            {
                s_KeyCtrl++;
                break;
            }
            if(g_RemoteCtrlMsg.Status.Connect == 0)
            {
                s_StartTime = g_SysTickTime;    //断线无效
                s_KeyCtrl++;
                break;
            }
            break;
        case 11:
            if(g_SysTickTime - s_StartTime > KEY_LOCK_PRESS_TIME)
            {
                FlightStandBy();
            }
            s_KeyCtrl = 0;
            break;
            
        case 20:
            if(g_RemoteCtrlMsg.Status.UnLockPress == 0)
            {
                s_KeyCtrl++;
                break;
            }
            if(g_SysTickTime - s_StartTime > KEY_UNLOCK_PRESS_TIME)
            {
                s_KeyCtrl++;
                break;
            }
            if(g_RemoteCtrlMsg.Status.UnLockPress == 0)
            {
                s_StartTime = g_SysTickTime;    //断线无效
                s_KeyCtrl++;
                break;
            }
            break;
        case 21:
            if(g_SysTickTime - s_StartTime > KEY_UNLOCK_PRESS_TIME)
            {
                FlightWorking();
            }
            s_KeyCtrl = 0;
            break;
        default:
            s_KeyCtrl = 0;
            break;
    }

}

void UpdateRocker(INT16U Roll, INT16U Pitch, INT16U Yaw, INT16U Throttle)
{
    g_RemoteCtrlMsg.RockerRoll = Roll;
    g_RemoteCtrlMsg.RockerPitch = Pitch;
    g_RemoteCtrlMsg.RockerYaw = Yaw;
    g_RemoteCtrlMsg.RockerThrottle = Throttle;
}

static INT8U GetSum(INT8U *Buff, INT16U Lenth)
{
    INT16U ii;
    INT64U Sum;
    
    Sum = 0;
    
    for(ii = 0 ; ii < Lenth ; ii++)
    {
        Sum += Buff[ii];
    }
    
    return (INT8U)Sum;
}

//0~1
FP32 GetThrottleRate(void)
{
    FP32 Throttle;
    
    Throttle = (FP32)(g_RemoteCtrlMsg.RockerThrottle - ROCKER_VAL_BASE) / ROCKER_VAL_DIFF;
    
    return Throttle;
}


