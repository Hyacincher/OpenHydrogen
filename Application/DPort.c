#include "DPort.h"

volatile DPortInfo_t    g_DPortCtrlMsg;

extern void SetMotorLock(BOOLEAN Lock);
extern void SetRockerMode(INT8U Index, BOOLEAN Status);

static void DPortTrigger(void);
static void DPortScan(void);
static void DPortIOHandle(DPortChannel_e Port);
static DPortLevel_e GetDPortLevel(DPortChannel_e Port);


void DigitalPortInit(void)
{
    memset((void *)&g_DPortCtrlMsg, 0, sizeof(g_DPortCtrlMsg));
    
    g_DPortCtrlMsg.DPort[L_PLUG_CHECK].Status.Activity = 1;//上电按键处理需要一定时间
    g_DPortCtrlMsg.DPort[R_PLUG_CHECK].Status.Activity = 1;
}

void DigitalPortTask(void)
{
    switch(g_DPortCtrlMsg.TaskStage)
    {
        case 0:
            g_DPortCtrlMsg.TaskTimer = g_SystemTime;
            g_DPortCtrlMsg.TaskStage++;
            break;
        case 1:
            if((g_SystemTime - g_DPortCtrlMsg.TaskTimer) > 0)
            {
                DPortScan();
                DPortTrigger();
                g_DPortCtrlMsg.TaskStage++;
            }
            break;
        case 2:
            g_DPortCtrlMsg.TaskStage = 0;
            break;
        default:
            g_DPortCtrlMsg.TaskStage = 0;
            g_DPortCtrlMsg.TaskTimer = g_SystemTime;
            break;
    }
}

static void DPortTrigger(void)
{
    if(g_DPortCtrlMsg.DPort[L_PLUG_CHECK].Status.Activity == 0)
    {
        
    }
    if(g_DPortCtrlMsg.DPort[R_PLUG_CHECK].Status.Activity == 0)
    {
        
    }    
    
    if(g_DPortCtrlMsg.DPort[L_KEY_TKOF].Status.TriggerLong)
    {//自动起飞/降落
        g_DPortCtrlMsg.DPort[L_KEY_TKOF].Status.TriggerLong = 0;
        
    }
    
    //电机解锁状态设置
    SetMotorLock(g_DPortCtrlMsg.DPort[L_LOCK_MOTOR].Status.Activity);
    
    SetRockerMode(0, g_DPortCtrlMsg.DPort[L_LOCK_ROCKER].Status.Activity);
    
    
}

static void DPortScan(void)
{
    for(INT16U ii = 0 ; ii < AllDPort ; ii++)
    {
        DPortIOHandle(ii);
    }
}

static void DPortIOHandle(DPortChannel_e Port)
{
    DPortLevel_e IO;
    
    IO = GetDPortLevel(Port);
    
    if((IO == DPortValid) && (g_DPortCtrlMsg.DPort[Port].IO.LastIOStatus == DPortInvalid))
    {//产生了有效边沿
        g_DPortCtrlMsg.DPort[Port].IO.AccumuStart = 1;
    }
    else if((IO == DPortInvalid) && (g_DPortCtrlMsg.DPort[Port].IO.LastIOStatus == DPortValid))
    {//产生了失效边沿
        g_DPortCtrlMsg.DPort[Port].IO.AccumuEnd = 1;
    }
    
    if(g_DPortCtrlMsg.DPort[Port].IO.AccumuStart)
    {
        if((IO == DPortValid))
        {//按下叠加
            g_DPortCtrlMsg.DPort[Port].IO.Count++;
        }
    }
    g_DPortCtrlMsg.DPort[Port].IO.LastIOStatus = IO;
    
    if(g_DPortCtrlMsg.DPort[Port].IO.AccumuStart & g_DPortCtrlMsg.DPort[Port].IO.AccumuEnd)
    {
        if((g_DPortCtrlMsg.DPort[Port].IO.Count >= TRIGGER_SHORT_TIME) &&\
            (g_DPortCtrlMsg.DPort[Port].IO.Count < TRIGGER_LONG_TIME))
        {
            g_DPortCtrlMsg.DPort[Port].Status.TriggerShort = 1;
        }
        else if(g_DPortCtrlMsg.DPort[Port].IO.Count >= TRIGGER_LONG_TIME)
        {
            if(!g_DPortCtrlMsg.DPort[Port].IO.NeedWait)
            {
                g_DPortCtrlMsg.DPort[Port].Status.TriggerLong = 1;
            }
        }
        g_DPortCtrlMsg.DPort[Port].IO.Count = 0;
        g_DPortCtrlMsg.DPort[Port].IO.AccumuStart = 0;
        g_DPortCtrlMsg.DPort[Port].IO.AccumuEnd = 0;
        g_DPortCtrlMsg.DPort[Port].IO.NeedWait = 0;
        g_DPortCtrlMsg.DPort[Port].IO.Wait = 0;
    }
    else if(g_DPortCtrlMsg.DPort[Port].IO.AccumuStart && (g_DPortCtrlMsg.DPort[Port].IO.Count == TRIGGER_LONG_TIME))
    {//达到了长按，但是没松手
        g_DPortCtrlMsg.DPort[Port].Status.TriggerLong = 1;
        g_DPortCtrlMsg.DPort[Port].IO.NeedWait = 1;
    }
    else if((g_DPortCtrlMsg.DPort[Port].IO.AccumuStart) && (g_DPortCtrlMsg.DPort[Port].IO.NeedWait))
    {//连按等待
        g_DPortCtrlMsg.DPort[Port].IO.Wait++;
    }
    if((g_DPortCtrlMsg.DPort[Port].IO.NeedWait) && (g_DPortCtrlMsg.DPort[Port].IO.Wait > TRIGGER_INTERVAL_TIME))
    {//等待结束，可以重新判断
        g_DPortCtrlMsg.DPort[Port].IO.Count = 0;
        g_DPortCtrlMsg.DPort[Port].IO.NeedWait = 0;
        g_DPortCtrlMsg.DPort[Port].IO.Wait = 0;
    }
    
    if(IO == DPortValid)
    {
        g_DPortCtrlMsg.DPort[Port].IO.LevelValid++;
        g_DPortCtrlMsg.DPort[Port].IO.LevelInValid = 0;
    }
    else
    {
        g_DPortCtrlMsg.DPort[Port].IO.LevelValid = 0;
        g_DPortCtrlMsg.DPort[Port].IO.LevelInValid++;
    }
    if(g_DPortCtrlMsg.DPort[Port].IO.LevelValid > TRIGGER_SHORT_TIME)
    {
        g_DPortCtrlMsg.DPort[Port].Status.Activity = 1;
    }
    if(g_DPortCtrlMsg.DPort[Port].IO.LevelInValid > TRIGGER_SHORT_TIME)
    {
        g_DPortCtrlMsg.DPort[Port].Status.Activity = 0;
    }
}

static DPortLevel_e GetDPortLevel(DPortChannel_e Port)
{
    DPortLevel_e Level;
    
    switch(Port)
    {
        case L_PLUG_CHECK:
            Level = (DPortLevel_e)(READL_PLUG_CHECK() ^ L_PLUG_CHECK_NTRIG);
            break;
        case L_KEY_TKOF:
            Level = (DPortLevel_e)(READL_KEY_TKOF() ^ L_KEY_TKOF_NTRIG);
            break;
        case L_KEY_1:
            Level = (DPortLevel_e)(READL_KEY_1() ^ L_KEY_1_NTRIG);
            break;
        case L_KEY_2:
            Level = (DPortLevel_e)(READL_KEY_2() ^ L_KEY_2_NTRIG);
            break;
        case L_LOCK_MOTOR:
            Level = (DPortLevel_e)(READL_LOCK_MOTOR() ^ L_LOCK_MOTOR_TRIG);
            break;
        case L_LOCK_ROCKER:
            Level = (DPortLevel_e)(READL_LOCK_ROCKER() ^ L_LOCK_ROCKER_NTRIG);
            break;
        case R_PLUG_CHECK:
            Level = (DPortLevel_e)(READR_PLUG_CHECK() ^ R_PLUG_CHECK_NTRIG);
            break;
        case R_SW_HEIGHT:
            Level = (DPortLevel_e)(READR_SW_HEIGHT() ^ R_SW_HEIGHT_NTRIG);
            break;
        case R_SW_DRIEC:
            Level = (DPortLevel_e)(READR_SW_DRIEC() ^ R_SW_DRIEC_NTRIG);
            break;
        case R_KEY_MODE:
            Level = (DPortLevel_e)(READR_KEY_MODE() ^ R_KEY_MODE_NTRIG);
            break;
        case R_KEY_1:
            Level = (DPortLevel_e)(READR_KEY_1() ^ R_KEY_1_NTRIG);
            break;
        case R_KEY_2:
            Level = (DPortLevel_e)(READR_KEY_2() ^ R_KEY_2_NTRIG);
            break;        
        case AllDPort:
            Level = 0;
            break;
        default:
            Level = 0;
            break;
    }
    return Level;
}
