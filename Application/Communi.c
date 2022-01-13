#include "Communi.h"

volatile CommuniInfo_t  g_CommuniCtrlMsg;


static void TransmitRocker(void);
static INT8U CalSum(INT8U *Buff, INT16U Lenth);
static void ResetAuxTrigger(void);
static void UpdateAuxLevelTrans(INT8U Aux, INT8U Shift, BOOLEAN Binary);
static void UpdateAuxTriggerTrans(INT8U Aux, INT8U Shift, BOOLEAN Binary);

void CommnuniInit(void)
{
    memset((void *)&g_CommuniCtrlMsg, 0, sizeof(g_CommuniCtrlMsg));
    
    NRFInit();
}

void CommnuniTask(void)
{
    switch(g_CommuniCtrlMsg.TaskStage)
    {
        case 0:
            g_CommuniCtrlMsg.TaskTime = g_SystemTime;
            g_CommuniCtrlMsg.TaskStage++;
            break;
        case 1:
            if((g_SystemTime - g_CommuniCtrlMsg.TaskTime) > 1)
            {
                //1ms发一次有一定的错误率，表现为帧后移，帧变成同一重复数据
                //2ms错误率明显下降，几乎不存在错误
                TransmitRocker();
                ResetAuxTrigger();
                g_CommuniCtrlMsg.TaskStage++;
            }
            break;
        case 2:
            g_CommuniCtrlMsg.TaskStage = 0;
            break;
        default:
            g_CommuniCtrlMsg.TaskStage = 0;
            g_CommuniCtrlMsg.TaskTime = g_SystemTime;
            break;
    }
}

static void TransmitRocker(void)
{
    g_NRFCtrlMsg.TxBuff[RC_FRAME_HEAD] = FRAME_HEAD;
    g_NRFCtrlMsg.TxBuff[RC_FRAME_TAIL] = FRAME_TAIL;
    
    INT8U Sum = CalSum(&g_NRFCtrlMsg.TxBuff[0], RC_CHECK_SUM);
    g_NRFCtrlMsg.TxBuff[RC_CHECK_SUM] = Sum;
    
    NRFTransmitData();
}

void SetMotorUnLock(BOOLEAN UnLock)
{
    UpdateAuxLevelTrans(0, L0_MUnLock, UnLock);
}

void SetAutoTakeoff(BOOLEAN Takeoff)
{
    UpdateAuxTriggerTrans(0, T0_AutoTK, Takeoff);
}

static INT8U CalSum(INT8U *Buff, INT16U Lenth)
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

void UpdateRockerTrans(INT16S Throttle, INT16S Yaw, INT16S Picth, INT16S Roll)
{
    g_NRFCtrlMsg.TxBuff[RC_ROCKER_THRO_H] = ((INT16U)Throttle >> 8);
    g_NRFCtrlMsg.TxBuff[RC_ROCKER_THRO_L] = ((INT16U)Throttle);
    
    g_NRFCtrlMsg.TxBuff[RC_ROCKER_YAW_H] = ((INT16U)Yaw >> 8);
    g_NRFCtrlMsg.TxBuff[RC_ROCKER_YAW_L] = ((INT16U)Yaw);

    g_NRFCtrlMsg.TxBuff[RC_ROCKER_PITCH_H] = ((INT16U)Picth >> 8);
    g_NRFCtrlMsg.TxBuff[RC_ROCKER_PITCH_L] = ((INT16U)Picth);

    g_NRFCtrlMsg.TxBuff[RC_ROCKER_ROLL_H] = ((INT16U)Roll >> 8);
    g_NRFCtrlMsg.TxBuff[RC_ROCKER_ROLL_L] = ((INT16U)Roll);
}

void UpdateModeTrans(INT8U Index, INT8U Mode)
{
    if(Index > 5)
    {
        return;
    }
    g_NRFCtrlMsg.TxBuff[RC_MODE_HEIGHT + Index] = Mode;
}

static void UpdateAuxLevelTrans(INT8U Aux, INT8U Shift, BOOLEAN Binary)
{
    INT8U AuxTemp;
    
    if(Aux > 2 || Shift > 7)
    {
        return;
    }
    
    AuxTemp = g_NRFCtrlMsg.TxBuff[RC_AUX_LEVEL_0 + Aux];
    AuxTemp &= ~(1<<Shift);
    AuxTemp |= ((1 & Binary)<<Shift);
    g_NRFCtrlMsg.TxBuff[RC_AUX_LEVEL_0 + Aux] = AuxTemp;
}

static void UpdateAuxTriggerTrans(INT8U Aux, INT8U Shift, BOOLEAN Binary)
{
    INT8U AuxTemp;
    
    if(Aux > 2 || Shift > 7)
    {
        return;
    }
    
    AuxTemp = g_NRFCtrlMsg.TxBuff[RC_AUX_TRIGGER_0 + Aux];
    AuxTemp &= ~(1<<Shift);
    AuxTemp |= ((1 & Binary)<<Shift);
    g_NRFCtrlMsg.TxBuff[RC_AUX_TRIGGER_0 + Aux] = AuxTemp;
}

static void ResetAuxTrigger(void)
{
    g_NRFCtrlMsg.TxBuff[RC_AUX_TRIGGER_0] = 0;
    g_NRFCtrlMsg.TxBuff[RC_AUX_TRIGGER_1] = 0;
    g_NRFCtrlMsg.TxBuff[RC_AUX_TRIGGER_2] = 0;
}