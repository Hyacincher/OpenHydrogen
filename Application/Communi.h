#ifndef COMMUNI_H
#define COMMUNI_H

#include "NRF24L01.h"

/* Channel 0~31 */
#define RC_FRAME_HEAD       0
#define RC_ENCRYP_CHECK     1       //加密校验，应该为一约定值
#define RC_MODE_HEIGHT      2
#define RC_MODE_DIREC       3
#define RC_MODE_UNUSE0      4
#define RC_MODE_UNUSE1      5
#define RC_MODE_UNUSE2      6
#define RC_MODE_UNUSE3      7
#define RC_ROCKER_THRO_H    8
#define RC_ROCKER_THRO_L    9
#define RC_ROCKER_YAW_H     10
#define RC_ROCKER_YAW_L     11
#define RC_ROCKER_PITCH_H   12
#define RC_ROCKER_PITCH_L   13
#define RC_ROCKER_ROLL_H    14
#define RC_ROCKER_ROLL_L    15
#define RC_ROCKER_RESERVE0  16
#define RC_ROCKER_RESERVE1  17
#define RC_ROCKER_RESERVE2  18
#define RC_ROCKER_RESERVE3  19
#define RC_ROCKER_RESERVE4  20
#define RC_ROCKER_RESERVE5  21
#define RC_ROCKER_RESERVE6  22
#define RC_ROCKER_RESERVE7  23
#define RC_AUX_LEVEL_0      24      //AUX LEVEL CHANNEL 0   电平通道，不自动复位
#define RC_AUX_LEVEL_1      25      //AUX LEVEL CHANNEL 1
#define RC_AUX_LEVEL_2      26      //AUX LEVEL CHANNEL 2
#define RC_AUX_TRIGGER_0    27      //AUX TRIGGER CHANNEL 0 触发通道，发送1次自动复位
#define RC_AUX_TRIGGER_1    28      //AUX TRIGGER CHANNEL 1
#define RC_AUX_TRIGGER_2    29      //AUX TRIGGER CHANNEL 2
#define RC_CHECK_SUM        30
#define RC_FRAME_TAIL       31
#define RC_FRAME_LENTH      (RC_FRAME_TAIL+1)

#define FRAME_HEAD          0xAA
#define FRAME_TAIL          0x55

typedef struct
{
    struct
    {
    }Status;
    INT8U TaskStage;
    INT32U TaskTime;
    
}CommuniInfo_t;


typedef enum
{
    L0_MUnLock = 0,
}AUX_Level0_e;

typedef enum
{
    T0_AutoTK = 0,
    T0_LKEY1,
    T0_LKEY2,
    T0_RKEY1,
    T0_RKEY2,
    T0_Reserve5,
    T0_Reserve6,
    T0_Reserve7
}AUX_Trigger0_e;

void CommnuniInit(void);
void CommnuniTask(void);
void SetMotorUnLock(BOOLEAN Lock);
void UpdateRockerTrans(INT16S Throttle, INT16S Yaw, INT16S Picth, INT16S Roll);
void UpdateModeTrans(INT8U Index, INT8U Mode);

#endif

