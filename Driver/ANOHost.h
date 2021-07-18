#ifndef ANOHOST_H
#define ANOHOST_H

#include "includes.h"
#include "LORA433.h"
#include "PID.h"
#include "Motor.h"

#define ANO_RXBUFF_LEN    50   

typedef enum
{
    ANOFunc = 0,
    ANOLenth,
    ANOData,
    ANOSum,
    ANOFrame
}ANOFrame_t;

void ANOSendStatus(FP32 Roll, FP32 Pitch, FP32 Yaw, FP32 Altitude, INT8U Flymode, INT8U Armed);

void ANOSendPID(PIDInfo *PID1, PIDInfo *PID2, PIDInfo *PID3);
void ANOSendCheck(INT8U Func, INT8U CheckSum);


void ANOReceive(INT8U Buff[ANO_RXBUFF_LEN]);

#endif

