#ifndef DBGPRINT_H
#define DBGPRINT_H

#include "includes.h"
#include "Attitude.h"
#include "Stabilizer.h"
#include "LSM303D.h"
#include "BMP280.h"

#include "ANOHost.h"

typedef struct
{
    struct
    {
        volatile INT8U ReceiveOK : 1;
        volatile INT8U ReceiveStart : 1;
        volatile INT8U Reserve : 6;
    }Status;
    
    volatile INT16U ReceiveLen;
    
    volatile INT8U ReceiveBuff[ANO_RXBUFF_LEN];
    volatile INT16U ReceiveIndex;
    
}DebugPrint_t;

void DbgPrintInit(void);
void DbgPrintTask(void);
void VisualScope_Output(float data1 ,float data2 ,float data3 ,float data4);

void USER_UART_IRQHandler(UART_HandleTypeDef *huart);

extern DebugPrint_t g_DbgPrintCtrlMsg;

#endif


