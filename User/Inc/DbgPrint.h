#ifndef DBGPRINT_H
#define DBGPRINT_H

#include "includes.h"
#include "LORA433.h"
#include "Attitude.h"
#include "LSM303D.h"
#include "Stabilizer.h"

typedef enum
{
    Wired = 0,
    WireLess
}SendMethod_e;

typedef enum
{
    Function = 0,
    Lenth,
    Data,
    SUM,
    Frame
}ANOFrame_t;

#ifdef DEBUG_MODE

#define ANO_PRINT 1
#define VISUAL_SCOPE_PRINT  1
    
#endif

#define RECEIVE_BUFF_LEN    50   

typedef struct
{
    struct
    {
        volatile INT8U ReceiveOK : 1;
        volatile INT8U ReceiveStart : 1;
        volatile INT8U Reserve : 6;
    }Status;
    
    volatile INT16U ReceiveLen;
    
    volatile INT8U ReceiveBuff[RECEIVE_BUFF_LEN];
    volatile INT16U ReceiveIndex;
    
}DebugPrint_t;

void DbgPrintInit(void);
void DbgPrintTask(void);
void VisualScope_Output(float data1 ,float data2 ,float data3 ,float data4);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);

extern DebugPrint_t g_DbgPrintCtrlMsg;

#endif


