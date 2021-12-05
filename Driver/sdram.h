#ifndef _SDRAM_H
#define _SDRAM_H

#include "includes.h"
#include "stm32f4xx_hal.h"

extern SDRAM_HandleTypeDef SDRAM_Handler;//SDRAM句柄
#define Bank5_SDRAM_ADDR    ((INT32U)(0XC0000000)) //SDRAM开始地址

//SDRAM配置参数
#define SDRAM_MODEREG_BURST_LENGTH_1             ((INT16U)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((INT16U)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((INT16U)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((INT16U)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((INT16U)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((INT16U)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((INT16U)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((INT16U)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((INT16U)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((INT16U)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((INT16U)0x0200)

void SDRAM_Init(void);
INT8U SDRAM_Send_Cmd(INT8U bankx,INT8U cmd,INT8U refresh,INT16U regval);
void FMC_SDRAM_WriteBuffer(INT8U *pBuffer,INT32U WriteAddr,INT32U n);
void FMC_SDRAM_ReadBuffer(INT8U *pBuffer,INT32U ReadAddr,INT32U n);
void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram);
#endif
