#ifndef CPU_H
#define CPU_H

#include "includes.h"
#include "stm32f4xx_hal.h"

#include "sdram.h"
#include "ltdc.h"

void HardwareInit(void);
void Error_Handler(void);
#endif

