#ifndef LORA433_H
#define LORA433_H

#include "cpu.h"

#define LORA_MD0_HIGH()     GPIOA->ODR |= (1<<3)
#define LORA_MD0_LOW()      GPIOA->ODR &= ~(1<<3)
#define LORA_AUX_HIGH()     GPIOC->ODR |= (1<<4)
#define LORA_AUX_LOW()      GPIOC->ODR &= ~(1<<4)
#define LORA_DelayMS(x)      Hal_DelayMs(x)



void LORA433Init(void);
void LORASendData(INT8U *Data, INT16U Len);

#endif


