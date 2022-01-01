#ifndef ADC_H
#define ADC_H

#include "cpu.h"

#define ADC_RESELUTION  4096.0f
#define ADC_REF_VOLT    3300

extern volatile INT16U g_ADC1Volt[ADC1_Channel];   //mv
extern volatile INT16U g_ADC2Volt[ADC2_Channel];   //mv

void ADC_Init(void);

#endif

