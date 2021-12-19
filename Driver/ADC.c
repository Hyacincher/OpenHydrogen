#include "ADC.h"

volatile INT16U g_ADC1Volt[ADC1_Channel];   //mv
volatile INT16U g_ADC2Volt[ADC2_Channel];   //mv
    
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    INT32U Add1[ADC1_Channel],Add2[ADC2_Channel];
    
    if(hadc == &hadc1)
    {
        memset(Add1, 0, sizeof(Add1));
        
        for(INT16U jj = 0 ; jj < ADC_AVE_LENTH ; jj++)
        {
            for(INT16U ii = L_AIO_0 ; ii < ADC1_Channel ; ii++)
            {
                Add1[ii] += g_ADC1_Buff[ii + (jj * ADC1_Channel)];
            }
        }
        
        for(INT16U ii = L_AIO_0 ; ii < ADC1_Channel ; ii++)
        {
            Add1[ii] /= ADC_AVE_LENTH;
            Add1[ii] = (Add1[ii] / ADC_RESELUTION) * ADC_REF_VOLT;
            g_ADC1Volt[ii] = Add1[ii];
        }
    }
    else if(hadc == &hadc2)
    {
        memset(Add2, 0, sizeof(Add2));
        
        for(INT16U jj = 0 ; jj < ADC_AVE_LENTH ; jj++)
        {
            for(INT16U ii = R_AIO_2 ; ii < ADC2_Channel ; ii++)
            {
                Add2[ii] += g_ADC2_Buff[ii + (jj * ADC2_Channel)];
            }
        }
        
        for(INT16U ii = R_AIO_2 ; ii < ADC2_Channel ; ii++)
        {
            Add2[ii] /= ADC_AVE_LENTH;
            Add2[ii] = (Add2[ii] / ADC_RESELUTION) * ADC_REF_VOLT;
            g_ADC2Volt[ii] = Add2[ii];
        }
    }
}