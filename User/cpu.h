#ifndef CPU_H
#define CPU_H

#include "includes.h"
#include "stm32f4xx_hal.h"

#include "sdram.h"
#include "ltdc.h"

#define R_PLUG_Pin GPIO_PIN_2
#define R_PLUG_GPIO_Port GPIOE
#define R_DIO_1_Pin GPIO_PIN_3
#define R_DIO_1_GPIO_Port GPIOE
#define R_DIO_2_Pin GPIO_PIN_4
#define R_DIO_2_GPIO_Port GPIOE
#define L_PLUG_Pin GPIO_PIN_13
#define L_PLUG_GPIO_Port GPIOC
#define R_DIO_3_Pin GPIO_PIN_11
#define R_DIO_3_GPIO_Port GPIOI
#define R_AIO_0_Pin GPIO_PIN_1
#define R_AIO_0_GPIO_Port GPIOC
#define R_AIO_2_Pin GPIO_PIN_1
#define R_AIO_2_GPIO_Port GPIOA
#define R_AIO_1_Pin GPIO_PIN_2
#define R_AIO_1_GPIO_Port GPIOA
#define R_DIO_5_Pin GPIO_PIN_2
#define R_DIO_5_GPIO_Port GPIOH
#define R_DIO_4_Pin GPIO_PIN_3
#define R_DIO_4_GPIO_Port GPIOH
#define L_AIO_0_Pin GPIO_PIN_3
#define L_AIO_0_GPIO_Port GPIOA
#define L_AIO_1_Pin GPIO_PIN_4
#define L_AIO_1_GPIO_Port GPIOA
#define L_AIO_2_Pin GPIO_PIN_5
#define L_AIO_2_GPIO_Port GPIOA
#define L_AIO_3_Pin GPIO_PIN_6
#define L_AIO_3_GPIO_Port GPIOA
#define BAT_VOLT_Pin GPIO_PIN_7
#define BAT_VOLT_GPIO_Port GPIOA
#define R_DIO_7_Pin GPIO_PIN_4
#define R_DIO_7_GPIO_Port GPIOC
#define R_DIO_6_Pin GPIO_PIN_5
#define R_DIO_6_GPIO_Port GPIOC
#define R_AIO_4_Pin GPIO_PIN_0
#define R_AIO_4_GPIO_Port GPIOB
#define R_AIO_3_Pin GPIO_PIN_1
#define R_AIO_3_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_8
#define NRF_IRQ_GPIO_Port GPIOH
#define NRF_CS_Pin GPIO_PIN_12
#define NRF_CS_GPIO_Port GPIOB
#define NRF_SCK_Pin GPIO_PIN_13
#define NRF_SCK_GPIO_Port GPIOB
#define NRF_MISO_Pin GPIO_PIN_14
#define NRF_MISO_GPIO_Port GPIOB
#define NRF_MOSI_Pin GPIO_PIN_15
#define NRF_MOSI_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_13
#define NRF_CE_GPIO_Port GPIOD
#define LORA_TX_Pin GPIO_PIN_6
#define LORA_TX_GPIO_Port GPIOC
#define LORA_RX_Pin GPIO_PIN_7
#define LORA_RX_GPIO_Port GPIOC
#define LORA_AUX_Pin GPIO_PIN_8
#define LORA_AUX_GPIO_Port GPIOC
#define LORA_MD0_Pin GPIO_PIN_9
#define LORA_MD0_GPIO_Port GPIOC
#define DBG_UART_TX_Pin GPIO_PIN_9
#define DBG_UART_TX_GPIO_Port GPIOA
#define DBG_UART_RX_Pin GPIO_PIN_10
#define DBG_UART_RX_GPIO_Port GPIOA
#define R_DIO_8_Pin GPIO_PIN_10
#define R_DIO_8_GPIO_Port GPIOC
#define L_DIO_8_Pin GPIO_PIN_11
#define L_DIO_8_GPIO_Port GPIOC
#define L_DIO_7_Pin GPIO_PIN_12
#define L_DIO_7_GPIO_Port GPIOC
#define L_DIO_6_Pin GPIO_PIN_2
#define L_DIO_6_GPIO_Port GPIOD
#define L_DIO_5_Pin GPIO_PIN_3
#define L_DIO_5_GPIO_Port GPIOD
#define L_DIO_4_Pin GPIO_PIN_7
#define L_DIO_4_GPIO_Port GPIOD
#define L_DIO_3_Pin GPIO_PIN_10
#define L_DIO_3_GPIO_Port GPIOG
#define L_DIO_2_Pin GPIO_PIN_13
#define L_DIO_2_GPIO_Port GPIOG
#define L_DIO_1_Pin GPIO_PIN_14
#define L_DIO_1_GPIO_Port GPIOG

#define ADC_AVE_LENTH       20

#define READL_PLUG_CHECK()      HAL_GPIO_ReadPin(L_PLUG_GPIO_Port, L_PLUG_Pin)
#define READL_KEY_TKOF()        HAL_GPIO_ReadPin(L_DIO_1_GPIO_Port, L_DIO_1_Pin)
#define READL_KEY_1()           HAL_GPIO_ReadPin(L_DIO_2_GPIO_Port, L_DIO_2_Pin)
#define READL_KEY_2()           HAL_GPIO_ReadPin(L_DIO_3_GPIO_Port, L_DIO_3_Pin)
#define READL_LOCK_MOTOR()      HAL_GPIO_ReadPin(L_DIO_4_GPIO_Port, L_DIO_4_Pin)
#define READL_LOCK_ROCKER()     HAL_GPIO_ReadPin(L_DIO_5_GPIO_Port, L_DIO_5_Pin)

#define READR_PLUG_CHECK()      HAL_GPIO_ReadPin(R_PLUG_GPIO_Port, R_PLUG_Pin)
#define READR_SW_HEIGHT()       HAL_GPIO_ReadPin(R_DIO_4_GPIO_Port, R_DIO_4_Pin)
#define READR_SW_DRIEC()        HAL_GPIO_ReadPin(R_DIO_5_GPIO_Port, R_DIO_5_Pin)
#define READR_KEY_MODE()        HAL_GPIO_ReadPin(R_DIO_6_GPIO_Port, R_DIO_6_Pin)
#define READR_KEY_1()           HAL_GPIO_ReadPin(R_DIO_7_GPIO_Port, R_DIO_7_Pin)
#define READR_KEY_2()           HAL_GPIO_ReadPin(R_DIO_8_GPIO_Port, R_DIO_8_Pin)

#define LCD_BK_ENABLE()         HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2)
#define LCD_BK_DISABLE()        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2)

typedef enum
{
    L_AIO_0 = 0,
    L_AIO_1,
    L_AIO_2,
    L_AIO_3,
    BAT_VOLT,
    ADC1_Channel
}ADC1Channel_e;

typedef enum
{
    R_AIO_2 = 0,
    R_AIO_1,
    R_AIO_4,
    R_AIO_3,
    R_AIO_0,
    ADC2_Channel
}ADC2Channel_e;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

extern volatile INT16U g_ADC1_Buff[ADC1_Channel * ADC_AVE_LENTH];
extern volatile INT16U g_ADC2_Buff[ADC2_Channel * ADC_AVE_LENTH];
extern volatile INT32U g_SystemTime;

void HardwareInit(void);
void Error_Handler(void);
void SysTickISR(void);

#endif

