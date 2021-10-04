#ifndef CPU_H
#define CPU_H

#include "stm32f4xx_hal.h"
#include "includes.h"

#ifdef __cplusplus
extern "C" {
#endif


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef enum
{
    IMUAxisX = 0,
    IMUAxisY,
    IMUAxisZ,
    IMUAxisAll
}IMUAxis_e;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RUN_Pin GPIO_PIN_0
#define LED_RUN_GPIO_Port GPIOC
#define LED_FAULT_Pin GPIO_PIN_1
#define LED_FAULT_GPIO_Port GPIOC
#define BAT_VOLT_Pin GPIO_PIN_3
#define BAT_VOLT_GPIO_Port GPIOC
#define LORA_MDO_Pin GPIO_PIN_3
#define LORA_MDO_GPIO_Port GPIOA
#define LORA_AUX_Pin GPIO_PIN_4
#define LORA_AUX_GPIO_Port GPIOC
#define NRF_MODE_Pin GPIO_PIN_5
#define NRF_MODE_GPIO_Port GPIOC
#define MPU_CS_Pin GPIO_PIN_0
#define MPU_CS_GPIO_Port GPIOB
#define NRF_CS_Pin GPIO_PIN_1
#define NRF_CS_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_11
#define NRF_IRQ_GPIO_Port GPIOB
#define BMP_CS_Pin GPIO_PIN_8
#define BMP_CS_GPIO_Port GPIOC
#define LSM_CS_Pin GPIO_PIN_3
#define LSM_CS_GPIO_Port GPIOB
#define LSM_INT2_Pin GPIO_PIN_4
#define LSM_INT2_GPIO_Port GPIOB
#define LSM_INT1_Pin GPIO_PIN_5
#define LSM_INT1_GPIO_Port GPIOB

/*-------------------Portable-----------------------*/
#define IWDG_INIT()         MX_IWDG_Init()
#define IWDG_FEED()         HAL_IWDG_Refresh(&hiwdg)

#define LED_RUN_OPEN()      HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET)
#define LED_RUN_CLOSE()     HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_SET)
#define LED_RUN_TOGGLE()    HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin)


#define LED_FAULT_OPEN()      HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_RESET)
#define LED_FAULT_CLOSE()     HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET)
#define LED_FAULT_TOGGLE()    HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin)
/*--------------------------------------------------*/

#ifdef __cplusplus
}
#endif

extern ADC_HandleTypeDef hadc1;

extern CAN_HandleTypeDef hcan2;

extern IWDG_HandleTypeDef hiwdg;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

extern volatile INT64U g_SysTickTime;

void HardWareInit(void);
void SwitchSPITiming(SPI_TypeDef * SPIn, BOOLEAN ClockIdle, BOOLEAN ClockPhase);
void MX_IWDG_Init(void);
void Hal_DelayMs(INT32U MS);

#endif


