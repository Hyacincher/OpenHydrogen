#include "Motor.h"

/*
左前MOTOR4    右前MOTOR2
左后MOTOR3    右后MOTOR1
*/
static INT8U s_MotorUnlock = 0;

static void MotorOpen(INT32U Channel);
static void MotorClose(INT32U Channel);

void MotorInit(void)
{
    MotorLock();
    HAL_TIM_Base_Start(&htim1);
    
    TIM1->CCR1 = MOTOR_OUT_MIN;
    TIM1->CCR2 = MOTOR_OUT_MIN;
    TIM1->CCR3 = MOTOR_OUT_MIN;
    TIM1->CCR4 = MOTOR_OUT_MIN;
}

void MotorUnLock(void)
{
    if(s_MotorUnlock == 0)
    {
        TIM1->CCR1 = MOTOR_OUT_MIN;
        TIM1->CCR2 = MOTOR_OUT_MIN;
        TIM1->CCR3 = MOTOR_OUT_MIN;
        TIM1->CCR4 = MOTOR_OUT_MIN;
        
        MotorOpen(1);
        MotorOpen(2);
        MotorOpen(3);
        MotorOpen(4);
        
        Hal_DelayMs(1500);
        
        s_MotorUnlock = 1;
    }
}

void MotorLock(void)
{
    MotorClose(1);
    MotorClose(2);
    MotorClose(3);
    MotorClose(4);
    s_MotorUnlock = 0;
}

/*
1   TIM_CHANNEL_1 0x00000000U                   
2   TIM_CHANNEL_2 0x00000004U                        
3   TIM_CHANNEL_3 0x00000008U
4   TIM_CHANNEL_4 0x0000000CU
*/
static void MotorOpen(INT32U Channel)
{
    HAL_TIM_OC_Start(&htim1, (Channel - 1)* 4);
}

/*
1   TIM_CHANNEL_1 0x00000000U                   
2   TIM_CHANNEL_2 0x00000004U                        
3   TIM_CHANNEL_3 0x00000008U                    
4   TIM_CHANNEL_4 0x0000000CU
*/
static void MotorClose(INT32U Channel)
{
    HAL_TIM_OC_Stop(&htim1, (Channel - 1)* 4);
}

/*
1   TIM_CHANNEL_1 0x00000000U                   
2   TIM_CHANNEL_2 0x00000004U                        
3   TIM_CHANNEL_3 0x00000008U                    
4   TIM_CHANNEL_4 0x0000000CU
*/
void MotorSetDuty(MotorChannel_e Channel, INT32U Value)
{
    INT8U ii;
    
    switch(Channel)
    {
        case Motor1:		/*MOTOR_M1*/
            TIM1->CCR1 = Value;
            break;
        case Motor2:		/*MOTOR_M2*/
            TIM1->CCR2 = Value;
            break;
        case Motor3:		/*MOTOR_M3*/
            TIM1->CCR3 = Value;
            break;
        case Motor4:		/*MOTOR_M4*/	
            TIM1->CCR4 = Value;
            break;
        default: 
            break;
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
    }
}

INT8U GetMotorUnLock(void)
{
    return s_MotorUnlock;
}

