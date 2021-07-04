#include "Motor.h"

/*
左前MOTOR4    右前MOTOR2
左后MOTOR3    右后MOTOR1
*/
MotorInfo   g_MotorCtrlMsg;


static void MotorOpen(INT32U Channel);
static void MotorClose(INT32U Channel);

void MotorInit(void)
{
    memset(&g_MotorCtrlMsg, 0, sizeof(g_MotorCtrlMsg));
    
    MotorUnLock();
    HAL_TIM_Base_Start(&htim1);
    
    TIM1->CCR1 = 2000;
    TIM1->CCR2 = 2000;
    TIM1->CCR3 = 2000;
    TIM1->CCR4 = 2000;
    
    g_MotorCtrlMsg.Motor1 = 2000;
    g_MotorCtrlMsg.Motor2 = 2000;
    g_MotorCtrlMsg.Motor3 = 2000;
    g_MotorCtrlMsg.Motor4 = 2000;
    
    HAL_Delay(1000);
}

void MotorTask(void)
{
    if(g_MotorCtrlMsg.State.Change)
    {
        g_MotorCtrlMsg.State.Change = 0;
        if(g_MotorCtrlMsg.State.Unlock)
        {
            MotorOpen(1);
            MotorOpen(2);
            MotorOpen(3);
            MotorOpen(4);
        }
        else
        {
            MotorClose(1);
            MotorClose(2);
            MotorClose(3);
            MotorClose(4);
        }
    }
}

void MotorUnLock(void)
{
    g_MotorCtrlMsg.State.Change = 1;
    g_MotorCtrlMsg.State.Unlock = 1;
}

void MotorLock(void)
{
    g_MotorCtrlMsg.State.Change = 1;
    g_MotorCtrlMsg.State.Unlock = 0;
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
void MotorSetDuty(INT16U Channel, INT32U Value)
{
    INT8U ii;
    
    switch(Channel)
    {
        case MOTOR_M1:		/*MOTOR_M1*/
            TIM1->CCR1 = g_MotorCtrlMsg.Motor1;
            break;
        case MOTOR_M2:		/*MOTOR_M2*/
            TIM1->CCR2 = g_MotorCtrlMsg.Motor2;
            break;
        case MOTOR_M3:		/*MOTOR_M3*/
            TIM1->CCR3 = g_MotorCtrlMsg.Motor3;
            break;
        case MOTOR_M4:		/*MOTOR_M4*/	
            TIM1->CCR4 = g_MotorCtrlMsg.Motor4;
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