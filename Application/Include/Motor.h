#ifndef MOTOR_H
#define MOTOR_H

#include "cpu.h"

#define MOTOR_M1  		0
#define MOTOR_M2  		1
#define MOTOR_M3  		2
#define MOTOR_M4  		3

#define MOTOR_OUT_MIN   2000
#define MOTOR_OUT_MAX   4000

typedef struct
{
    struct
    {
        INT8U Unlock : 1;   //1解锁       0锁定
        INT8U Change :1;    //存在状态改变
        INT8U Reserve : 6;
    }State;
    volatile INT32U Motor1,Motor2,Motor3,Motor4;
}MotorInfo;

void MotorInit(void);
void MotorLockCtrl(void);
void MotorSetDuty(INT16U Channel, INT32U Value);
void MotorUnLock(void);
void MotorLock(void);

extern MotorInfo g_MotorCtrlMsg;

#endif

