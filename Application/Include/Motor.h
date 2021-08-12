#ifndef MOTOR_H
#define MOTOR_H

#include "cpu.h"

typedef enum
{
    Motor1 = 0,
    Motor2,
    Motor3,
    Motor4,
    AllChannel
}MotorChannel_e;

#define MOTOR_OUT_MIN   2000
#define MOTOR_OUT_MAX   4000

void MotorInit(void);
void MotorSetDuty(MotorChannel_e Channel, INT32U Value);
void MotorUnLock(void);
void MotorLock(void);
INT8U GetMotorUnLock(void);

#endif

