#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "includes.h"
#include "MPU6000.h"
#include "LSM303D.h"


#define RAD                     (PI / 180.0f)
#define DEGREES_TO_RADIANS(angle)   ((angle) * RAD)
#define RADIANS_TO_DEGREES(angle)   ((angle) / RAD)

#define GYRO_SCALE              16.384f  //32768/总量程
#define GYRO_LPF_CUTOFF_FREQ  	80.0f

#define ACCE_SCALE              4096.0f  //重力加速度分度
#define ACCE_LPF_CUTOFF_FREQ 	15.0f

#define GYRO_UPDATE_RATE        RATE_500_HZ
#define ACCE_UPDATE_RATE        RATE_500_HZ

#define ATTITUDE_ESTIMAT_RATE	RATE_500_HZ				//姿态解算速率
#define ATTITUDE_ESTIMAT_DT		(1.0/ATTITUDE_ESTIMAT_RATE)

#define DCM_KP_ACC			    0.600f		//加速度补偿陀螺仪PI参数
#define DCM_KI_ACC			    0.005f

#define DCM_KP_MAG			    1.000f		//磁力计补偿陀螺仪PI参数
#define DCM_KI_MAG			    0.000f

#define SPIN_RATE_LIMIT         20			//旋转速率

#define MAX_TAKEOFF_ANGLE       20.0f

#define IMU_SMALL_ANGLE         15.0f

typedef struct
{
    struct
    {
        INT8U SmallAngle : 1;
        INT8U Reserve : 7;
    }Status;
    
    FP32 NormailAcce[IMUAxisAll];    //归一化后的XYZ数据
    FP32 NormailGyro[IMUAxisAll];    //归一化的角速度（弧度）
    FP32 NormailMag[IMUAxisAll];     //归一化的磁力计值

    FP32 Pitch;
    FP32 Roll;
    FP32 Yaw;
    
    FP32 Altitude;
    
    FP32 NEUAcce[IMUAxisAll];       //世界坐标系加速度
}AttitudeInfo;

void AttitudeInit(void);
void AttitudeTask(void);
void TransBodyVectorToNEU(FP32 Acce[IMUAxisAll]);
void TransNEUVectorToBody(FP32 Acce[IMUAxisAll]);

extern AttitudeInfo g_AttitudeCtrlMsg;

#endif

