#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "includes.h"
#include "MPU6000.h"
#include "LSM303D.h"

#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */

#define GYRO_SCALE  16.384f  //32768/������
#define GYRO_LPF_CUTOFF_FREQ  	80.0f

#define ACCE_SCALE  4096.0f  //
#define ACCE_LPF_CUTOFF_FREQ 	15.0f

#define IMU_AXIS_NUM    3
#define IMU_AXIS_X      0
#define IMU_AXIS_Y      1
#define IMU_AXIS_Z      2

#define GYRO_UPDATE_RATE    RATE_500_HZ
#define ACCE_UPDATE_RATE    RATE_500_HZ

#define ATTITUDE_ESTIMAT_RATE	RATE_500_HZ				//��̬��������
#define ATTITUDE_ESTIMAT_DT		(1.0/ATTITUDE_ESTIMAT_RATE)

#define DCM_KP_ACC			0.600f		//���ٶȲ���������PI����
#define DCM_KI_ACC			0.005f

#define DCM_KP_MAG			1.000f		//�����Ʋ���������PI����
#define DCM_KI_MAG			0.000f

#define RAD    (PI / 180.0f)
#define DEGREES_TO_RADIANS(angle) ((angle) * RAD)
#define RADIANS_TO_DEGREES(angle) ((angle) / RAD)

#define SPIN_RATE_LIMIT     20			//��ת����

typedef struct
{
    FP32 NormailAcce[IMU_AXIS_NUM];    //��һ�����XYZ����
    FP32 NormailGyro[IMU_AXIS_NUM];    
    FP32 NormailMag[IMU_AXIS_NUM];

    FP32 Pitch;
    FP32 Roll;
    FP32 Yaw;
}AttitudeInfo;

void AttitudeInit(void);
void AttitudeTask(void);

extern AttitudeInfo g_AttitudeCtrlMsg;

#endif

