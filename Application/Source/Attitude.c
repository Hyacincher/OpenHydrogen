#include "Attitude.h"

AttitudeInfo g_AttitudeCtrlMsg;
BiquadFilterInfo g_GyroLFPFilter[IMU_AXIS_NUM];
BiquadFilterInfo g_AcceLFPFilter[IMU_AXIS_NUM];

/**	
 * 姿态解算规则如下：
 *     ROLL  = 绕X轴旋转，右手定则，逆时针为正顺时针为负。
 *     PITCH = 绕Y轴旋转，右手定则，逆时针为正顺时针为负。
 *     YAW   = 绕Z轴旋转，右手定则，逆时针为正顺时针为负。
 */
 
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;//四元数
static float rMat[3][3];//四元数的旋转矩阵		
static float smallAngleCosZ;//水平最小角余弦值

static void NormailAttitude(void);
static void CalculateAttitude(void);
static void imuMahonyAHRSupdate(FP32 gx, FP32 gy, FP32 gz,
                                FP32 ax, FP32 ay, FP32 az,
                                FP32 mx, FP32 my, FP32 mz,
								BOOLEAN useMag,FP32 dt);
static void imuUpdateEulerAngles(void);

void AttitudeInit(void)
{
    memset(&g_AttitudeCtrlMsg, 0, sizeof(g_AttitudeCtrlMsg));
    
	//初始化二阶低通滤波
	for (INT16U axis = 0; axis < 3; axis++)
	{
		BiquadFilterInitLPF(&g_GyroLFPFilter[axis], GYRO_UPDATE_RATE, GYRO_LPF_CUTOFF_FREQ);
        BiquadFilterInitLPF(&g_AcceLFPFilter[axis], ACCE_UPDATE_RATE, ACCE_LPF_CUTOFF_FREQ);
	}
}

void AttitudeTask(void)
{/*直接欧拉角解算会有很多动态问题，使用四元数*/
    static INT64U s_SystemTime = 0;
    static INT8U s_AttitudeStage = 0;
    
    switch(s_AttitudeStage)
    {
        case 0:
            s_SystemTime = g_SysTickTime;
            NormailAttitude();//500hz
            CalculateAttitude();//500hz
            s_AttitudeStage++;
            break;
        case 1:
            if(g_SysTickTime - s_SystemTime >= 2)
            {
                s_AttitudeStage++;
            }
            break;
        case 2:
            s_AttitudeStage = 0;
            break;
    }
}

//归一化6轴数据,并且进行低通滤波
static void NormailAttitude(void)
{
    INT16U ii;
    FP32 Acce[IMU_AXIS_NUM],Gyro[IMU_AXIS_NUM],Mag[IMU_AXIS_NUM];
    
    MPU6000Update();
    LSM303DUpdate();

    for(ii = 0 ; ii < 3 ; ii++)
    {//先归一化
        Acce[ii] = g_MPUCtrlMsg.RawAcce[ii] / ACCE_SCALE;    //+-8g
        Gyro[ii] = g_MPUCtrlMsg.RawGyro[ii] / GYRO_SCALE;    //+-2000
    }

	for (INT16U axis = 0; axis < 3; axis++) 
	{
		g_AttitudeCtrlMsg.NormailGyro[axis] = BiquadLPFFilter(&g_GyroLFPFilter[axis], Gyro[axis]);
        g_AttitudeCtrlMsg.NormailGyro[axis] *= DEG2RAD;//角速度单位由度转为弧度
        g_AttitudeCtrlMsg.NormailAcce[axis] = BiquadLPFFilter(&g_AcceLFPFilter[axis], Acce[axis]);
        g_AttitudeCtrlMsg.NormailMag[axis] = g_LSMCtrLMsg.RawMag[axis];
	}
}

static void CalculateAttitude(void)
{
    FP32 Acce[3],Gyro[3],Mag[3];
    
    imuMahonyAHRSupdate(g_AttitudeCtrlMsg.NormailGyro[0], g_AttitudeCtrlMsg.NormailGyro[1], g_AttitudeCtrlMsg.NormailGyro[2],
                        g_AttitudeCtrlMsg.NormailAcce[0], g_AttitudeCtrlMsg.NormailAcce[1], g_AttitudeCtrlMsg.NormailAcce[2],
                        g_AttitudeCtrlMsg.NormailMag[0], g_AttitudeCtrlMsg.NormailMag[1], g_AttitudeCtrlMsg.NormailMag[2],
                        1, ATTITUDE_ESTIMAT_DT);
    
    //计算欧拉角               
    imuUpdateEulerAngles();    
}

//磁力计快速增益
static float imuMagFastPGainSaleFactor(void)
{
	//四轴上电后磁力计融合需要一段时间
	//为了快速融合，前100次使用快速增益
	static INT32U magFastPGainCount = 100;
	
	if ((magFastPGainCount--))  //!ARMING_FLAG(ARMED) && 
		return 10.0f;
	else
		return 1.0f;
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

static void imuMahonyAHRSupdate(FP32 gx, FP32 gy, FP32 gz,
                                FP32 ax, FP32 ay, FP32 az,
                                FP32 mx, FP32 my, FP32 mz,
								BOOLEAN useMag,FP32 dt)
{
	static float integralAccX = 0.0f,  integralAccY = 0.0f, integralAccZ = 0.0f;    //加速度积分误差
	static float integralMagX = 0.0f,  integralMagY = 0.0f, integralMagZ = 0.0f;    //磁力计积分误差
	float ex, ey, ez;

    //计算旋转速率(rad/s)
    const float spin_rate_sq = sq(gx) + sq(gy) + sq(gz);

    //Step 1: Yaw correction
    if (useMag) 
	{
		const float magMagnitudeSq = mx * mx + my * my + mz * mz;
		float kpMag = DCM_KP_MAG * imuMagFastPGainSaleFactor();
		
		if (magMagnitudeSq > 0.01f) 
		{
			//单位化磁力计测量值
			const float magRecipNorm = invSqrt(magMagnitudeSq);
			mx *= magRecipNorm;
			my *= magRecipNorm;
			mz *= magRecipNorm;
		
			//计算X\Y方向的磁通，磁北方向磁通
			const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
			const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
			const float bx = sqrtf(hx * hx + hy * hy);

			//磁力计误差是估计磁北和测量磁北之间的交叉乘积
			const float ez_ef = -(hy * bx);

			//旋转误差到机体坐标系
			ex = rMat[2][0] * ez_ef;
			ey = rMat[2][1] * ez_ef;
			ez = rMat[2][2] * ez_ef;
		}
		else 
		{
			ex = 0;
			ey = 0;
			ez = 0;
		}

		//累计误差补偿
		if (DCM_KI_MAG > 0.0f) 
		{
			//如果旋转速率大于限制值则停止积分
			if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) 
			{
				integralMagX += DCM_KI_MAG * ex * dt;
				integralMagY += DCM_KI_MAG * ey * dt;
				integralMagZ += DCM_KI_MAG * ez * dt;

				gx += integralMagX;
				gy += integralMagY;
				gz += integralMagZ;
			}
		}
		
		//误差补偿
		gx += kpMag * ex;
		gy += kpMag * ey;
		gz += kpMag * ez;
	}

	
    //Step 2: Roll and pitch correction
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		//单位化加速计测量值
		const float accRecipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= accRecipNorm;
		ay *= accRecipNorm;
		az *= accRecipNorm;

		//加速计读取的方向与重力加速计方向的差值，用向量叉乘计算
		ex = (ay * rMat[2][2] - az * rMat[2][1]);
		ey = (az * rMat[2][0] - ax * rMat[2][2]);
		ez = (ax * rMat[2][1] - ay * rMat[2][0]);

		//累计误差补偿
		if (DCM_KI_ACC > 0.0f) 
		{
			//如果旋转速率大于限制值则停止积分
			if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)))
			{
				integralAccX += DCM_KI_ACC * ex * dt;
				integralAccY += DCM_KI_ACC * ey * dt;
				integralAccZ += DCM_KI_ACC * ez * dt;

				gx += integralAccX;
				gy += integralAccY;
				gz += integralAccZ;
			}
		}

		//误差补偿
		gx += DCM_KP_ACC * ex;
		gy += DCM_KP_ACC * ey;
		gz += DCM_KP_ACC * ez;
	}
	
	//一阶近似算法，四元数运动学方程的离散化形式和积分
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    const float qa = q0;
    const float qb = q1;
    const float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

	//单位化四元数
    const float quatRecipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= quatRecipNorm;
    q1 *= quatRecipNorm;
    q2 *= quatRecipNorm;
    q3 *= quatRecipNorm;
    
    //计算四元数的旋转矩阵
    imuComputeRotationMatrix();
}


//更新欧拉角
static void imuUpdateEulerAngles(void)
{
	g_AttitudeCtrlMsg.Roll = RADIANS_TO_DEGREES(MyAtan2Approx(rMat[2][1], rMat[2][2]));//+-180
	g_AttitudeCtrlMsg.Pitch = RADIANS_TO_DEGREES((0.5f * PI) - MyAcosApprox(-rMat[2][0]));//arcsin = 0.5PI - arccos//+-90
	g_AttitudeCtrlMsg.Yaw = RADIANS_TO_DEGREES(MyAtan2Approx(rMat[1][0], rMat[0][0]));//+-180

	if (g_AttitudeCtrlMsg.Yaw < 0.0f)//转换位0~360
		g_AttitudeCtrlMsg.Yaw += 360.0f;

	//更新最小倾角状态
}