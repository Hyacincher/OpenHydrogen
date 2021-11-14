#include "Predictor2.h"

volatile PredictorInfo g_PredictorCtrlMsg;

static void UpdateRawHeight(void);
static void UpdateRawIMU(void);
static void UpdatePredictorZ(void);

static FP32 IntegralRateZ[3]={
0.02,   //ACC 0.03
0.025,  //VEL 0.05  越大速度越快，位置过冲越厉害
0.02    //POS 0.02  越大跟随真实传感器收敛就越快
};

static FP32 AltitudeErr[IMUAxisAll] = {0,0,0};
static FP32 AccCorrect[IMUAxisAll] = {0,0,0};
static FP32 VelCorrect[IMUAxisAll] = {0,0,0};
static FP32 PosCorrect[IMUAxisAll] = {0,0,0};
static FP32 EstiAcc[IMUAxisAll] = {0,0,0};
static FP32 VelDelta[IMUAxisAll] = {0,0,0};  
static FP32 OriginVel[IMUAxisAll] = {0,0,0};
static FP32 OriginPos[IMUAxisAll] = {0,0,0};

void PredictorInit(void)
{
    INT16U ii;
    
    memset((void *)&g_PredictorCtrlMsg, 0, sizeof(g_PredictorCtrlMsg));
    
}

void PredictorTask(void)
{
    static INT64U s_SystemTime = 0;
    static INT8U s_PredictorStage = 0;
    
    switch(s_PredictorStage)
    {
        case 0:
            s_SystemTime = g_SysTickTime;
            UpdateRawHeight();      //更新高度数据用于融合
            UpdateRawIMU();         //计算IMU数据用于预估
            UpdatePredictorZ();      //更新预估器
        
            s_PredictorStage++;
            break;
        case 1:
            if(g_SysTickTime - s_SystemTime >= 2)
            {//500hz
                s_PredictorStage++;
            }
            break;
        case 2:
            s_PredictorStage = 0;
            break;
        default:
            s_PredictorStage = 0;
            break;
    }
}

static void UpdateRawHeight(void)
{
    if(g_HeightCtrlMsg.Status.HeightIsStable)
    {
        g_PredictorCtrlMsg.RawData.Height = g_HeightCtrlMsg.Altitude * 100;
    }
    else
    {
        g_PredictorCtrlMsg.RawData.Height = 0;
    }
}

static void UpdateRawIMU(void)
{
    static FP32 s_CaliGravityCMSS = GRAVITY_CMSS;
    static INT64U s_GravityCaliTime = 0;
    FP32 s_AcceCMSS[IMUAxisAll];
    
    //原始机体加速度转换单位（cm/s2）
    s_AcceCMSS[IMUAxisX] = g_AttitudeCtrlMsg.NormailAcce[IMUAxisX] * GRAVITY_CMSS;
    s_AcceCMSS[IMUAxisY] = g_AttitudeCtrlMsg.NormailAcce[IMUAxisY] * GRAVITY_CMSS;
    s_AcceCMSS[IMUAxisZ] = g_AttitudeCtrlMsg.NormailAcce[IMUAxisZ] * GRAVITY_CMSS;
    
    //去除加速度偏置
    s_AcceCMSS[IMUAxisX] -= g_PredictorCtrlMsg.RawData.AccelerationBias[IMUAxisX];
    s_AcceCMSS[IMUAxisY] -= g_PredictorCtrlMsg.RawData.AccelerationBias[IMUAxisY];
    s_AcceCMSS[IMUAxisZ] -= g_PredictorCtrlMsg.RawData.AccelerationBias[IMUAxisZ];
    
    TransBodyVectorToNEU(s_AcceCMSS);   //本质是获得平行于地面（垂直地心）的XY加速度以及垂直于地面的Z加速度
    
    if(!g_PredictorCtrlMsg.Status.GravityCaliIsCplt && g_AttitudeCtrlMsg.Status.SmallAngle)
    {
        //收敛加速度偏执去除重力加速度
        const FP32 GravityOffsetErr = s_AcceCMSS[IMUAxisZ] - s_CaliGravityCMSS;
        s_CaliGravityCMSS += GravityOffsetErr * 0.0025f;
        
		if (MyFP32Abs(GravityOffsetErr) < 5)//误差要小于5cm/ss
		{
			if ((g_SysTickTime - s_GravityCaliTime) > 250) 
			{
				g_PredictorCtrlMsg.Status.GravityCaliIsCplt = 1;
			}
		}
		else 
		{
			g_PredictorCtrlMsg.Status.GravityCaliIsCplt = 0;
            s_GravityCaliTime = g_SysTickTime;
		}
    }
    
	//NEU坐标系加速度处理
	if (g_PredictorCtrlMsg.Status.GravityCaliIsCplt) 
	{
		s_AcceCMSS[IMUAxisZ] -= s_CaliGravityCMSS;//去除重力
		for (INT16U axis = 0; axis < 3; axis++)
		{
			s_AcceCMSS[axis] = ApplyDeadBand(s_AcceCMSS[axis], 4);//去除4(cm/ss)死区
            g_PredictorCtrlMsg.RawData.AccelerationNEU[axis] = s_AcceCMSS[axis];
		}
	}
	else 
	{
        for (INT16U axis = 0; axis < 3; axis++)
		{
            g_PredictorCtrlMsg.RawData.AccelerationNEU[axis] = 0;
        }
	}
}

void UpdatePredictorZ(void)
{
    AltitudeErr[IMUAxisZ] = g_PredictorCtrlMsg.RawData.Height - g_PredictorCtrlMsg.EstiData.Position[IMUAxisZ];

    AccCorrect[IMUAxisZ] += IntegralRateZ[0] * AltitudeErr[IMUAxisZ] * K_ACC_Z;
    VelCorrect[IMUAxisZ] += IntegralRateZ[1] * AltitudeErr[IMUAxisZ] * K_VEL_Z;
    PosCorrect[IMUAxisZ] += IntegralRateZ[2] * AltitudeErr[IMUAxisZ] * K_POS_Z;
    
    EstiAcc[IMUAxisZ] = g_PredictorCtrlMsg.RawData.AccelerationNEU[IMUAxisZ] + AccCorrect[IMUAxisZ];

    VelDelta[IMUAxisZ] = EstiAcc[IMUAxisZ] * PREDICTOR_UPDATE_DT;

    OriginPos[IMUAxisZ] += (g_PredictorCtrlMsg.EstiData.Velocity[IMUAxisZ] + 0.5 * VelDelta[IMUAxisZ]) * PREDICTOR_UPDATE_DT;

    g_PredictorCtrlMsg.EstiData.Position[IMUAxisZ] = OriginPos[IMUAxisZ] + PosCorrect[IMUAxisZ];

    OriginVel[IMUAxisZ] += VelDelta[IMUAxisZ];
    
    g_PredictorCtrlMsg.EstiData.Velocity[IMUAxisZ] = OriginVel[IMUAxisZ] + VelCorrect[IMUAxisZ];
    
    //更新世界坐标系加速度
    g_AttitudeCtrlMsg.NEUAcce[IMUAxisX] = g_PredictorCtrlMsg.RawData.AccelerationNEU[IMUAxisX];
    g_AttitudeCtrlMsg.NEUAcce[IMUAxisY] = g_PredictorCtrlMsg.RawData.AccelerationNEU[IMUAxisY];
    g_AttitudeCtrlMsg.NEUAcce[IMUAxisZ] = g_PredictorCtrlMsg.RawData.AccelerationNEU[IMUAxisZ];
    
    //更新预估的位置与速度
    g_HeightCtrlMsg.Height = g_PredictorCtrlMsg.EstiData.Position[IMUAxisZ];
    g_HeightCtrlMsg.Velocity = MyConstrainF(g_PredictorCtrlMsg.EstiData.Velocity[IMUAxisZ], -150.0f, 150.0f); //Z轴速度限幅
}

