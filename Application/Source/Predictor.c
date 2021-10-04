#include "Predictor.h"

volatile PredictorInfo g_PredictorCtrlMsg;

static void UpdateRawHeight(void);
static void UpdateRawIMU(void);
static void UpdatePredictor(void);
static void PublishNewEstimate(void);

static void CorrectPosVeloc(INT16U Axis, FP32 Dt, FP32 Err, FP32 Weight);
static void PredictPosVeloc(INT16U Axis, FP32 Dt, FP32 Acc);

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
            UpdatePredictor();      //更新预估器
            PublishNewEstimate();   //外界发布新的预估数据
        
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
			g_PredictorCtrlMsg.RawData.AccelerationNEU[axis] += \
            (s_AcceCMSS[axis] - g_PredictorCtrlMsg.RawData.AccelerationNEU[axis]) * 0.3f;//一阶低通
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

static void UpdatePredictor(void)
{
    PredictPosVeloc(IMUAxisX, PREDICTOR_UPDATE_DT, g_PredictorCtrlMsg.RawData.AccelerationNEU[IMUAxisX]);
    PredictPosVeloc(IMUAxisY, PREDICTOR_UPDATE_DT, g_PredictorCtrlMsg.RawData.AccelerationNEU[IMUAxisY]);
    PredictPosVeloc(IMUAxisZ, PREDICTOR_UPDATE_DT, g_PredictorCtrlMsg.RawData.AccelerationNEU[IMUAxisZ]);
    
    //加速度偏置值
    const BOOLEAN UpdateAccBias = (WEIGHT_ACC_BIAS > 0);
    FP32 AcceBiasCorr[IMUAxisAll] = {0};
    
    //气压计修正Z轴估算位移与速度
    const FP32 ZError = g_PredictorCtrlMsg.RawData.Height - g_PredictorCtrlMsg.EstimateData.Position[IMUAxisZ];
    CorrectPosVeloc(IMUAxisZ, PREDICTOR_UPDATE_DT, ZError, WEIGHT_Z_BARO);
    if(UpdateAccBias)
    {
        AcceBiasCorr[IMUAxisZ] -= ZError * sq(WEIGHT_Z_BARO);
    }
    
    //修正加速度偏置
    if(UpdateAccBias)
    {
        const FP32 AcceBiasCorrSQ = sq(AcceBiasCorr[IMUAxisX]) + sq(AcceBiasCorr[IMUAxisY]) + sq(AcceBiasCorr[IMUAxisZ]);
        if(AcceBiasCorrSQ < sq(GRAVITY_CMSS * 0.25f))
        {
            TransNEUVectorToBody(AcceBiasCorr);     //偏置转换为机体坐标系
            
            g_PredictorCtrlMsg.RawData.AccelerationBias[IMUAxisX] += AcceBiasCorr[IMUAxisX] * WEIGHT_ACC_BIAS * PREDICTOR_UPDATE_DT;
            g_PredictorCtrlMsg.RawData.AccelerationBias[IMUAxisY] += AcceBiasCorr[IMUAxisY] * WEIGHT_ACC_BIAS * PREDICTOR_UPDATE_DT;
            g_PredictorCtrlMsg.RawData.AccelerationBias[IMUAxisZ] += AcceBiasCorr[IMUAxisZ] * WEIGHT_ACC_BIAS * PREDICTOR_UPDATE_DT;
        }
    }
}

//NEU坐标系加速度估算位移和速度
static void PredictPosVeloc(INT16U Axis, FP32 Dt, FP32 Acc)
{
    g_PredictorCtrlMsg.EstimateData.Position[Axis] += (g_PredictorCtrlMsg.EstimateData.Velocity[Axis] * Dt) \
                                                        + ((Acc * Dt * Dt) / 2.0f);
    g_PredictorCtrlMsg.EstimateData.Velocity[Axis] += Acc * Dt;
}

//实际传感器修正位移和速度
static void CorrectPosVeloc(INT16U Axis, FP32 Dt, FP32 Err, FP32 Weight)
{
    FP32 EWDT = Err * Weight * Dt;
    
    g_PredictorCtrlMsg.EstimateData.Position[Axis] += EWDT;
    g_PredictorCtrlMsg.EstimateData.Velocity[Axis] += Weight * EWDT;
}

static void PublishNewEstimate(void)
{
    static INT64U s_PublishTime;
    
    //更新世界坐标系加速度
    g_AttitudeCtrlMsg.NEUAcce[IMUAxisX] = g_PredictorCtrlMsg.RawData.AccelerationNEU[IMUAxisX];
    g_AttitudeCtrlMsg.NEUAcce[IMUAxisY] = g_PredictorCtrlMsg.RawData.AccelerationNEU[IMUAxisY];
    g_AttitudeCtrlMsg.NEUAcce[IMUAxisZ] = g_PredictorCtrlMsg.RawData.AccelerationNEU[IMUAxisZ];
    
    //更新预估的位置与速度
    if(g_SysTickTime - s_PublishTime >= 10)
    {
        g_HeightCtrlMsg.Position = g_PredictorCtrlMsg.EstimateData.Position[IMUAxisZ];
        g_HeightCtrlMsg.Velocity = g_PredictorCtrlMsg.EstimateData.Velocity[IMUAxisZ];
        
        g_HeightCtrlMsg.Velocity = MyConstrainF(g_HeightCtrlMsg.Velocity, -150.0f, 150.0f); //Z轴速度限幅
        
        
        s_PublishTime = g_SysTickTime;
    }
}

