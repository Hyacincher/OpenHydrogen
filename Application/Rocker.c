#include "Rocker.h"

volatile RockerInfo_t   g_RockerCtrlMsg;

static void GetOriginRocker(void);
static void RockerTransform(void);

extern void UpdateRockerTrans(INT16S Throttle, INT16S Yaw, INT16S Picth, INT16S Roll);

void RockerInit(void)
{
    memset((void *)&g_RockerCtrlMsg, 0, sizeof(g_RockerCtrlMsg));
    
    //FLASH参数读取
    g_RockerCtrlMsg.Throttle.Max = 2870;//1598  2870  325
    g_RockerCtrlMsg.Throttle.Mid = 1635;
    g_RockerCtrlMsg.Throttle.Min = 325;

    g_RockerCtrlMsg.Yaw.Max = 2856; //1668   2856   438
    g_RockerCtrlMsg.Yaw.Mid = 1668;
    g_RockerCtrlMsg.Yaw.Min = 438;
    
    g_RockerCtrlMsg.Roll.Max = 2984;//1836 2984 472
    g_RockerCtrlMsg.Roll.Mid = 1836;
    g_RockerCtrlMsg.Roll.Min = 472;
    
    g_RockerCtrlMsg.Pitch.Max = 3013;//1744  3013   364
    g_RockerCtrlMsg.Pitch.Mid = 1744;    
    g_RockerCtrlMsg.Pitch.Min = 364;
}

void RockerTask(void)
{
    switch(g_RockerCtrlMsg.TaskStage)
    {
        case 0:
            g_RockerCtrlMsg.TaskTimer = g_SystemTime;
            g_RockerCtrlMsg.TaskStage++;
            break;
        case 1:
            if((g_SystemTime - g_RockerCtrlMsg.TaskTimer) > 0)
            {
                GetOriginRocker();
                RockerTransform();
                UpdateRockerTrans(g_RockerCtrlMsg.Throttle.Value, g_RockerCtrlMsg.Yaw.Value, \
                                    g_RockerCtrlMsg.Pitch.Value, g_RockerCtrlMsg.Roll.Value);
                g_RockerCtrlMsg.TaskStage++;
            }
            break;
        case 2:
            g_RockerCtrlMsg.TaskStage = 0;
            break;
        default:
            g_RockerCtrlMsg.TaskStage = 0;
            g_RockerCtrlMsg.TaskTimer = g_SystemTime;
            break;
    }
}

void GetOriginRocker(void)
{
    g_RockerCtrlMsg.Throttle.Volt = g_ADC1Volt[L_AIO_0];
    g_RockerCtrlMsg.Yaw.Volt = g_ADC1Volt[L_AIO_1];
    g_RockerCtrlMsg.Pitch.Volt = g_ADC2Volt[R_AIO_2];
    g_RockerCtrlMsg.Roll.Volt = g_ADC2Volt[R_AIO_1];
}

static void RockerTransform(void)
{
    //从上到下递增，从左往右递增
    
    /* Throttle */
    if(g_RockerCtrlMsg.Throttle.Volt <= g_RockerCtrlMsg.Throttle.Mid)
    {
        g_RockerCtrlMsg.Throttle.Rate = (g_RockerCtrlMsg.Throttle.Volt - g_RockerCtrlMsg.Throttle.Min) \
                                        / (FP32)(g_RockerCtrlMsg.Throttle.Mid - g_RockerCtrlMsg.Throttle.Min) / 2;
    }
    else
    {
        g_RockerCtrlMsg.Throttle.Rate = ((g_RockerCtrlMsg.Throttle.Volt - g_RockerCtrlMsg.Throttle.Mid)\
                                        / (FP32)(g_RockerCtrlMsg.Throttle.Max - g_RockerCtrlMsg.Throttle.Mid) / 2) + 0.5f;
    }
    g_RockerCtrlMsg.Throttle.Rate = 0.5 - g_RockerCtrlMsg.Throttle.Rate; //转换递增方向，并转换成-0.5~+0.5
    g_RockerCtrlMsg.Throttle.Rate = MyConstrainF(g_RockerCtrlMsg.Throttle.Rate, -0.5f, 0.5f);
    g_RockerCtrlMsg.Throttle.Value = g_RockerCtrlMsg.Throttle.Rate * ROCKER_ALL_RANGE;
    g_RockerCtrlMsg.Throttle.Value = MyApplyDeadBand(g_RockerCtrlMsg.Throttle.Value, ROCKER_MID_VALUE, ROCKER_DEAD_BAND);
    g_RockerCtrlMsg.Throttle.Value = MyConstrainINT32S(g_RockerCtrlMsg.Throttle.Value, ROCKER_MIN_VALUE, ROCKER_MAX_VALUE);
    
    /* Yaw */
    if(g_RockerCtrlMsg.Yaw.Volt <= g_RockerCtrlMsg.Yaw.Mid)
    {
        g_RockerCtrlMsg.Yaw.Rate = (g_RockerCtrlMsg.Yaw.Volt - g_RockerCtrlMsg.Yaw.Min) \
                                        / (FP32)(g_RockerCtrlMsg.Yaw.Mid - g_RockerCtrlMsg.Yaw.Min) / 2;
    }
    else
    {
        g_RockerCtrlMsg.Yaw.Rate = ((g_RockerCtrlMsg.Yaw.Volt - g_RockerCtrlMsg.Yaw.Mid)\
                                        / (FP32)(g_RockerCtrlMsg.Yaw.Max - g_RockerCtrlMsg.Yaw.Mid) / 2) + 0.5f;
    }
    g_RockerCtrlMsg.Yaw.Rate -= 0.5;
    g_RockerCtrlMsg.Yaw.Rate = MyConstrainF(g_RockerCtrlMsg.Yaw.Rate, -0.5f, 0.5f);
    g_RockerCtrlMsg.Yaw.Value = g_RockerCtrlMsg.Yaw.Rate * ROCKER_ALL_RANGE;
    g_RockerCtrlMsg.Yaw.Value = MyApplyDeadBand(g_RockerCtrlMsg.Yaw.Value, ROCKER_MID_VALUE, ROCKER_DEAD_BAND);
    g_RockerCtrlMsg.Yaw.Value = MyConstrainINT32S(g_RockerCtrlMsg.Yaw.Value, ROCKER_MIN_VALUE, ROCKER_MAX_VALUE);
    
    /* Pitch */
    if(g_RockerCtrlMsg.Pitch.Volt <= g_RockerCtrlMsg.Pitch.Mid)
    {
        g_RockerCtrlMsg.Pitch.Rate = (g_RockerCtrlMsg.Pitch.Volt - g_RockerCtrlMsg.Pitch.Min) \
                                        / (FP32)(g_RockerCtrlMsg.Pitch.Mid - g_RockerCtrlMsg.Pitch.Min) / 2;
    }
    else
    {
        g_RockerCtrlMsg.Pitch.Rate = ((g_RockerCtrlMsg.Pitch.Volt - g_RockerCtrlMsg.Pitch.Mid)\
                                        / (FP32)(g_RockerCtrlMsg.Pitch.Max - g_RockerCtrlMsg.Pitch.Mid) / 2) + 0.5f;
    }
    g_RockerCtrlMsg.Pitch.Rate = 0.5 - g_RockerCtrlMsg.Pitch.Rate; //转换递增方向
    g_RockerCtrlMsg.Pitch.Rate = MyConstrainF(g_RockerCtrlMsg.Pitch.Rate, -0.5f, 0.5f);
    g_RockerCtrlMsg.Pitch.Value = g_RockerCtrlMsg.Pitch.Rate * ROCKER_ALL_RANGE;
    g_RockerCtrlMsg.Pitch.Value = MyApplyDeadBand(g_RockerCtrlMsg.Pitch.Value, ROCKER_MID_VALUE, ROCKER_DEAD_BAND);
    g_RockerCtrlMsg.Pitch.Value = MyConstrainINT32S(g_RockerCtrlMsg.Pitch.Value, ROCKER_MIN_VALUE, ROCKER_MAX_VALUE);
    
    /* Roll */
    if(g_RockerCtrlMsg.Roll.Volt <= g_RockerCtrlMsg.Roll.Mid)
    {
        g_RockerCtrlMsg.Roll.Rate = (g_RockerCtrlMsg.Roll.Volt - g_RockerCtrlMsg.Roll.Min) \
                                        / (FP32)(g_RockerCtrlMsg.Roll.Mid - g_RockerCtrlMsg.Roll.Min) / 2;
    }
    else
    {
        g_RockerCtrlMsg.Roll.Rate = ((g_RockerCtrlMsg.Roll.Volt - g_RockerCtrlMsg.Roll.Mid)\
                                        / (FP32)(g_RockerCtrlMsg.Roll.Max - g_RockerCtrlMsg.Roll.Mid) / 2) + 0.5f;
    }
    g_RockerCtrlMsg.Roll.Rate -= 0.5;
    g_RockerCtrlMsg.Roll.Rate = MyConstrainF(g_RockerCtrlMsg.Roll.Rate, -0.5f, 0.5f);
    g_RockerCtrlMsg.Roll.Value = g_RockerCtrlMsg.Roll.Rate * ROCKER_ALL_RANGE;
    g_RockerCtrlMsg.Roll.Value = MyApplyDeadBand(g_RockerCtrlMsg.Roll.Value, ROCKER_MID_VALUE, ROCKER_DEAD_BAND);
    g_RockerCtrlMsg.Roll.Value = MyConstrainINT32S(g_RockerCtrlMsg.Roll.Value, ROCKER_MIN_VALUE, ROCKER_MAX_VALUE);
}

void SetRockerMode(INT8U Index, BOOLEAN Status)
{
    if(Index == 0)
    {//左摇杆
        g_RockerCtrlMsg.Status.LeftRockerMode = Status;
    }
    else if(Index == 1)
    {//右摇杆
        g_RockerCtrlMsg.Status.RightRockerMode = Status;
    }
}
