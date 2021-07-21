#include "ANOHost.h"

#define ANO_VERSION "V4.01"

static INT8U GetHighByte(INT16S Data);
static INT8U GetLowByte(INT16S Data);
static INT8U GetSum(INT8U* Buff, INT16U Len);

extern void UpdateRocker(INT16U Roll, INT16U Pitch, INT16U Yaw, INT16U Throttle);

static INT8U GetHighByte(INT16S Data)
{
    return (Data>>8);
}

static INT8U GetLowByte(INT16S Data)
{
    return (Data&0xff);
}

static INT8U GetSum(INT8U* Buff, INT16U Len)
{
    INT16U ii;
    INT32U Sum = 0;
    
    if(Buff == NULL)
    {
        return 0;
    }
    
    for(ii = 0 ; ii < Len ; ii++)
    {
        Sum += Buff[ii];
    }
    
    return (INT8U)Sum;
}

void ANOSendStatus(FP32 Roll, FP32 Pitch, FP32 Yaw, FP32 Altitude, INT8U Flymode, INT8U Armed)
{
    INT8U Send[50] = {0};
    
    Send[0] = 0xAA;
    Send[1] = 0xAA;
    Send[2] = 0x01;//Function
    Send[3] = 0x0C;//LEN
    Send[4] = GetHighByte((INT16S)(-Roll * 100));
    Send[5] = GetLowByte((INT16S)(-Roll * 100));
    Send[6] = GetHighByte((INT16S)(Pitch * 100));
    Send[7] = GetLowByte((INT16S)(Pitch * 100));
    Send[8] = GetHighByte((INT16S)(Yaw * 100));
    Send[9] = GetLowByte((INT16S)(Yaw * 100));
    Send[10] = GetHighByte((INT16S)((INT32S)(Altitude * 100) >> 16));   //cm
    Send[11] = GetLowByte((INT16S)((INT32S)(Altitude * 100) >> 16));
    Send[12] = GetHighByte((INT16S)(Altitude * 100));
    Send[13] = GetLowByte((INT16S)(Altitude * 100));
    Send[14] = Flymode;//Fly mode
    Send[15] = Armed;//Armed
    Send[16] = GetSum(Send, 15);//Check
    LORASendData((INT8U *)Send, 17); 
}

void ANOSendCheck(INT8U Func, INT8U CheckSum)
{
    INT8U Send[ANO_RXBUFF_LEN];
    
    Send[0] = 0xAA;
    Send[1] = 0xAA;
    Send[2] = 0xEF;//Check
    Send[3] = 0x02;
    Send[4] = Func;
    Send[5] = CheckSum;
    Send[6] = GetSum(Send, 6);//Check
    LORASendData((INT8U *)Send, 7);
}

void ANOSendPID(PIDInfo *PID1, PIDInfo *PID2, PIDInfo *PID3)
{
    INT8U Send[ANO_RXBUFF_LEN];
    
    Send[0] = 0xAA;
    Send[1] = 0xAA;
    Send[2] = 0x10;
    Send[3] = 0x12;
    Send[4] = GetHighByte((INT16U)(PID1->kp * 1000));
    Send[5] = GetLowByte((INT16U)(PID1->kp * 1000));
    Send[6] = GetHighByte((INT16U)(PID1->ki * 1000));
    Send[7] = GetLowByte((INT16U)(PID1->ki * 1000));
    Send[8] = GetHighByte((INT16U)(PID1->kd * 1000));
    Send[9] = GetLowByte((INT16U)(PID1->kd * 1000));
    
    Send[10] = GetHighByte((INT16U)(PID2->kp * 1000));
    Send[11] = GetLowByte((INT16U)(PID2->kp * 1000));
    Send[12] = GetHighByte((INT16U)(PID2->ki * 1000));
    Send[13] = GetLowByte((INT16U)(PID2->ki * 1000));
    Send[14] = GetHighByte((INT16U)(PID2->kd * 1000));
    Send[15] = GetLowByte((INT16U)(PID2->kd * 1000));

    Send[16] = GetHighByte((INT16U)(PID3->kp * 1000));
    Send[17] = GetLowByte((INT16U)(PID3->kp * 1000));
    Send[18] = GetHighByte((INT16U)(PID3->ki * 1000));
    Send[19] = GetLowByte((INT16U)(PID3->ki * 1000));
    Send[20] = GetHighByte((INT16U)(PID3->kd * 1000));
    Send[21] = GetLowByte((INT16U)(PID3->kd * 1000));
    
    Send[22] = GetSum(Send, 22);
    LORASendData((INT8U *)Send, 23);    
}

void ANOReceive(INT8U Buff[ANO_RXBUFF_LEN])
{
    //带进来无帧头数据
    
    if(Buff[ANOFunc] == 1)
    {//CONMAND
        switch(Buff[ANOData])
        {
            case 0xA0:
                MotorLock();
                break;
            case 0xA1:
                MotorUnLock();
                break;
        }
    }
    else if(Buff[ANOFunc] == 2)
    {//CONMAND
        INT16U ii;
        
        switch(Buff[ANOData])
        {
            case 0x01:
                //ANOSendPID(&g_PIDCtrlMsg[RATE_ROLL], &g_PIDCtrlMsg[RATE_PITCH], &g_PIDCtrlMsg[RATE_YAW]);
                //LORA无响应
                break;
        }
    }
    else if(Buff[ANOFunc] == 0x10)
    {//角速度PID
        INT16U PIDTable[9];
        INT16U ii;
        
        for(ii = 0 ; ii < 9 ; ii++)
        {
            PIDTable[ii] = INT8UToINT16U(Buff + ANOData + (ii * 2));
        }
        for(ii = RATE_ROLL ; ii <= RATE_YAW ; ii++)
        {
            g_PIDCtrlMsg[ii].kp = (FP32)PIDTable[(ii * 3) + 0] / 1000;
            g_PIDCtrlMsg[ii].ki = (FP32)PIDTable[(ii * 3) + 1] / 1000;
            g_PIDCtrlMsg[ii].kd = (FP32)PIDTable[(ii * 3) + 2] / 1000;                
        }
        ANOSendCheck(0x10, Buff[20]);
    }
    else if(Buff[ANOFunc] == 0x11)
    {//角度PID
        INT16U PIDTable[9];
        INT16U ii;
        
        for(ii = 0 ; ii < 9 ; ii++)
        {
            PIDTable[ii] = INT8UToINT16U(Buff + ANOData + (ii * 2));
        }
        for(ii = ANGLE_ROLL ; ii <= ANGLE_YAW ; ii++)
        {
            g_PIDCtrlMsg[ii].kp = (FP32)PIDTable[((ii - ANGLE_ROLL) * 3) + 0] / 1000;
            g_PIDCtrlMsg[ii].ki = (FP32)PIDTable[((ii - ANGLE_ROLL) * 3) + 1] / 1000;
            g_PIDCtrlMsg[ii].kd = (FP32)PIDTable[((ii - ANGLE_ROLL) * 3) + 2] / 1000;                
        }
        ANOSendCheck(0x11, Buff[20]);
    }
    else if(Buff[ANOFunc] == 0x12)
    {
        ANOSendCheck(0x12, Buff[20]);
    }
    else if(Buff[ANOFunc] == 0x13)
    {
        ANOSendCheck(0x13, Buff[20]);
    }   
    else if(Buff[ANOFunc] == 0x14)
    {
        ANOSendCheck(0x14, Buff[20]);
    }
    else if(Buff[ANOFunc] == 0x15)
    {
        ANOSendCheck(0x15, Buff[20]);
    }
    else if(Buff[ANOFunc] == 0xF1)
    {
        INT16U RockerTable[4];
        INT16U ii;
        
        for(ii = 0 ; ii < 4 ; ii++)
        {
            RockerTable[ii] = INT8UToINT16U(Buff + ANOData + (ii * 2));
        }
        
        UpdateRocker(RockerTable[0], RockerTable[1], RockerTable[2], RockerTable[3]);
    }
}
