#include "DbgPrint.h"

DebugPrint_t    g_DbgPrintCtrlMsg;

static INT8U GetHighByte(INT16S Data);
static INT8U GetLowByte(INT16S Data);
static INT8U GetSum(INT8U* Buff, INT16U Len);
static void UARTSendData(INT8U *Data, INT8U Size);
static void DbgSendData(SendMethod_e Send, INT8U *Data, INT8U Size);

void DbgPrintInit(void)
{
    memset(&g_DbgPrintCtrlMsg, 0, sizeof(g_DbgPrintCtrlMsg));
}

void DbgPrintTask(void)
{
    INT8U Send[50] = {0};
    static INT64U s_SystemTime,s_SystemTime2,s_SystemTime3 = 0;
    
    //暂时用ANO协议，但是不要用上位机，lora无线收发切换不过来，没法用上位机
    
//    if(g_SysTickTime - s_SystemTime2 >= 5)
//    {//收发同事进行Lora反应补过来
//    #ifdef ANO_PRINT
//        Send[0] = 0xAA;
//        Send[1] = 0xAA;
//        Send[2] = 0x01;//Function
//        Send[3] = 0x0C;//LEN
//        Send[4] = GetHighByte((INT16S)(-g_AttitudeCtrlMsg.Roll * 100));
//        Send[5] = GetLowByte((INT16S)(-g_AttitudeCtrlMsg.Roll * 100));
//        Send[6] = GetHighByte((INT16S)(g_AttitudeCtrlMsg.Pitch * 100));
//        Send[7] = GetLowByte((INT16S)(g_AttitudeCtrlMsg.Pitch * 100));
//        Send[8] = 0;//Yaw   //GetHighByte((INT16S)(g_LSMCtrLMsg.Yaw * 100))
//        Send[9] = 0;         //GetLowByte((INT16S)(g_LSMCtrLMsg.Yaw * 100))
//        Send[10] = 0x00;//Altitude
//        Send[11] = 0x00;
//        Send[12] = 0x00;
//        Send[13] = 0x00;
//        Send[14] = 0x00;//Fly mode
//        Send[15] = g_MotorCtrlMsg.State.Unlock;//Armed
//        Send[16] = GetSum(Send, 15);//Check
//        DbgSendData(WireLess, (INT8U *)Send, 17);
//    #endif
//        s_SystemTime2 = g_SysTickTime;
//    }
    if(g_SysTickTime - s_SystemTime >= 1)
    {
    #ifdef VISUAL_SCOPE_PRINT
        VisualScope_Output(g_AttitudeCtrlMsg.Roll * 10, g_AttitudeCtrlMsg.Pitch * 10 , \
                            g_StabiliCtrlMsg.AngleOutRoll * 10, g_StabiliCtrlMsg.AngleOutPitch * 10);
    #endif   
    
    s_SystemTime = g_SysTickTime;
    }
    
    if(g_DbgPrintCtrlMsg.Status.ReceiveOK)
    {
        INT8U DealBuff[RECEIVE_BUFF_LEN];
        
        memcpy(DealBuff, (INT8U *)g_DbgPrintCtrlMsg.ReceiveBuff, RECEIVE_BUFF_LEN);
        
        g_DbgPrintCtrlMsg.Status.ReceiveOK = 0;
        
        if(DealBuff[Function] == 1)
        {//CONMAND
            switch(DealBuff[Data])
            {
                case 0xA0:
                    MotorLock();
                    break;
                case 0xA1:
                    MotorUnLock();
                    break;
            }
        }
        else if(DealBuff[Function] == 0x10)
        {//角速度PID
            INT16U PIDTable[9];
            INT16U ii;
            
            for(ii = 0 ; ii < 9 ; ii++)
            {
                PIDTable[ii] = INT8UToINT16U(DealBuff + Data + (ii * 2));
            }
            for(ii = RATE_ROLL ; ii <= RATE_YAW ; ii++)
            {
                g_PIDCtrlMsg[ii].kp = (FP32)PIDTable[(ii * 3) + 0] / 1000;
                g_PIDCtrlMsg[ii].ki = (FP32)PIDTable[(ii * 3) + 1] / 1000;
                g_PIDCtrlMsg[ii].kd = (FP32)PIDTable[(ii * 3) + 2] / 1000;                
            }
            
            //lora收发切换没那么快  //手动发
//            Send[0] = 0xAA;
//            Send[1] = 0xAA;
//            Send[2] = 0xEF;//Check
//            Send[3] = 0x02;
//            Send[4] = 0x10;
//            Send[5] = DealBuff[20];
//            Send[6] = GetSum(Send, 6);//Check
//            DbgSendData(WireLess, (INT8U *)Send, 7);
        }
        else if(DealBuff[Function] == 0x11)
        {//角度PID
            INT16U PIDTable[9];
            INT16U ii;
            
            for(ii = 0 ; ii < 9 ; ii++)
            {
                PIDTable[ii] = INT8UToINT16U(DealBuff + Data + (ii * 2));
            }
            for(ii = ANGLE_ROLL ; ii <= ANGLE_YAW ; ii++)
            {
                g_PIDCtrlMsg[ii].kp = (FP32)PIDTable[(ii * 3) + 0] / 1000;
                g_PIDCtrlMsg[ii].ki = (FP32)PIDTable[(ii * 3) + 1] / 1000;
                g_PIDCtrlMsg[ii].kd = (FP32)PIDTable[(ii * 3) + 2] / 1000;                
            }
        }
    }
}

static void DbgSendData(SendMethod_e Send, INT8U *Data, INT8U Size)
{
    if(Send == Wired)
    {
        UARTSendData((INT8U *)Data, Size);
    }
    else if(Send == WireLess)
    {
        LORASendData((INT8U *)Data, Size);
    }
}

/*
*函数名：   UART_Transmit
*功  能：   串口发送字符串
*输入参数： huart       串口句柄
            Data        字符串指针
            Size        字符串发送长度
*返回参数： none
*/
static void UARTSendData(INT8U *Data, INT8U Size)
{
    HAL_UART_Transmit(&huart6 , Data , Size , 5);
}

void UART6ReceiveData(INT8U Size)
{

}

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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart4)
    {
//        g_DbgPrintCtrlMsg.Status.ReceiveOK = 1;
//        g_DbgPrintCtrlMsg.ReceiveLen = Size;
//        UART_Start_Receive_IT(&huart4, g_DbgPrintCtrlMsg.ReceiveBuff, RECEIVE_BUFF_LEN);
        //LORA回复数据间隔太长，每个byte都会触发IDLE
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    INT16U Clear;
    
    if(huart == &huart4)
    {
        if(huart->ErrorCode == HAL_UART_ERROR_ORE)
        {
            Clear = huart->Instance->SR;
            Clear = huart->Instance->DR;
            __HAL_UART_CLEAR_OREFLAG(huart);       
        }
    }
}

void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(huart == &huart4)
    {
        static INT16U FrameHead = 0;
        INT8U Data;
        
        if(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
        {
            __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
            
            Data = huart->Instance->DR;

            if(g_DbgPrintCtrlMsg.Status.ReceiveOK == 0)
            {//处理了帧才能接收下一帧
                if(g_DbgPrintCtrlMsg.Status.ReceiveStart)
                {
                    g_DbgPrintCtrlMsg.ReceiveBuff[g_DbgPrintCtrlMsg.ReceiveIndex] = Data;
                    g_DbgPrintCtrlMsg.ReceiveIndex++;
                
                    if(g_DbgPrintCtrlMsg.ReceiveIndex  > (g_DbgPrintCtrlMsg.ReceiveBuff[Lenth] + 2))
                    {//接收完了
                        g_DbgPrintCtrlMsg.ReceiveIndex = 0;
                        g_DbgPrintCtrlMsg.Status.ReceiveOK = 1;
                        g_DbgPrintCtrlMsg.Status.ReceiveStart = 0;
                    }
                    
                    if(g_DbgPrintCtrlMsg.ReceiveIndex >= RECEIVE_BUFF_LEN)
                    {
                        g_DbgPrintCtrlMsg.Status.ReceiveOK = 0;
                        g_DbgPrintCtrlMsg.Status.ReceiveStart = 0;
                    }
                }
                else
                {
                    FrameHead <<= 8;
                    FrameHead |= Data;
                    if(FrameHead == 0xAAAF)
                    {
                        g_DbgPrintCtrlMsg.Status.ReceiveStart = 1;
                    }
                }
            }
        }
    }
}

/*---------------------------Visual Scope---------------------------------*/
/*
**函数信息      CRC_CHECK 
**功能描述      CRC校验检查
**输入参数      Buf         待校验的数据
                CRC_CNT     接收到的CRC
**输出参数      CRC计算出的校验码
*/
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    int i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xA001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

/*
**函数信息      VisualScope_Output 
**功能描述      VisualScope数据输出
**输入参数      
**输出参数      CRC计算出的校验码
*/
void VisualScope_Output(float data1 ,float data2 ,float data3 ,float data4)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  int i;
  unsigned short CRC16 = 0;

  temp[0] = (int)data1;
  temp[1] = (int)data2;
  temp[2] = (int)data3;
  temp[3] = (int)data4;

  temp1[0] = (unsigned int)temp[0] ;
  temp1[1] = (unsigned int)temp[1];
  temp1[2] = (unsigned int)temp[2];
  temp1[3] = (unsigned int)temp[3];
  
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256; 
  
  DbgSendData(Wired, databuf, 10);
}