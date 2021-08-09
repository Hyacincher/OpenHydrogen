#include "DbgPrint.h"

DebugPrint_t    g_DbgPrintCtrlMsg;

static void UARTSendData(INT8U *Data, INT8U Size);

void DbgPrintInit(void)
{
    memset(&g_DbgPrintCtrlMsg, 0, sizeof(g_DbgPrintCtrlMsg));
    
    LORA433Init();
    
    UART_Start_Receive_IT(&huart4, (INT8U *)g_DbgPrintCtrlMsg.ReceiveBuff, ANO_RXBUFF_LEN);
    //打开接收中断
}

void DbgPrintTask(void)
{
    INT8U Send[50] = {0};
    static INT64U s_SystemTime= 0;
    
    if(g_SysTickTime - s_SystemTime >= 2)
    {
        VisualScope_Output(g_AttitudeCtrlMsg.Roll * 10, g_AttitudeCtrlMsg.Pitch * 10 , \
                            g_PIDCtrlMsg[RATE_ROLL].error , g_PIDCtrlMsg[RATE_ROLL].integ);
        //ANOSendStatus(g_AttitudeCtrlMsg.Roll, g_AttitudeCtrlMsg.Pitch, g_AttitudeCtrlMsg.Yaw, \
                  g_BMPCtrlMsg.Altitude, 0, g_MotorCtrlMsg.State.Unlock);
        //发送阻塞时间太长了
        
        s_SystemTime = g_SysTickTime;
    }
    
    if(g_DbgPrintCtrlMsg.Status.ReceiveOK)
    {
        INT8U DealBuff[ANO_RXBUFF_LEN];
        
        memcpy(DealBuff, (INT8U *)g_DbgPrintCtrlMsg.ReceiveBuff, ANO_RXBUFF_LEN);
        
        g_DbgPrintCtrlMsg.Status.ReceiveOK = 0;
        
        ANOReceive(DealBuff);
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
                
                    if(g_DbgPrintCtrlMsg.ReceiveIndex  > (g_DbgPrintCtrlMsg.ReceiveBuff[ANOLenth] + 2))
                    {//接收完了
                        g_DbgPrintCtrlMsg.ReceiveIndex = 0;
                        g_DbgPrintCtrlMsg.Status.ReceiveOK = 1;
                        g_DbgPrintCtrlMsg.Status.ReceiveStart = 0;
                    }
                    
                    if(g_DbgPrintCtrlMsg.ReceiveIndex >= ANO_RXBUFF_LEN)
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
  
  //UARTSendData(databuf, 10);
  LORASendData((INT8U *)databuf, 10); 
}