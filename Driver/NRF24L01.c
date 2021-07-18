#include "NRF24L01.h"

NRFInfo g_NRFCtrlMsg;

static const INT8U TXAddress[TX_ADR_WIDTH]={0x82,0x66,0x97,0x13,0x30};
static const INT8U RXAddress[TX_ADR_WIDTH]={0x82,0x66,0x97,0x13,0x30};

inline static INT8U NRFWriteRead(INT8U Data)
{
    INT8U Receive;
    
    HAL_SPI_TransmitReceive(&hspi2, &Data, &Receive, 1, 1000);
    
    return Receive;
}

static void NRFWriteReg(INT8U Reg, INT8U Data)
{
    NRF_ENABLE();
    HAL_SPI_Transmit(&hspi2, &Reg, 1, 1000);
    HAL_SPI_Transmit(&hspi2, &Data, 1, 1000);
    NRF_DISABLE();
}

static INT8U NRFReadReg(INT8U Reg)
{
    INT8U Transmit[2]={0};
    INT8U Receive[2]={0};
    
    Transmit[0] = Reg;
    Transmit[1] = 0xff;
    
    NRF_ENABLE();
    HAL_SPI_TransmitReceive(&hspi2, Transmit, Receive, 2, 1000);
    NRF_DISABLE();
    
    return Receive[1];
}

static void LSMReadContinul(INT8U Reg, INT8U *pRxBuff, INT16U Len)
{
    INT8U Transmit;
    INT16U ii;
    
    Transmit = Reg;
    
    NRF_ENABLE();
    HAL_SPI_Transmit(&hspi2, &Transmit, 1, 1000);
    for(ii = 0 ; ii < Len ; ii++)
    {
        pRxBuff[ii] = NRFWriteRead(0xff);
    }
    NRF_DISABLE();
}

static void NRFWriteContinul(INT8U Reg, INT8U *pTxBuff, INT16U Len)
{
    INT8U Transmit;
    
    Transmit = Reg;
    
    NRF_ENABLE();
    HAL_SPI_Transmit(&hspi2, &Transmit, 1, 1000);
    HAL_SPI_Transmit(&hspi2, pTxBuff, Len, 1000);
    NRF_DISABLE();
}

static void PowerOff(void)
{
    NRF_MODE_LOW();
    NRF_DelayMS(1);   
    
    NRFWriteReg(NRF_WRITE_REG+CONFIG, 0x0D);
 
    NRF_MODE_HIGH();
    
    NRF_DelayMS(20);
}

static INT8U NRFRxMode(void)
{
    INT8U RxBuff[10];
    INT16U ii;
    
    PowerOff();
    NRF_MODE_LOW();
    NRF_DelayMS(1);
    
    NRFWriteContinul(NRF_WRITE_REG + RX_ADDR_P0, (INT8U *)&RXAddress[0], RX_ADR_WIDTH);
    NRFWriteReg(NRF_WRITE_REG + EN_AA, 0x01); // Enable Auto.Ack:Pipe0
    NRFWriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01); // Enable Pipe0
    NRFWriteReg(NRF_WRITE_REG + RF_CH, 12); // Select RF channel 12
    NRFWriteReg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);
    NRFWriteReg(NRF_WRITE_REG + RF_SETUP, 0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
    NRFWriteReg(NRF_WRITE_REG + CONFIG, 0x3f); //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式    
    NRF_MODE_HIGH();
    NRF_DelayMS(1);
    
    LSMReadContinul(NRF_READ_REG+RX_ADDR_P0, RxBuff, RX_ADR_WIDTH); //读出写入的地址
    
	for(ii=0;ii<RX_ADR_WIDTH;ii++)
    {
        if(RxBuff[ii]!=RXAddress[ii])
        {
            break; 
        }
    }
	if(ii != RX_ADR_WIDTH)
    {
        return 0;
    }//检测24L01错误	
	return 1;		 //检测到24L01
}

static INT8U NRFTxMode(void)
{
    INT8U RxBuff[10];
    INT16U ii;
    
    PowerOff();
    NRF_MODE_LOW();
    NRF_DelayMS(1);
    
  	NRFWriteContinul(NRF_WRITE_REG+TX_ADDR,(INT8U *)TXAddress,TX_ADR_WIDTH);//写TX节点地址 
  	NRFWriteContinul(NRF_WRITE_REG+RX_ADDR_P0,(INT8U*)RXAddress,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK
    NRFWriteContinul(WR_TX_PLOAD, g_NRFCtrlMsg.TxBuff, TX_PLOAD_WIDTH); 
    
  	NRFWriteReg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	NRFWriteReg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	NRFWriteReg(NRF_WRITE_REG+SETUP_RETR,0x0a);//设置自动重发间隔时间:250 + 86us;最大自动重发次数:10次
  	NRFWriteReg(NRF_WRITE_REG+RF_CH,12);       //设置RF通道为12
  	NRFWriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRFWriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
    NRF_MODE_HIGH();
    NRF_DelayMS(1);
    
    LSMReadContinul(NRF_READ_REG+TX_ADDR,RxBuff,TX_ADR_WIDTH); //读出写入的地址
    
	for(ii=0;ii<TX_ADR_WIDTH;ii++)
    {
        if(RxBuff[ii]!=TXAddress[ii])
        {
            break; 
        }
    }
	if(ii != TX_ADR_WIDTH)
    {
        return 0;
    }//检测24L01错误	
	return 1;		 //检测到24L01
}		

void NRFInit(void)
{
    memset(&g_NRFCtrlMsg, 0, sizeof(g_NRFCtrlMsg));
    
    if(NRFRxMode() != 1)
    {
        //
        return;
    }
    
    g_NRFCtrlMsg.RxBuff[8] = 25;    //油门
    g_NRFCtrlMsg.RxBuff[10] = 50;   //pitch
    g_NRFCtrlMsg.RxBuff[12] = 50;   //roll
    g_NRFCtrlMsg.RxBuff[14] = 50;   //yaw
}

void NRFTransmitData(void)
{
    INT8U Status;

    NRFWriteContinul(WR_TX_PLOAD, g_NRFCtrlMsg.TxBuff, TX_PLOAD_WIDTH);
    
    while(NRF_MODE_READ());
    
    Status = NRFReadReg(STATUS);
    NRFWriteReg(NRF_WRITE_REG+STATUS,Status);
    
    if(Status&STA_MARK_MX)
    {
        NRFWriteReg(NRF_WRITE_REG+STATUS,0xff);
    }
    else if(Status&STA_MARK_TX)
    {
        return;
    }
}

void NRFReceiveData(void)
{
    INT8U Status;
    
    Status = NRFReadReg(STATUS);
    NRFWriteReg(NRF_WRITE_REG+STATUS,Status);
    
    if(Status&STA_MARK_RX)
    {
        LSMReadContinul(RD_RX_PLOAD, g_NRFCtrlMsg.RxBuff, RX_PLOAD_WIDTH);
        NRFWriteReg(FLUSH_RX, 0xff);
        return ;
    }
}

/*
*函数名：  GetRemoteCtrlMsg
*功  能：  获取遥控通道数据
*输入参数：数据通道
*返回参数： none
*
*每个数据占两个byte,低位在前，高位在后
*/
INT16U GetRemoteCtrlMsg(INT8U Channel)
{
    INT16U Data=0;
    
    Data = g_NRFCtrlMsg.RxBuff[Channel];
    Data |= (INT16U)(g_NRFCtrlMsg.RxBuff[Channel + 1] << 8);
    
    return Data;
}

