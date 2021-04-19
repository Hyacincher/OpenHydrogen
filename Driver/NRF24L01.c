#include "NRF24L01.h"

#define TX_PLOAD_WIDTH 32
#define RX_PLOAD_WIDTH 32
#define TX_ADR_WIDTH   5
#define RX_ADR_WIDTH   5

static const INT8U TXAddress[TX_ADR_WIDTH]={0x82,0x66,0x97,0x13,0x30};
static const INT8U RXAddress[TX_ADR_WIDTH]={0x82,0x66,0x97,0x13,0x30};

NRFInfo g_NRFCtrlMsg;

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
    
    //SwitchSPITiming(SPI2, 0, 0);
    
    PowerOff();
    NRF_MODE_LOW();
    NRF_DelayMS(1);
    
    NRFWriteContinul(NRF_WRITE_REG + RX_ADDR_P0, (INT8U *)&RXAddress[0], RX_ADR_WIDTH);
    NRFWriteReg(NRF_WRITE_REG + EN_AA, 0x01); // Enable Auto.Ack:Pipe0
    NRFWriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01); // Enable Pipe0
    NRFWriteReg(NRF_WRITE_REG + RF_CH, 12); // Select RF channel 12
    NRFWriteReg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);
    NRFWriteReg(NRF_WRITE_REG + RF_SETUP, 0x0f);//����TX�������,0db����,2Mbps,���������濪��   
    NRFWriteReg(NRF_WRITE_REG + CONFIG, 0x3f); //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ    
    NRF_MODE_HIGH();
    NRF_DelayMS(1);
    
    LSMReadContinul(NRF_READ_REG+RX_ADDR_P0, RxBuff, RX_ADR_WIDTH); //����д��ĵ�ַ
    
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
    }//���24L01����	
	return 1;		 //��⵽24L01
}

static INT8U NRFTxMode(void)
{
    INT8U RxBuff[10];
    INT16U ii;
    
    //SwitchSPITiming(SPI2, 0, 0);
    
    PowerOff();
    NRF_MODE_LOW();
    NRF_DelayMS(1);
    
  	NRFWriteContinul(NRF_WRITE_REG+TX_ADDR,(INT8U *)TXAddress,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	NRFWriteContinul(NRF_WRITE_REG+RX_ADDR_P0,(INT8U*)RXAddress,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
    NRFWriteContinul(WR_TX_PLOAD, g_NRFCtrlMsg.TxBuff, TX_PLOAD_WIDTH); 
    
  	NRFWriteReg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  	NRFWriteReg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	NRFWriteReg(NRF_WRITE_REG+SETUP_RETR,0x0a);//�����Զ��ط����ʱ��:250 + 86us;����Զ��ط�����:10��
  	NRFWriteReg(NRF_WRITE_REG+RF_CH,12);       //����RFͨ��Ϊ12
  	NRFWriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	NRFWriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
    NRF_MODE_HIGH();
    NRF_DelayMS(1);
    
    LSMReadContinul(NRF_READ_REG+TX_ADDR,RxBuff,TX_ADR_WIDTH); //����д��ĵ�ַ
    
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
    }//���24L01����	
	return 1;		 //��⵽24L01
}		

void NRFInit(void)
{
    memset(&g_NRFCtrlMsg, 0, sizeof(g_NRFCtrlMsg));
    
    if(NRFRxMode() != 1)
    {
        //
        return;
    }
    
    g_NRFCtrlMsg.RxBuff[8] = 25;    //����
    g_NRFCtrlMsg.RxBuff[10] = 50;   //pitch
    g_NRFCtrlMsg.RxBuff[12] = 50;   //roll
    g_NRFCtrlMsg.RxBuff[14] = 50;   //yaw
}

void NRFTransmitData(NRFInfo *NRF)
{
    INT8U Status;
    
    //SwitchSPITiming(SPI2, 0, 0);
    
    NRFWriteContinul(WR_TX_PLOAD,NRF->TxBuff,TX_PLOAD_WIDTH);
    
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

void NRFReceiveData(NRFInfo *NRF)
{
    INT8U Status;
    
    //SwitchSPITiming(SPI2, 0, 0);
    
    Status = NRFReadReg(STATUS);
    NRFWriteReg(NRF_WRITE_REG+STATUS,Status);
    
    if(Status&STA_MARK_RX)
    {
        LSMReadContinul(RD_RX_PLOAD, NRF->RxBuff, RX_PLOAD_WIDTH);
        NRFWriteReg(FLUSH_RX, 0xff);
        return ;
    }
}

/*
*��������  GetRemoteCtrlMsg
*��  �ܣ�  ��ȡң��ͨ������
*���������*MPU6000��MPU6000���ݽṹ��ָ�룩
*���ز����� none
*
*ÿ������ռ����byte,��λ��ǰ����λ�ں�
*/
INT16U GetRemoteCtrlMsg(INT8U Channel)
{
    INT16U Data=0;
    
    Data = g_NRFCtrlMsg.RxBuff[Channel];
    Data |= (INT16U)(g_NRFCtrlMsg.RxBuff[Channel + 1] << 8);
    
    return Data;
}

