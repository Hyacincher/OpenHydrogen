#include "LSM303D.h"

LSMInfo_t   g_LSMCtrLMsg;

static void LSMWriteReg(INT8U Reg, INT8U Data)
{
    INT8U s_SendBuff[2];
    INT8U s_ReceiveBuff[2];

    s_SendBuff[0] = Reg;
    s_SendBuff[1] = Data;
    
    LSM_ENABLE();
    HAL_SPI_TransmitReceive(&hspi3, s_SendBuff, s_ReceiveBuff, 2, 1000);
    LSM_DISABLE();
}

static INT8U LSMReadReg(INT8U Reg)
{
    INT8U Transmit[2]={0};
    INT8U Receive[2]={0};
    
    Transmit[0] = Reg | LSM303D_READ;
    
    LSM_ENABLE();
    HAL_SPI_TransmitReceive(&hspi3, Transmit, Receive, 2, 1000);
    LSM_DISABLE();
    
    return Receive[1];
}

static void LSMReadContinul(INT8U Reg, INT8U *pRxBuff, INT16U Len)
{
    INT8U Transmit;
    
    Transmit = Reg | LSM303D_CONTINUL | LSM303D_READ;
    
    LSM_ENABLE();
    HAL_SPI_Transmit(&hspi3, &Transmit, 1, 1000);
    HAL_SPI_Receive(&hspi3, pRxBuff, Len, 1000);
    LSM_DISABLE();   
}

void LSM303DInit(void)
{
    INT8U ID = LSMReadReg(LSM_REG_WHO_AM_I);
    
    if(ID == LSM_DEVICE_ID)
    {
        LSMWriteReg(LSM_REG_CTRL_0, 0x80);  //reboot mem
        LSM_DelayMS(50);
        LSMWriteReg(LSM_REG_CTRL_0, 0x00);  //start
        LSM_DelayMS(1);
        
        LSMWriteReg(LSM_REG_CTRL_1, 0xA7);  //25hz    //1010 0111
        LSM_DelayMS(1);
        //INT8U Read = LSMReadReg(LSM_REG_CTRL_1);
        
        LSMWriteReg(LSM_REG_CTRL_2, 0x18);  //acc filter banwidth 773hz //+-8g //0001 1000
        LSM_DelayMS(1);
        
        LSMWriteReg(LSM_REG_CTRL_4, 0x04);  //mag int2
        LSM_DelayMS(1);
        
        LSMWriteReg(LSM_REG_CTRL_5, 0xF4);  //mag 100hz / enable temp   //1111 0100
        LSM_DelayMS(1);
        
        LSMWriteReg(LSM_REG_CTRL_6, 0x00);  //+-2 gauss
        LSM_DelayMS(1);
        
        LSMWriteReg(LSM_REG_CTRL_7, 0x00);  //mag continul mode / not reset acc reg//1000 0000
        LSM_DelayMS(1);     
    }
}

void LSM303DUpdate(void)
{
    INT8U MagBuff[6]={0};
    INT8U AccBuff[6]={0};
    
    LSMReadContinul(LSM_REG_M_OUTX_L, MagBuff, 6);    
    LSMReadContinul(LSM_REG_A_OUTX_L, AccBuff, 6);
    
    g_LSMCtrLMsg.RawMag[0] = (INT16S)MagBuff[0] | ((INT16S)MagBuff[1] << 8);
    g_LSMCtrLMsg.RawMag[1] = (INT16S)MagBuff[2] | ((INT16S)MagBuff[3] << 8);
    g_LSMCtrLMsg.RawMag[2] = (INT16S)MagBuff[4] | ((INT16S)MagBuff[5] << 8);

    g_LSMCtrLMsg.RawAcce[0] = (INT16S)AccBuff[0] | ((INT16S)AccBuff[1] << 8);
    g_LSMCtrLMsg.RawAcce[1] = (INT16S)AccBuff[2] | ((INT16S)AccBuff[3] << 8);
    g_LSMCtrLMsg.RawAcce[2] = (INT16S)AccBuff[4] | ((INT16S)AccBuff[5] << 8);
    
    //切换到机体坐标系
    SensorAlign(g_LSMCtrLMsg.RawMag, LSM_INSTALL_ROTATE);
}