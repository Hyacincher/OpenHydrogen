#include "MPU6000.h"

MPUInfo_t g_MPUCtrlMsg;

static void MPUSetRate(INT16U Rate);
static void MPUSetLPF(INT16U Lpf);

/*
*函数名：   MPUWriteReg
*功  能：   MPU6000写寄存器
*输入参数： Reg         寄存器地址
            Data        待写入数据
*返回参数： none
*/
void MPUWriteReg(INT8U Reg, INT8U Data)
{
    INT8U s_SendBuff[2];
    INT8U s_ReceiveBuff[2];

    s_SendBuff[0] = Reg | MPU9250_WRITE;
    s_SendBuff[1] = Data;
    
    MPU_ENABLE();
    HAL_SPI_TransmitReceive(&hspi1, s_SendBuff, s_ReceiveBuff, 2, 1000);
    MPU_DISABLE();
}

/*
*函数名：   MPUReadReg
*功  能：   MPU6000读寄存器
*输入参数： Reg         寄存器地址
*返回参数： 读出数据
*/
INT8U MPUReadReg(INT8U Reg)
{
    INT8U Transmit[2]={0};
    INT8U Receive[2]={0};
    
    Transmit[0] = Reg | MPU9250_READ;
    
    MPU_ENABLE();
    HAL_SPI_TransmitReceive(&hspi1, Transmit, Receive, 2, 1000);
    MPU_DISABLE();
    
    return Receive[1];
}

/*
*函数名：   MPU6000Init
*功  能：   复位MPU6000寄存器
*输入参数： none
*返回参数： none
*/
void MPU6000Init(void)
{
    INT8U ID;

    memset(&g_MPUCtrlMsg, 0, sizeof(g_MPUCtrlMsg));
    
    SPI1->CR1 &= ~(1<<6);  //disable spi
    SPI1->CR1 &= ~(7<<3);   //clear register
    SPI1->CR1 |= (6<<3);    //fclk/128
    SPI1->CR1 |= (1<<6);  //enable spi
    
	MPUWriteReg(MPU_PWR_MGMT1_REG, 0X80);       //复位MPU6050
    MPU_DelayMS(50);
	MPUWriteReg(MPU_SIGPATH_RST_REG, 3|2|1);    //唤醒MPU6050
    MPU_DelayMS(50);
    
	MPUWriteReg(MPU_PWR_MGMT1_REG, 0X80);       //复位MPU6050
    MPU_DelayMS(50);
	MPUWriteReg(MPU_SIGPATH_RST_REG, 3|2|1);	//唤醒MPU6050
    MPU_DelayMS(50);
    
    ID = MPUReadReg(MPU_DEVICE_ID_REG);
    
    if(ID == MPU_DEVICE_ID)
    {
        MPUWriteReg(MPU_PWR_MGMT1_REG, 0x01);   //设置X轴陀螺作为时钟 
        MPU_DelayMS(15);
        
        MPUWriteReg(MPU_USER_CTRL_REG, 0x10);   //禁止I2C接口
        MPU_DelayMS(15);
        
        MPUWriteReg(MPU_PWR_MGMT2_REG, 0x00);   
        MPU_DelayMS(15);
        
        MPUWriteReg(MPU_SAMPLE_RATE_REG, 0x00); //设置采样率
        MPU_DelayMS(15);
        // Accel Sample Rate 1kHz
		// Gyroscope Output Rate =  1kHz when the DLPF is enabled
        
        MPUWriteReg(MPU_GYRO_CFG_REG, 3<<3);    //设置陀螺仪 +/- 2000 DPS量程
        MPU_DelayMS(15);
        
        MPUWriteReg(MPU_ACCEL_CFG_REG, 2<<3);   //设置加速度 +/- 8 G 量程
        MPU_DelayMS(15);
        
        MPUWriteReg(MPU_INTBP_CFG_REG, 0);      //设置中断引脚功能
        MPU_DelayMS(15);
        
        MPUWriteReg(MPU_CFG_REG, 0x02);         //设置低通滤波带宽98hz
        MPU_DelayMS(1);
    }
    else
    {
        //err
    }
    
    SPI1->CR1 &= ~(1<<6);  //disable spi
    SPI1->CR1 &= ~(7<<3);   //clear register
    SPI1->CR1 |= (2<<3);    //fclk/8
    SPI1->CR1 |= (1<<6);  //enable spi
    
    MPU6000Update();
}

static void MPUSetRate(INT16U Rate)
{
	INT8U data;
    
	if(Rate>1000)Rate=1000;
	if(Rate<4)Rate=4;
	data=1000/Rate-1;
	MPUWriteReg(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
    
 	MPUSetLPF(Rate/2);	//自动设置LPF为采样率的一半
}

//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
static void MPUSetLPF(INT16U Lpf)
{
	INT8U data=0;
    
	if(Lpf>=188)data=1;
	else if(Lpf>=98)data=2;
	else if(Lpf>=42)data=3;
	else if(Lpf>=20)data=4;
	else if(Lpf>=10)data=5;
	else data=6; 
	MPUWriteReg(MPU_CFG_REG,data);//设置数字低通滤波器
}

/*
*函数名：   MPUUpdateData
*功  能：   更新MPU6000读数
*输入参数： *MPU6000     处理句柄
*返回参数： none
*/
void MPU6000Update(void)
{
    INT8U s_ReceiveBuff[14]={0};
    INT8U s_Transmit;
    INT8U ii;
    INT16S s_Temperature;
    
    s_Transmit = MPU_ACCEL_XOUTH_REG|MPU9250_READ;
    
    MPU_ENABLE();
    HAL_SPI_Transmit(&hspi1, &s_Transmit, 1, 10);
    HAL_SPI_Receive(&hspi1, s_ReceiveBuff, 14, 10);
    MPU_DISABLE();
    
    for(ii = 0 ; ii < 6 ; ii++)
    {
        if((ii % 2)==0)
        {
            g_MPUCtrlMsg.RawAcce[ii / 2] = s_ReceiveBuff[ii];
            g_MPUCtrlMsg.RawAcce[ii / 2] <<= 8;
        }
        else
        {
            g_MPUCtrlMsg.RawAcce[ii / 2] |= s_ReceiveBuff[ii];
        }
    }
    
    for(ii = 6 ; ii < 8 ; ii++)
    {
        if((ii % 2)==0)
        {
            s_Temperature = s_ReceiveBuff[ii];
            s_Temperature <<= 8;
        }
        else
        {
            s_Temperature |= s_ReceiveBuff[ii];
        }
    }
    
    FP32 Input;
    
    Input = (FP32)((s_Temperature)/333.87) + 21;
    LowPassFilter(0.2, &Input, &g_MPUCtrlMsg.Temperature);
    
    for(ii = 8 ; ii < 14 ; ii++)
    {
        if((ii % 2)==0)
        {
            g_MPUCtrlMsg.RawGyro[(ii - 8)/ 2] = s_ReceiveBuff[ii];
            g_MPUCtrlMsg.RawGyro[(ii - 8) / 2] <<= 8;
        }
        else
        {
            g_MPUCtrlMsg.RawGyro[(ii - 8) / 2] |= s_ReceiveBuff[ii];
        }
    }
    
    INT16S Temp;
    //转换到标准右手坐标系
    Temp = g_MPUCtrlMsg.RawAcce[1];
    g_MPUCtrlMsg.RawAcce[1] = -g_MPUCtrlMsg.RawAcce[0];
    g_MPUCtrlMsg.RawAcce[0] = Temp;
    
    Temp = g_MPUCtrlMsg.RawGyro[1];
    g_MPUCtrlMsg.RawGyro[1] = -g_MPUCtrlMsg.RawGyro[0];
    g_MPUCtrlMsg.RawGyro[0] = Temp;    
}

