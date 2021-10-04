#include "bmp280.h"

BMPInfo_t  g_BMPCtrlMsg;
static FP32 s_BaroGndPressure = 101325.0;
static FP32 s_BaroGndAltitude = 0;
static INT32U s_BaroCaliTimeout = 0;

/*配置bmp280气压和温度过采样 工作模式*/
#define BMP280_PRESSURE_OSR         (BMP280_OVERSAMP_16X)
#define BMP280_TEMPERATURE_OSR      (BMP280_OVERSAMP_4X)
#define BMP280_MODE                 (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_NORMAL_MODE) //

/*配置bmp280气压IIR滤波器*/
#define BMP280_FILTER               (4 << 2)	// BMP280_FILTER_COEFF_16

#define BMP280_DATA_FRAME_SIZE      (6)

#define CONST_PF 0.1902630958	//(1/5.25588f) Pressure factor
#define FIX_TEMP 28				// Fixed Temperature. ASL is a function of pressure and temperature, but as the temperature changes so much (blow a little towards the flie and watch it drop 5 degrees) it corrupts the ASL estimates.
								// TLDR: Adjusting for temp changes does more harm than good.

static void BMPGetData(void);
static INT32U BMPCompensateT(INT32S adcT);
static INT32U BMPCompensateP(INT32S adcP);
static FP32 PressureToAltitude(FP32 Pressure);
static void BaroCalibration(FP32 Pressure);

static void BMPWriteReg(INT8U Reg, INT8U Data)
{
    INT8U s_SendBuff[2];
    INT8U s_ReceiveBuff[2];

    s_SendBuff[0] = Reg;
    s_SendBuff[1] = Data;
    
    BMP_ENABLE();
    HAL_SPI_TransmitReceive(&hspi2, s_SendBuff, s_ReceiveBuff, 2, 1000);
    BMP_DISABLE();
}

static INT8U BMPReadReg(INT8U Reg)
{
    INT8U Transmit[2]={0};
    INT8U Receive[2]={0};
    
    Transmit[0] = Reg | BMP280_READ;
    
    BMP_ENABLE();
    HAL_SPI_TransmitReceive(&hspi2, Transmit, Receive, 2, 1000);
    BMP_DISABLE();
    
    return Receive[1];
}

static void BMPReadContinul(INT8U Reg, INT8U *pRxBuff, INT16U Len)
{
    INT8U Transmit;
    
    Transmit = Reg | BMP280_READ;
    
    BMP_ENABLE();
    HAL_SPI_Transmit(&hspi2, &Transmit, 1, 1000);
    HAL_SPI_Receive(&hspi2, pRxBuff, Len, 1000);
    BMP_DISABLE();   
}

void BMP280Init(void)
{
    INT8U ID = 0x00;
    
    BMP_ENABLE();       //enable bmp280 spi
    BMP_DelayMS(10);
    BMP_DISABLE();
    
    ID = BMPReadReg(BMP280_CHIP_ID);
	
	if (ID == BMP280_DEFAULT_CHIP_ID)//读取正常
	{
        BMPReadContinul(BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, (INT8U *)&g_BMPCtrlMsg.BmpCali, 24);//读取校准数据
        BMPWriteReg(BMP280_CTRL_MEAS_REG, BMP280_MODE);//设置过采样率和工作模式
        BMPWriteReg(BMP280_CONFIG_REG, BMP280_FILTER);//配置IIR滤波
	}
    else
	{
	}
}

static void BMPGetData(void)
{
    INT8U data[BMP280_DATA_FRAME_SIZE];

    BMPReadContinul(BMP280_PRESSURE_MSB_REG, data, BMP280_DATA_FRAME_SIZE);
    g_BMPCtrlMsg.RawPressure = (INT32S)((((INT32U)(data[0])) << 12) | (((INT32U)(data[1])) << 4) | ((INT32U)data[2] >> 4));
    g_BMPCtrlMsg.RawTemperature = (INT32S)((((INT32U)(data[3])) << 12) | (((INT32U)(data[4])) << 4) | ((INT32U)data[5] >> 4));
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static INT32U BMPCompensateT(INT32S adcT)
{
    INT32S var1, var2, T;

    var1 = ((((adcT >> 3) - ((INT32S)g_BMPCtrlMsg.BmpCali.dig_T1 << 1))) * ((INT32S)g_BMPCtrlMsg.BmpCali.dig_T2)) >> 11;
    var2  = (((((adcT >> 4) - ((INT32S)g_BMPCtrlMsg.BmpCali.dig_T1)) * ((adcT >> 4) - ((INT32S)g_BMPCtrlMsg.BmpCali.dig_T1))) >> 12) * ((INT32S)g_BMPCtrlMsg.BmpCali.dig_T3)) >> 14;
    g_BMPCtrlMsg.BmpCali.t_fine = var1 + var2;
    T = (g_BMPCtrlMsg.BmpCali.t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static INT32U BMPCompensateP(INT32S adcP)
{
    INT64S var1, var2, p;
    var1 = ((INT64S)g_BMPCtrlMsg.BmpCali.t_fine) - 128000;
    var2 = var1 * var1 * (INT64S)g_BMPCtrlMsg.BmpCali.dig_P6;
    var2 = var2 + ((var1*(INT64S)g_BMPCtrlMsg.BmpCali.dig_P5) << 17);
    var2 = var2 + (((INT64S)g_BMPCtrlMsg.BmpCali.dig_P4) << 35);
    var1 = ((var1 * var1 * (INT64S)g_BMPCtrlMsg.BmpCali.dig_P3) >> 8) + ((var1 * (INT64S)g_BMPCtrlMsg.BmpCali.dig_P2) << 12);
    var1 = (((((INT64S)1) << 47) + var1)) * ((INT64S)g_BMPCtrlMsg.BmpCali.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adcP;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((INT64S)g_BMPCtrlMsg.BmpCali.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((INT64S)g_BMPCtrlMsg.BmpCali.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((INT64S)g_BMPCtrlMsg.BmpCali.dig_P7) << 4);
    return (INT32U)p;
}

//单位Pa
static FP32 PressureToAltitude(FP32 Pressure)
{
    FP32 Altitude;
    
    if(Pressure > 0)
    {
        //Converts pressure to altitude above sea level (ASL) in meters
        //Altitude = 44330.77*(1-pow(Pressure / 101325, (1/5.256)));
        
        //温度补偿算法
        Altitude = ((pow((1015.7f / Pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;
    }
    else
    {
        Altitude = 0;
    }
    
    return Altitude;
}

//气压计1个标准大气压校准
static void BaroCalibration(FP32 Pressure)
{
	//慢慢收敛校准
    const FP32 PressureError = Pressure - s_BaroGndPressure;
    s_BaroGndPressure += PressureError * 0.15f;

    if (MyFP32Abs(PressureError) < (s_BaroGndPressure * 0.00005f))  // 0.005% calibration error (should give c. 10cm calibration error)
	{
        if ((g_SysTickTime - s_BaroCaliTimeout) > 250) 
		{
            s_BaroGndAltitude = PressureToAltitude(s_BaroGndPressure);
            g_BMPCtrlMsg.Status.IsStable = 1;
        }
    }
    else 
	{
        s_BaroCaliTimeout = g_SysTickTime;
    }
}

static INT32S BaroMedianFilter(INT32S NewPressure)
{
    static INT32S FilterBuff[MEDIAN_FILTER_LEN];
    static INT32U FilterIndex = 0;
    static BOOLEAN FilterIsReady = 0;
    
    INT32S NextIndex = FilterIndex + 1;
    if (NextIndex == MEDIAN_FILTER_LEN) 
	{
        NextIndex = 0;
        FilterIsReady = 1;
    }
    
    INT32S LastIndex = FilterIndex - 1;
    if (LastIndex < 0) 
	{
        LastIndex = MEDIAN_FILTER_LEN - 1;
    }
    
    const INT32S LastPressure = FilterBuff[LastIndex];

    if (FilterIsReady) 
	{
        if (MyINT32SAbs(LastPressure - NewPressure) < MAX_DELTA_ERROR)
		{
            FilterBuff[FilterIndex] = NewPressure;
            FilterIndex = NextIndex;
            return QuickMedian3_INT32S(FilterBuff);
        } 
		else
		{
            // glitch threshold exceeded, so just return previous reading and don't add the glitched reading to the filter array
            return FilterBuff[LastIndex];
        }
    } 
	else 
	{
        FilterBuff[FilterIndex] = NewPressure;
        FilterIndex = NextIndex;
        return NewPressure;
    }
}

void BMP280Update(void)
{
    FP64 Pressure;
    
	BMPGetData();
	
	g_BMPCtrlMsg.Temperature = BMPCompensateT(g_BMPCtrlMsg.RawTemperature)/100.0f;	/*单位度*/
	Pressure = (FP32)BMPCompensateP(g_BMPCtrlMsg.RawPressure);
    Pressure /= 256.0f;		    /*单位Pa*/
    
    g_BMPCtrlMsg.Pressure = BaroMedianFilter(Pressure * 10) / 10.0f;    //整形计算，保留小数

    if(!g_BMPCtrlMsg.Status.IsStable)
    {
        BaroCalibration(g_BMPCtrlMsg.Pressure);
        g_BMPCtrlMsg.Altitude = 0;
    }
    else
    {
        g_BMPCtrlMsg.Altitude = PressureToAltitude(g_BMPCtrlMsg.Pressure) - s_BaroGndAltitude;
    }
}

