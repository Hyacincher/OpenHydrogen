#ifndef __BMP280_H
#define __BMP280_H

#include "cpu.h"
#include <math.h>

/*portable*/
#define BMP_ENABLE()    GPIOC->ODR &= ~(1<<8)
#define BMP_DISABLE()   GPIOC->ODR |= (1<<8)
#define BMP_DelayMS(x)   Hal_DelayMs(x)
/*--------*/

#define BMP280_WRITE        0x00
#define BMP280_READ         0x80

#define BMP280_DEFAULT_CHIP_ID			(0x58)

#define BMP280_CHIP_ID					(0xD0)  /* Chip ID Register */
#define BMP280_RST_REG					(0x60)  /* Softreset Register */
#define BMP280_STAT_REG					(0x73)  /* Status Register */
#define BMP280_CTRL_MEAS_REG			(0x74)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG				(0x75)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG			(0x77)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG			(0x78)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG		(0x79)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG		(0x7A)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG		(0x7B)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG		(0x7C)  /* Temperature XLSB Reg */

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x08)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)

#define BMP280_OVERSAMP_SKIPPED         (0x00)
#define BMP280_OVERSAMP_1X              (0x01)
#define BMP280_OVERSAMP_2X              (0x02)
#define BMP280_OVERSAMP_4X              (0x03)
#define BMP280_OVERSAMP_8X              (0x04)
#define BMP280_OVERSAMP_16X             (0x05)

#define BMP280_FILTER_COEFF_OFF         (0x00)
#define BMP280_FILTER_COEFF_2           (0x01)
#define BMP280_FILTER_COEFF_4           (0x02)
#define BMP280_FILTER_COEFF_8           (0x03)
#define BMP280_FILTER_COEFF_16          (0x04)

#define BMP280_FORCED_MODE             	(0x01)
#define BMP280_NORMAL_MODE				(0x03)


#define MEDIAN_FILTER_LEN       3
#define MAX_DELTA_ERROR         1000

typedef struct
{
    INT16U dig_T1;	/* calibration T1 data */
    INT16S dig_T2; /* calibration T2 data */
    INT16S dig_T3; /* calibration T3 data */
    INT16U dig_P1;	/* calibration P1 data */
    INT16S dig_P2; /* calibration P2 data */
    INT16S dig_P3; /* calibration P3 data */
    INT16S dig_P4; /* calibration P4 data */
    INT16S dig_P5; /* calibration P5 data */
    INT16S dig_P6; /* calibration P6 data */
    INT16S dig_P7; /* calibration P7 data */
    INT16S dig_P8; /* calibration P8 data */
    INT16S dig_P9; /* calibration P9 data */
    INT32S t_fine; /* calibration t_fine data */    
}BMPCaliInfo_t;
    
typedef struct 
{
    BMPCaliInfo_t BmpCali;
    
    struct
    {
        INT8U IsStable : 1;
        INT8U Reserve : 7;
    }Status;
    
    INT32S RawPressure;     //读出的芯片原始数据
    INT32S RawTemperature;
    
    FP32 Pressure;
    FP32 Temperature;
    FP32 Altitude;
}BMPInfo_t;

void BMP280Init(void);
void BMP280Update(void);

extern BMPInfo_t  g_BMPCtrlMsg;
#endif


