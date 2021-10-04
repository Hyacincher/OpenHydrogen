#ifndef LSM303D_H
#define LSM303D_H

#include "cpu.h"
#include "SensorAlign.h"

#define LSM_ENABLE()    GPIOB->ODR &= ~(1<<3)
#define LSM_DISABLE()   GPIOB->ODR |= (1<<3)
#define LSM_DelayMS(x)   Hal_DelayMs(x)

#define LSM_INSTALL_ROTATE  CW90_FLIP

#define LSM303D_WRITE           0x00
#define LSM303D_READ            0x80
#define LSM303D_CONTINUL        0x40

#define LSM_DEVICE_ID       0x49

#define LSM_REG_TEMP_L          0x05
#define LSM_REG_TEMP_H          0x06
#define LSM_REG_M_STATUS        0x07
#define LSM_REG_M_OUTX_L        0x08
#define LSM_REG_M_OUTX_H        0x09
#define LSM_REG_M_OUTY_L        0x0A
#define LSM_REG_M_OUTY_H        0x0B
#define LSM_REG_M_OUTZ_L        0x0C
#define LSM_REG_M_OUTZ_H        0x0D
#define LSM_REG_WHO_AM_I        0x0F
#define LSM_REG_M_INTCTRL       0x12
#define LSM_REG_M_INTSRC        0x13
#define LSM_REG_M_INTTHS_L      0x14
#define LSM_REG_M_INTTHS_H      0x15
#define LSM_REG_M_OFFX_L        0x16
#define LSM_REG_M_OFFX_H        0x17
#define LSM_REG_M_OFFY_L        0x18
#define LSM_REG_M_OFFY_H        0x19
#define LSM_REG_M_OFFZ_L        0x1A
#define LSM_REG_M_OFFZ_H        0x1B
#define LSM_REG_M_REFE_X        0x1C
#define LSM_REG_M_REFE_Y        0x1D
#define LSM_REG_M_REFE_Z        0x1E

#define LSM_REG_CTRL_0          0x1F
#define LSM_REG_CTRL_1          0x20
#define LSM_REG_CTRL_2          0x21
#define LSM_REG_CTRL_3          0x22
#define LSM_REG_CTRL_4          0x23
#define LSM_REG_CTRL_5          0x24
#define LSM_REG_CTRL_6          0x25
#define LSM_REG_CTRL_7          0x26

#define LSM_REG_A_STATUS        0x27
#define LSM_REG_A_OUTX_L        0x28
#define LSM_REG_A_OUTX_H        0x29
#define LSM_REG_A_OUTY_L        0x2A
#define LSM_REG_A_OUTY_H        0x2B
#define LSM_REG_A_OUTZ_L        0x2C
#define LSM_REG_A_OUTZ_H        0x2D

#define LSM_REG_FIFO_CTRL       0x2E
#define LSM_REG_FIFO_SRC        0x2F
#define LSM_REG_IG_CFG1         0x30
#define LSM_REG_IG_SRC1         0x31
#define LSM_REG_IG_THS1         0x32
#define LSM_REG_IG_DUR1         0x33
#define LSM_REG_IG_CFG2         0x34
#define LSM_REG_IG_SRC2         0x35
#define LSM_REG_IG_THS2         0x36
#define LSM_REG_IG_DUR2         0x37
#define LSM_REG_CLICK_CFG       0x38
#define LSM_REG_CLICK_SRC       0x39
#define LSM_REG_CLICK_THS       0x3A
#define LSM_REG_TIME_LMT        0x3B
#define LSM_REG_TIME_LATENCY    0x3C
#define LSM_REG_TIME_WINDOW     0x3D
#define LSM_REG_ACT_THS         0x3E
#define LSM_REG_ACT_DUR         0x3F


typedef struct 
{
    INT16S RawMag[3];     //读出的芯片原始数据
    INT16S RawAcce[3];
}LSMInfo_t;

void LSM303DInit(void);
void LSM303DUpdate(void);

extern LSMInfo_t g_LSMCtrLMsg;
#endif

