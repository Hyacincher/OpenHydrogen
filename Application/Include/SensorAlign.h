#ifndef SENSORALIGN_H
#define SENSORALIGN_H

#include "cpu.h"

typedef enum
{
    CW0_POSITIVE = 0,       //保持原位
    CW90_POSITIVE,          //顺时针旋转90°，不翻转
    CW180_POSITIVE,         //顺时针旋转180°，不翻转
    CW270_POSITIVE,         //顺时针旋转270°，不翻转
    CW0_FLIP,               //保持原位，上下翻转
    CW90_FLIP,              //顺时针旋转90°，上下翻转
    CW180_FLIP,             //顺时针旋转180°，上下翻转
    CW270_FLIP              //顺时针旋转270°，上下翻转
}AlignDirec_e;


void SensorAlign(INT16S *Sensor, AlignDirec_e Align);

#endif

