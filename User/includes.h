#ifndef INCLUDES_H
#define INCLUDES_H

#include <math.h>
#include <string.h>

typedef unsigned char       BOOLEAN;        /*  Boolean 布尔变量           */
typedef unsigned char       INT8U;          /*  Unsigned  8 bit quantity   */                       
typedef signed   char       INT8S;          /*  Signed    8 bit quantity   */                         
typedef unsigned short      INT16U;         /*  Unsigned 16 bit quantity   */
typedef signed   short      INT16S;         /*  Signed   16 bit quantity   */
typedef unsigned int        INT32U;         /*  Unsigned 32 bit quantity   */
typedef signed   int        INT32S;         /*  Signed   32 bit quantity   */
typedef signed long long    INT64S;         /*  Signed   64 bit quantity   */
typedef unsigned long long  INT64U;         /*  Unsigned 64 bit quantity   */
typedef float               FP32;           /*  Single precision floating  point*/
typedef double              FP64;           /*  Double precision floating  point*/

#define     TRUE            1
#define     FALSE           0

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define DEBUG_MODE  1

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_125_HZ		125
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define FLOAT_EQUAL_DELTA   0.000001f

/*
2021.12.05--V0.01
1、建立工程

2021.12.19--V0.02
1、增加ADC读取驱动
2、增加LCD驱动，写字符驱动
3、增加NRF驱动

*/
#endif 




