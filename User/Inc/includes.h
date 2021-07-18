#ifndef INCLUDES_H
#define INCLUDES_H

#include <math.h>
#include <string.h>

typedef unsigned char  BOOLEAN;                      /*  Boolean 布尔变量           */
typedef unsigned char  INT8U;                        /*  Unsigned  8 bit quantity   */                       
typedef signed   char  INT8S;                        /*  Signed    8 bit quantity   */                         
typedef unsigned short INT16U;                       /*  Unsigned 16 bit quantity   */
typedef signed   short INT16S;                       /*  Signed   16 bit quantity   */
typedef unsigned int   INT32U;                       /*  Unsigned 32 bit quantity   */
typedef signed   int   INT32S;                       /*  Signed   32 bit quantity   */
typedef signed long long INT64S;			     /*  Signed   64 bit quantity   */
typedef unsigned long long INT64U;			     /*  Unsigned 64 bit quantity   */
typedef float          FP32;   			 /*  Single precision floating  point*/
typedef double         FP64; 			 /*  Double precision floating  point*/

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "Mymath.h"
#include "Filter.h"

#define DEBUG_MODE  1


#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#endif 




