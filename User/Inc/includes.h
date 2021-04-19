#ifndef INCLUDES_H
#define INCLUDES_H

#include <math.h>
#include <string.h>

typedef unsigned char  BOOLEAN;                      /*  Boolean ��������           */
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


#define DEBUG_MODE  1
//#define ANO_PRINT 1
#define VISUAL_SCOPE_PRINT  1

#define ATTITUDE_UPDATE_RATE    1000


#endif 




