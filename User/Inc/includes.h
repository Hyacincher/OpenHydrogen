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

#define FLOAT_EQUAL_DELTA   0.000001f
/*
2021.08.29--V0.01
1、修复串级PID的bug，角速度环单位不统一，err失效，内环失效，重调PID

2021.08.31--V0.02
1、调整手动模式油门
2、重调PID完成
3、完成无头模式

2021.09.07--V0.03
1、PID重调，本轮调试完成，下一轮主要任务定高

2021.09.21--V0.04
1、IMU坐标系统一对齐，其他地方使用统一的参数，不再进行单独的方向调整
2、高度数据融合debug完成

2021.10.06--V0.05
1、解决气压计偶尔失效的问题，与电路地线干扰有关，暂时不焊接LORA调试，下版本电路做地线隔离
2、调整高度数据融合参数，使曲线更贴合实际变化
3、修正PWM输出频率

2021.10.xx--V0.06
1、修改PID输出方向，对齐新的电机方向
2、微调PID

2021.11.14--V0.07
1、调整三轴PID
2、改变高度融合策略，采用APM三阶融合
3、增加定高功能，当前PID可以进行基本定高，需细化调整
4、现在丢控之后进自动降落模式，而不是改变打杆量来降落

*/
#endif 




