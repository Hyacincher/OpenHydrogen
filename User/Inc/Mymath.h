#ifndef  MYMATH_H
#define  MYMATH_H

#include "includes.h"

#define sq(x) ((x)*(x))

#define PI2 6.283185307                     //2Pi
#define PI 3.1415926535

#if defined(VERY_FAST_MATH)
#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0
#else
#define sinPolyCoef3 -1.666665710e-1f                                          // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5  8.333017292e-3f                                          // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f                                          // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9  2.600054768e-6f                                          // Double:  2.600054767890361277123254766503271638682e-6
#endif

void Convolution(FP32* input1, FP32* input2, FP32* output, INT16U mm, INT16U nn);
FP32 MyFP32Abs(FP32 x);
FP32 MyFP32Max(FP32 x, FP32 y);
FP32 MyFP32Min(FP32 x, FP32 y);
FP32 MyConstrainF(FP32 Value, FP32 Min, FP32 Max);
INT32S MyConstrainINT32S(INT32S Value, INT32S Min, INT32S Max);
INT16U MyConstrainINT16U(INT16U Value, INT16U Min, INT16U Max);
INT16U MyINT16UAbs(INT16U x, INT16U y);

INT16U GetDigit(INT32U Number, INT8U Digit);
void SetDigit(INT32U *Number, INT8U Digit, INT8U Value);

void BubbleSortINT16(INT16U *p , INT16U Length);
void BubbleSortFP32(FP32 *p , INT16U Length);

INT32U MyPow(INT32U X, INT32U Y);
FP32 KeepDecimals(INT8U Keep, FP32 Data);

FP64 Myatan2(FP64 Y, FP64 X);
FP32 MySinApprox(FP32 x);
FP32 MyCosApprox(FP32 x);
FP32 MyAtan2Approx(FP32 y, FP32 x);
FP32 MyAcosApprox(FP32 x);


INT16U INT8UToINT16U(INT8U Byte[2]);
#endif
