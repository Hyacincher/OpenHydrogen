#include "Mymath.h"

FP32 MyFP32Min(FP32 x, FP32 y) {
	return x < y ? x : y;
}

FP32 MyFP32Max(FP32 x, FP32 y) {
	return x > y ? x : y;
}

FP32 MyFP32Abs(FP32 x)
{
    return x > 0 ? x : (-x);
}

INT16U MyINT16UAbs(INT16U x, INT16U y)
{
    return x > y ? (x - y) : (y - x);
}

FP64 Myatan2(FP64 Y, FP64 X)
{
    FP64 Result;
    
    if(X > 0)
    {
        Result = atan(Y/X);
    }
    else if(X == 0)
    {
        if(Y > 0)
        {
            Result = PI / 2;
        }
        else if(Y < 0)
        {
            Result = -(PI / 2);
        }
        else
        {
            Result = 0;
        }
    }
    else
    {
        if(Y >= 0)
        {
            Result = atan(Y/X) + PI;
        }
        else
        {
            Result = atan(Y/X) - PI;
        }
    }
    
    return Result;
}

/*
*函数名：convolution
*功  能：求两个输入卷积
*输入参数：
    FP64 *Input1        输入1数组
    FP64 *Input2        输入2数组
    INT16U Number1      输入1点数
    INT16U Number2      输入2点数
    FP64* Output        输出数组（长度为NUM1+NUM2-1）
*返回参数：None
*/
void Convolution(FP32* input1, FP32* input2, FP32* output, INT16U mm, INT16U nn)
{
	//存储地址
	FP32* xx = (FP32*)malloc(sizeof(FP32) * (mm + nn - 1));
	//开始卷积
	for (INT16U i = 0; i < mm + nn - 1; i++)
	{
		xx[i] = 0;
		//以位数最少的卷积作为卷积次数
		for (INT16U j = 0; j < MyFP32Min(mm,nn) ; j++)
        {
            //第一个卷积比第二个卷数积少执行
            if (mm <= nn)
            {
				if (i - j >= 0 && i - j < MyFP32Max(mm, nn))
				{
					xx[i] += input1[j] * input2[i - j];
				}
			}
			//第一个卷积比第二个卷积数多执行
			else
			{
				if (i - j >= 0 && i - j < MyFP32Max(mm, nn))
				{
					xx[i] += input2[j] * input1[i - j];
				}
			}
		}
	}
	for (INT16U i = 0; i < mm+nn-1; i++)
    {
		output[i] = xx[i];
	}
	free(xx);
}


/*
取得某一位的数字，十进制
*/
INT16U GetDigit(INT32U Number, INT8U Digit)
{
    INT16U Result;
    
    Result = ((INT32U)(Number / Digit)) % 10;
    
    return Result;
}

/*
设置某一位的数字，十进制
*/
void SetDigit(INT32U *Number, INT8U Digit, INT8U Value)
{
    INT16U Result;
    
    Result = ((INT32U)(*Number / Digit)) % 10;
    
    if(Number == NULL)
    {
        return;
    }
    *Number -= (Result * Digit);
    
    *Number += (Value * Digit);
}

void BubbleSortINT16(INT16U *p , INT16U Length)
{
    INT16U ii = 0 , jj = 0;
    INT16U Temp = 0;

    for(ii = 0 ; ii < Length - 1 ; ii++)
    {
        for (jj = 0 ; jj < Length - 1 - ii ; jj++)
        {
            if (p[jj] > p [jj + 1])
            {
                Temp = p[jj];
                p[jj] = p[jj + 1];
                p[jj + 1] = Temp;
            }
        }
    }
}

void BubbleSortFP32(FP32 *p , INT16U Length)
{
    INT16U ii = 0 , jj = 0;
    FP32 Temp = 0;

    for(ii = 0 ; ii < Length - 1 ; ii++)
    {
        for (jj = 0 ; jj < Length - 1 - ii ; jj++)
        {
            if (p[jj] > p[jj + 1])
            {
                Temp = p[jj];
                p[jj] = p[jj + 1];
                p[jj + 1] = Temp;
            }
        }
    }    
}


INT32U MyPow(INT32U X, INT32U Y)
{
    INT32U Result;
    INT16U ii;
    
    Result = X;
    if(Y == 0)
    {
        return 1;
    }
    
    for(ii = 0 ; ii < Y - 1 ; ii++)
    {
        Result *= X;
    }
    
    return Result;
}

/**
* @brief 四舍五入保留小数
* @param Keep   要保留的位数
* @param Data   输入数据
* @return 保留后的小数
*/
FP32 KeepDecimals(INT8U Keep, FP32 Data)
{
    INT32U Shift;
    FP32 Result;
	INT8U Neg = 0;
	
	if (Data < 0)
	{
		Neg = 1;
	}
    
    Shift = MyPow(10,(Keep + 1));
    Data += ((1 / Shift) * 5);
    
    Data *= (Shift/10);
    
    Result = (INT32U)Data;
    
	if (Neg)
	{
		return -(Result/(Shift/10));
	}
	else
	{
		return Result/(Shift/10);
	}
}

FP32 MySinApprox(FP32 x)
{
    INT32S xint = x;
    if (xint < -32 || xint > 32) return 0.0f;                               // Stop here on error input (5 * 360 Deg)
    while (x >  PI) x -= (2.0f * PI);                                 // always wrap input angle to -PI..PI
    while (x < -PI) x += (2.0f * PI);
    if (x >  (0.5f * PI)) x =  (0.5f * PI) - (x - (0.5f * PI));   // We just pick -90..+90 Degree
    else if (x < -(0.5f * PI)) x = -(0.5f * PI) - ((0.5f * PI) + x);
    FP32 x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));    
}

FP32 MyCosApprox(FP32 x)
{
    return MySinApprox(x + (0.5f * PI));
}

FP32 MyAtan2Approx(FP32 y, FP32 x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f

    FP32 res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MyFP32Max(absX, absY);
    if (res) res = MyFP32Min(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (PI / 2.0f) - res;
    if (x < 0) res = PI - res;
    if (y < 0) res = -res;
    return res;
}

// http://http.developer.nvidia.com/Cg/acos.html
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.
// Absolute error <= 6.7e-5
FP32 MyAcosApprox(FP32 x)
{
    FP32 xa = fabsf(x);
    FP32 result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return PI - result;
    else
        return result;
}


//值限幅
FP32 MyConstrainF(FP32 Value, FP32 Min, FP32 Max)
{
    if(Value < Min)
    {
        return Min;
    }
    else if(Value > Max)
    {
        return Max;
    }
    
    return Value;
}

INT16U INT8UToINT16U(INT8U Byte[2])
{
    INT16U Data;
    
    Data = (INT16U)Byte[0] << 8;
    Data |= (INT16U)(Byte[1] & 0x00ff);
    
    return Data;
}