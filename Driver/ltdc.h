#ifndef _LCD_H
#define _LCD_H

#include "includes.h"
#include "stm32f4xx_hal.h"
#include "font.h"
//#include "pic.h"

//LCD LTDC重要参数集
typedef struct  
{							 
	INT32U pwidth;      //LCD面板的宽度,固定参数,不随显示方向改变,如果为0,说明没有任何RGB屏接入
	INT32U pheight;		//LCD面板的高度,固定参数,不随显示方向改变
	INT16U hsw;			//水平同步宽度
	INT16U vsw;			//垂直同步宽度
	INT16U hbp;			//水平后廊
	INT16U vbp;			//垂直后廊
	INT16U hfp;			//水平前廊
	INT16U vfp;			//垂直前廊 
	INT8U activelayer;  //当前层编号:0/1	
	INT8U dir;          //0,竖屏;1,横屏;
	INT16U width;       //LCD宽度
	INT16U height;      //LCD高度
	INT32U pixsize;		//每个像素所占字节数
}LCD_Info_t; 

extern LCD_Info_t lcdltdc;		            //管理LCD LTDC参数
extern LTDC_HandleTypeDef LTDC_Handler;	    //LTDC句柄
extern DMA2D_HandleTypeDef DMA2D_Handler;   //DMA2D句柄

#define LCD_PIXEL_FORMAT_ARGB8888       0X00    
#define LCD_PIXEL_FORMAT_RGB888         0X01    
#define LCD_PIXEL_FORMAT_RGB565         0X02       
#define LCD_PIXEL_FORMAT_ARGB1555       0X03      
#define LCD_PIXEL_FORMAT_ARGB4444       0X04     
#define LCD_PIXEL_FORMAT_L8             0X05     
#define LCD_PIXEL_FORMAT_AL44           0X06     
#define LCD_PIXEL_FORMAT_AL88           0X07      

///////////////////////////////////////////////////////////////////////
//用户修改配置部分:

#define LCD_WIDTH                   480
#define LCD_HIGHT                   800

//定义颜色像素格式,一般用RGB565
#define LCD_PIXFORMAT				LCD_PIXEL_FORMAT_RGB565	
//定义默认背景层颜色
#define LTDC_BACKLAYERCOLOR			0X00000000	
//LCD帧缓冲区首地址,这里定义在SDRAM里面.
#define LCD_FRAME_BUF_ADDR			0XC0000000  

//LCD IO操作
#define LCD_BK_ENABLE()             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define LCD_BK_DISABLE()            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)

#define LCD_SPI_CS(status)          HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, status)
#define LCD_Delay(MS)               HAL_Delay(MS)
//#define LCD_RESET()               接的单片机复位
#define LCD_SPI_SDO(status)         HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, status)
#define LCD_SPI_SCK(status)         HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, status)

void LTDC_Switch(INT8U sw);					//LTDC开关
void LTDC_Layer_Switch(INT8U layerx,INT8U sw);	//层开关
void LTDC_Select_Layer(INT8U layerx);			//层选择
void LTDC_Display_Dir(INT8U dir);				//显示方向控制
void LTDC_Draw_Point(INT16U x,INT16U y,INT32U color);//画点函数
INT32U LTDC_Read_Point(INT16U x,INT16U y);			//读点函数
void LTDC_Fill(INT16U sx,INT16U sy,INT16U ex,INT16U ey,INT32U color);			//矩形单色填充函数
void LTDC_Color_Fill(INT16U sx,INT16U sy,INT16U ex,INT16U ey,INT16U *color);	//矩形彩色填充函数
void LTDC_Clear(INT32U color);					//清屏函数

INT8U LTDC_Clk_Set(INT32U pllsain,INT32U pllsair,INT32U pllsaidivr);//LTDC时钟配置
void LTDC_Layer_Window_Config(INT8U layerx,INT16U sx,INT16U sy,INT16U width,INT16U height);//LTDC层窗口设置
void LTDC_Layer_Parameter_Config(INT8U layerx,INT32U bufaddr,INT8U pixformat,INT8U alpha,INT8U alpha0,INT8U bfac1,INT8U bfac2,INT32U bkcolor);//LTDC基本参数设置
void LTDC_Init(void);						//LTDC初始化函数
#endif 
