#include "ltdc.h"

LTDC_HandleTypeDef  LTDC_Handler;	    //LTDC句柄
DMA2D_HandleTypeDef DMA2D_Handler; 	    //DMA2D句柄

volatile INT16U g_LCD_Cache[LCD_WIDTH*LCD_HIGHT] __attribute__((section(".ARM.__at_0xC0000000")));
INT32U *g_LCD_FrameBuff[2];       //LTDC LCD帧缓存数组指针,必须指向对应大小的内存区域

LCD_Info_t g_LCDCtrlMsg;        //管理LCD LTDC的重要参数

static void LTDC_Para_Init(void);

/*
内存方向
————————————————————————————
|   0   480     ……          |触摸接口
|   1   ……  ……  ……          |
|   ……  ……  ……  ……          |屏接口
|   479 ……      383999      |
————————————————————————————
*/

//打开LCD开关
//lcd_switch:1 打开,0，关闭
void LTDC_Switch(INT8U sw)
{
	if(sw==1) __HAL_LTDC_ENABLE(&LTDC_Handler);
	else if(sw==0)__HAL_LTDC_DISABLE(&LTDC_Handler);
}

//开关指定层
//layerx:层号,0,第一层; 1,第二层
//sw:1 打开;0关闭
void LTDC_Layer_Switch(INT8U layerx,INT8U sw)
{
	if(sw==1) __HAL_LTDC_LAYER_ENABLE(&LTDC_Handler,layerx);
	else if(sw==0) __HAL_LTDC_LAYER_DISABLE(&LTDC_Handler,layerx);
	__HAL_LTDC_RELOAD_CONFIG(&LTDC_Handler);
}

//选择层
//layerx:层号;0,第一层;1,第二层;
void LTDC_Select_Layer(INT8U layerx)
{
	g_LCDCtrlMsg.activelayer=layerx;
}

//设置LCD显示方向
//dir:0,竖屏；1,横屏
void LTDC_Display_Dir(DisplyDir_e dir)
{
    g_LCDCtrlMsg.dir=dir; 	//显示方向
	if(dir == LCD_DIR_VER)			//竖屏
	{
        g_LCDCtrlMsg.width=g_LCDCtrlMsg.pwidth;
		g_LCDCtrlMsg.height=g_LCDCtrlMsg.pheight;
	}else if(dir == LCD_DIR_HOR)	//横屏
	{
		g_LCDCtrlMsg.width=g_LCDCtrlMsg.pheight;
		g_LCDCtrlMsg.height=g_LCDCtrlMsg.pwidth;		
	}
}

//画点函数
//x,y:写入坐标
//color:颜色值
void LTDC_Draw_Point(INT16U x,INT16U y,INT32U color)
{
	if(g_LCDCtrlMsg.dir)	//横屏
	{
        *(INT16U*)((INT32U)g_LCD_FrameBuff[g_LCDCtrlMsg.activelayer]+g_LCDCtrlMsg.pixsize*(g_LCDCtrlMsg.pwidth*x+y))=color;
	}else 			//竖屏
	{
        *(INT16U*)((INT32U)g_LCD_FrameBuff[g_LCDCtrlMsg.activelayer]+g_LCDCtrlMsg.pixsize*(g_LCDCtrlMsg.pwidth*y+(g_LCDCtrlMsg.pwidth-x-1)))=color; 
	}
}

//读点函数
//x,y:读取点的坐标
//返回值:颜色值
INT32U LTDC_Read_Point(INT16U x,INT16U y)
{
	if(g_LCDCtrlMsg.dir)	//横屏
	{
		return *(INT16U*)((INT32U)g_LCD_FrameBuff[g_LCDCtrlMsg.activelayer]+g_LCDCtrlMsg.pixsize*(g_LCDCtrlMsg.pwidth*x+y));
	}else 			//竖屏
	{
		return *(INT16U*)((INT32U)g_LCD_FrameBuff[g_LCDCtrlMsg.activelayer]+g_LCDCtrlMsg.pixsize*(g_LCDCtrlMsg.pwidth*y+(g_LCDCtrlMsg.pwidth-x-1))); 
	}
}

//LTDC填充矩形,DMA2D填充
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
//color:要填充的颜色
//有时候需要频繁的调用填充函数，所以为了速度，填充函数采用寄存器版本，
//不过下面有对应的库函数版本的代码。
void LTDC_Fill(INT16U sx,INT16U sy,INT16U ex,INT16U ey,INT32U color)
{
	INT32U psx,psy,pex,pey;	//以LCD面板为基准的坐标系,不随横竖屏变化而变化
	INT32U timeout=0; 
	INT16U offline;
	INT32U addr; 
	//坐标系转换
	if(g_LCDCtrlMsg.dir)	//横屏
	{
        psx=sy;pex=ey;
        psy=sx;pey=ex;
	}else			//竖屏
	{
        psx=(g_LCDCtrlMsg.pwidth-ex-1);pex=(g_LCDCtrlMsg.pwidth-sx-1);
        psy=sy;pey=ey;
	}
    
	offline=g_LCDCtrlMsg.pwidth-(pex-psx+1);
	addr=((INT32U)g_LCD_FrameBuff[g_LCDCtrlMsg.activelayer]+g_LCDCtrlMsg.pixsize*(g_LCDCtrlMsg.pwidth*psy+psx));
	__HAL_RCC_DMA2D_CLK_ENABLE();	//使能DM2D时钟
	DMA2D->CR&=~(DMA2D_CR_START);	//先停止DMA2D
	DMA2D->CR=DMA2D_R2M;			//寄存器到存储器模式
	DMA2D->OPFCCR=LCD_PIXFORMAT;	//设置颜色格式
	DMA2D->OOR=offline;				//设置行偏移                   //行末尾跳过的点数

	DMA2D->OMAR=addr;				//输出存储器地址               //起始地址
	DMA2D->NLR=(pey-psy+1)|((pex-psx+1)<<16);	//设定行数寄存器   //高16位是行传输点数，低16位是行数
	DMA2D->OCOLR=color;						//设定输出颜色寄存器 
	DMA2D->CR|=DMA2D_CR_START;				//启动DMA2D
	while((DMA2D->ISR&(DMA2D_FLAG_TC))==0)	//等待传输完成
	{
		timeout++;
		if(timeout>0X1FFFFF)break;	//超时退出
	} 
	DMA2D->IFCR|=DMA2D_FLAG_TC;		//清除传输完成标志 		
}

//在指定区域内填充指定颜色块,DMA2D填充	
//此函数仅支持INT16U,RGB565格式的颜色数组填充.
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)  
//注意:sx,ex,不能大于lcddev.width-1;sy,ey,不能大于lcddev.height-1!!!
//color:要填充的颜色数组首地址
void LTDC_Color_Fill(INT16U sx,INT16U sy,INT16U ex,INT16U ey,INT16U *color)
{
	INT32U psx,psy,pex,pey;	//以LCD面板为基准的坐标系,不随横竖屏变化而变化
	INT32U timeout=0; 
	INT16U offline;
	INT32U addr; 
	//坐标系转换
	if(g_LCDCtrlMsg.dir)	//横屏
	{
        psx=sy;pex=ey;
        psy=sx;pey=ex;
	}else			//竖屏
	{
        psx=(g_LCDCtrlMsg.pwidth-ex-1);pex=(g_LCDCtrlMsg.pwidth-sx-1);
        psy=sy;pey=ey;
	}
    
	offline=g_LCDCtrlMsg.pwidth-(pex-psx+1);
	addr=((INT32U)g_LCD_FrameBuff[g_LCDCtrlMsg.activelayer]+g_LCDCtrlMsg.pixsize*(g_LCDCtrlMsg.pwidth*psy+psx));
	__HAL_RCC_DMA2D_CLK_ENABLE();	//使能DM2D时钟
	DMA2D->CR&=~(DMA2D_CR_START);	//先停止DMA2D
	DMA2D->CR=DMA2D_M2M;			//存储器到存储器模式
	DMA2D->FGPFCCR=LCD_PIXFORMAT;	//设置颜色格式
	DMA2D->FGOR=0;					//前景层行偏移为0
	DMA2D->OOR=offline;				//设置行偏移 

	DMA2D->FGMAR=(INT32U)color;		//源地址
	DMA2D->OMAR=addr;				//输出存储器地址
	DMA2D->NLR=(pey-psy+1)|((pex-psx+1)<<16);	//设定行数寄存器 
	DMA2D->CR|=DMA2D_CR_START;					//启动DMA2D
	while((DMA2D->ISR&(DMA2D_FLAG_TC))==0)		//等待传输完成
	{
		timeout++;
		if(timeout>0X1FFFFF)break;	//超时退出
	} 
	DMA2D->IFCR|=DMA2D_FLAG_TC;				//清除传输完成标志  	
}  

//LCD清屏
//color:颜色值
void LTDC_Clear(INT32U color)
{
	LTDC_Fill(0,0,g_LCDCtrlMsg.width-1,g_LCDCtrlMsg.height-1,color);
}

//LTDC时钟(Fdclk)设置函数
//Fvco=Fin*pllsain; 
//Fdclk=Fvco/pllsair/2*2^pllsaidivr=Fin*pllsain/pllsair/2*2^pllsaidivr;

//Fvco:VCO频率
//Fin:输入时钟频率一般为1Mhz(来自系统时钟PLLM分频后的时钟,见时钟树图)
//pllsain:SAI时钟倍频系数N,取值范围:50~432.  
//pllsair:SAI时钟的分频系数R,取值范围:2~7
//pllsaidivr:LCD时钟分频系数,取值范围:RCC_PLLSAIDIVR_2/4/8/16,对应分频2~16 
//返回值:0,成功;1,失败。
INT8U LTDC_Clk_Set(INT32U pllsain,INT32U pllsair,INT32U pllsaidivr)
{
	RCC_PeriphCLKInitTypeDef PeriphClkIniture;
	
	//LTDC输出像素时钟，需要根据自己所使用的LCD数据手册来配置！
    PeriphClkIniture.PeriphClockSelection=RCC_PERIPHCLK_LTDC;	//LTDC时钟 	
	PeriphClkIniture.PLLSAI.PLLSAIN=pllsain;    
	PeriphClkIniture.PLLSAI.PLLSAIR=pllsair;  
	PeriphClkIniture.PLLSAIDivR=pllsaidivr;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkIniture)==HAL_OK)    //配置像素时钟
    {
        return 0;   //成功
    }
    else return 1;  //失败    
}

//LTDC,层颜窗口设置,窗口以LCD面板坐标系为基准
//注意:此函数必须在LTDC_Layer_Parameter_Config之后再设置.
//layerx:层值,0/1.
//sx,sy:起始坐标
//width,height:宽度和高度
void LTDC_Layer_Window_Config(INT8U layerx,INT16U sx,INT16U sy,INT16U width,INT16U height)
{
    HAL_LTDC_SetWindowPosition(&LTDC_Handler,sx,sy,layerx);  //设置窗口的位置
    HAL_LTDC_SetWindowSize(&LTDC_Handler,width,height,layerx);//设置窗口大小    
}

//LTDC,基本参数设置.
//注意:此函数,必须在LTDC_Layer_Window_Config之前设置.
//layerx:层值,0/1.
//bufaddr:层颜色帧缓存起始地址
//pixformat:颜色格式.0,ARGB8888;1,RGB888;2,RGB565;3,ARGB1555;4,ARGB4444;5,L8;6;AL44;7;AL88
//alpha:层颜色Alpha值,0,全透明;255,不透明
//alpha0:默认颜色Alpha值,0,全透明;255,不透明
//bfac1:混合系数1,4(100),恒定的Alpha;6(101),像素Alpha*恒定Alpha
//bfac2:混合系数2,5(101),恒定的Alpha;7(111),像素Alpha*恒定Alpha
//bkcolor:层默认颜色,32位,低24位有效,RGB888格式
//返回值:无
void LTDC_Layer_Parameter_Config(INT8U layerx,INT32U bufaddr,INT8U pixformat,INT8U alpha,INT8U alpha0,INT8U bfac1,INT8U bfac2,INT32U bkcolor)
{
	LTDC_LayerCfgTypeDef pLayerCfg;
	
	pLayerCfg.WindowX0=0;                       //窗口起始X坐标
	pLayerCfg.WindowY0=0;                       //窗口起始Y坐标
	pLayerCfg.WindowX1=g_LCDCtrlMsg.pwidth;          //窗口终止X坐标
	pLayerCfg.WindowY1=g_LCDCtrlMsg.pheight;         //窗口终止Y坐标
	pLayerCfg.PixelFormat=pixformat;		    //像素格式
	pLayerCfg.Alpha=alpha;				        //Alpha值设置，0~255,255为完全不透明
	pLayerCfg.Alpha0=alpha0;			        //默认Alpha值
	pLayerCfg.BlendingFactor1=(INT32U)bfac1<<8;    //设置层混合系数
	pLayerCfg.BlendingFactor2=(INT32U)bfac2<<8;	//设置层混合系数
	pLayerCfg.FBStartAdress=bufaddr;	        //设置层颜色帧缓存起始地址
	pLayerCfg.ImageWidth=g_LCDCtrlMsg.pwidth;        //设置颜色帧缓冲区的宽度    
	pLayerCfg.ImageHeight=g_LCDCtrlMsg.pheight;      //设置颜色帧缓冲区的高度
	pLayerCfg.Backcolor.Red=(INT8U)(bkcolor&0X00FF0000)>>16;   //背景颜色红色部分
	pLayerCfg.Backcolor.Green=(INT8U)(bkcolor&0X0000FF00)>>8;  //背景颜色绿色部分
	pLayerCfg.Backcolor.Blue=(INT8U)bkcolor&0X000000FF;        //背景颜色蓝色部分
	HAL_LTDC_ConfigLayer(&LTDC_Handler,&pLayerCfg,layerx);   //设置所选中的层
}  

//LCD初始化函数
void LTDC_Init(void)
{   
    LTDC_Clk_Set(160, 2, RCC_PLLSAIDIVR_2);   //设置像素时钟 40Mhz
    
    g_LCDCtrlMsg.pwidth=LCD_WIDTH;      //面板宽度,单位:像素
    g_LCDCtrlMsg.pheight=LCD_HIGHT;     //面板高度,单位:像素
    g_LCDCtrlMsg.hsw=2;                //水平同步宽度
    g_LCDCtrlMsg.vsw=7;                //垂直同步宽度
    g_LCDCtrlMsg.hbp=2;                 //水平后廊
    g_LCDCtrlMsg.vbp=2;				    //垂直后廊
    g_LCDCtrlMsg.hfp=18;                //水平前廊
    g_LCDCtrlMsg.vfp=8;				    //垂直前廊
    g_LCDCtrlMsg.pixsize=2;				//每个像素占2个字节
	g_LCD_FrameBuff[0]=(INT32U*)g_LCD_Cache;

    //LTDC配置
    LTDC_Handler.Instance=LTDC;
    LTDC_Handler.Init.HSPolarity=LTDC_HSPOLARITY_AL;         //水平同步极性
    LTDC_Handler.Init.VSPolarity=LTDC_VSPOLARITY_AL;         //垂直同步极性
    LTDC_Handler.Init.DEPolarity=LTDC_DEPOLARITY_AL;         //数据使能极性
	LTDC_Handler.Init.PCPolarity=LTDC_PCPOLARITY_IPC;   	//像素时钟极性
    LTDC_Handler.Init.HorizontalSync=g_LCDCtrlMsg.hsw-1;          //水平同步宽度
    LTDC_Handler.Init.VerticalSync=g_LCDCtrlMsg.vsw-1;            //垂直同步宽度
    LTDC_Handler.Init.AccumulatedHBP=g_LCDCtrlMsg.hsw+g_LCDCtrlMsg.hbp-1; //水平同步后沿宽度
    LTDC_Handler.Init.AccumulatedVBP=g_LCDCtrlMsg.vsw+g_LCDCtrlMsg.vbp-1; //垂直同步后沿高度
    LTDC_Handler.Init.AccumulatedActiveW=g_LCDCtrlMsg.hsw+g_LCDCtrlMsg.hbp+g_LCDCtrlMsg.pwidth-1;//有效宽度
    LTDC_Handler.Init.AccumulatedActiveH=g_LCDCtrlMsg.vsw+g_LCDCtrlMsg.vbp+g_LCDCtrlMsg.pheight-1;//有效高度
    LTDC_Handler.Init.TotalWidth=g_LCDCtrlMsg.hsw+g_LCDCtrlMsg.hbp+g_LCDCtrlMsg.pwidth+g_LCDCtrlMsg.hfp-1;   //总宽度
    LTDC_Handler.Init.TotalHeigh=g_LCDCtrlMsg.vsw+g_LCDCtrlMsg.vbp+g_LCDCtrlMsg.pheight+g_LCDCtrlMsg.vfp-1;  //总高度
    LTDC_Handler.Init.Backcolor.Red=0;           //屏幕背景层红色部分
    LTDC_Handler.Init.Backcolor.Green=0;         //屏幕背景层绿色部分
    LTDC_Handler.Init.Backcolor.Blue=0;          //屏幕背景色蓝色部分
    HAL_LTDC_Init(&LTDC_Handler);
 	
	//层配置
	LTDC_Layer_Parameter_Config(0,(INT32U)g_LCD_FrameBuff[0],LCD_PIXFORMAT,255,0,6,7,0X000000);//层参数配置
	LTDC_Layer_Window_Config(0,0,0,g_LCDCtrlMsg.pwidth,g_LCDCtrlMsg.pheight);	//层窗口配置,以LCD面板坐标系为基准,不要随便修改!
	
 	LTDC_Display_Dir(1);			//默认横屏
	LTDC_Select_Layer(0); 			//选择第1层
    LCD_BK_ENABLE();               //点亮背光
    LTDC_Clear(0xffffffff);			//清屏
    LTDC_Para_Init();
}

//LTDC底层IO初始化和时钟使能
//此函数会被HAL_LTDC_Init()调用
//hltdc:LTDC句柄
void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_LTDC_CLK_ENABLE();                //使能LTDC时钟
    __HAL_RCC_DMA2D_CLK_ENABLE();               //使能DMA2D时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();               //使能GPIOB时钟
    __HAL_RCC_GPIOF_CLK_ENABLE();               //使能GPIOF时钟
    __HAL_RCC_GPIOG_CLK_ENABLE();               //使能GPIOG时钟
    __HAL_RCC_GPIOH_CLK_ENABLE();               //使能GPIOH时钟
    __HAL_RCC_GPIOI_CLK_ENABLE();               //使能GPIOI时钟
    
    //初始化PB5，背光引脚
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);   //暂时关闭背光
    GPIO_Initure.Pin=GPIO_PIN_5;                //PB5推挽输出，控制背光
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL;              //上拉        
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    //LCD参数控制脚
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_SET);     //SCK
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);     //MOSI
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);     //CS
    GPIO_Initure.Pin=GPIO_PIN_8 | GPIO_PIN_3;                //PB5推挽输出，控制背光
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL;              //上拉        
    GPIO_Initure.Speed=GPIO_SPEED_LOW;         //低速
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);
    
    GPIO_Initure.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
    
    //初始化PF10
    GPIO_Initure.Pin=GPIO_PIN_10; 
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //复用
    GPIO_Initure.Pull=GPIO_NOPULL;
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速
    GPIO_Initure.Alternate=GPIO_AF14_LTDC;      //复用为LTDC
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);
    
    //初始化PG6,7,11
    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11;
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);
    
    //初始化PH9,10,11,12,13,14,15
    GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|\
                     GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
    
    //初始化PI0,1,2,4,5,6,7,9,10
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|\
                     GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10;
    HAL_GPIO_Init(GPIOI,&GPIO_Initure); 
}

static void LCD_WriteByteSPI(unsigned char byte)
{
    unsigned char n;
   
    for(n=0; n<8; n++)			
    {  
        if(byte&0x80)
        {
            LCD_SPI_SDO(1);
        }
        else
        {
            LCD_SPI_SDO(0);
        }
        byte<<= 1;

        LCD_SPI_SCK(0);
        LCD_SPI_SCK(1);
    }
}

static void SPI_WriteComm(INT16U CMD)
{
	LCD_SPI_CS(0);
	LCD_WriteByteSPI(0x70);
	LCD_WriteByteSPI(CMD);
	LCD_SPI_CS(1);
}

static void SPI_WriteData(INT16U tem_data)
{			
	LCD_SPI_CS(0);
	LCD_WriteByteSPI(0x72);
	LCD_WriteByteSPI(tem_data);
	LCD_SPI_CS(1);
}

static void LTDC_Para_Init(void)
{
    LCD_SPI_CS(1);
    LCD_Delay(20);
    LCD_SPI_CS(0);
    
    SPI_WriteComm(0x20);//exit_invert_mode
	SPI_WriteComm(0x29);//set_display_on
	SPI_WriteComm(0x3A);//set_pixel_format 
	SPI_WriteData(0x77);//70   0X60 26k

	SPI_WriteComm(0xB1);//RGB Interface Setting
	SPI_WriteData(0x00);
	SPI_WriteData(0x14);
	SPI_WriteData(0x06);

	SPI_WriteComm(0xB2);//Panel Characteristics Setting
	SPI_WriteData(0x10);//480 pixels
	SPI_WriteData(0xC8);//800 pixels

	SPI_WriteComm(0xB3);//Panel Drive Setting    Set the inversion mode


	SPI_WriteData(0x00);//1-dot inversion 0x01

	SPI_WriteComm(0xB4);//Display Mode Control
	SPI_WriteData(0x04);//Dither disable.

	SPI_WriteComm(0xB5);//Display Mode and Frame Memory Write Mode Setting
	SPI_WriteData(0x10);
	SPI_WriteData(0x30);
	SPI_WriteData(0x30);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xB6);//Display Control 2 ( GIP Specific )
	SPI_WriteData(0x01);
	SPI_WriteData(0x18);
	SPI_WriteData(0x02);
	SPI_WriteData(0x40);
	SPI_WriteData(0x10);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xc0);
	SPI_WriteData(0x01);
	SPI_WriteData(0x18);


	SPI_WriteComm(0xC3); 
	SPI_WriteData(0x03);
	SPI_WriteData(0x04);
	SPI_WriteData(0x03);
	SPI_WriteData(0x03);
	SPI_WriteData(0x03);

	LCD_Delay(40);

	SPI_WriteComm(0xC4);//VDD Regulator setting
	SPI_WriteData(0x02);
	SPI_WriteData(0x23);//GDC AP
	SPI_WriteData(0x11);//VRH1  Vreg1out=1.533xVCI(10)
	SPI_WriteData(0x12);//VRH2  Vreg2out=-1.533xVCI(10)
	SPI_WriteData(0x02);//BT 5 VGH/VGL  6/-4
	SPI_WriteData(0x77);//DDVDH 6C//0x56
	LCD_Delay(20);

	SPI_WriteComm(0xC5);
	SPI_WriteData(0x73);
	LCD_Delay(10);

	SPI_WriteComm(0xC6);
	SPI_WriteData(0x24);//VCI 23
	SPI_WriteData(0x60);//RESET RCO 53
	SPI_WriteData(0x00);//SBC GBC
	LCD_Delay(10);
	//GAMMA SETTING
	SPI_WriteComm(0xD0);
	SPI_WriteData(0x14);
	SPI_WriteData(0x01);
	SPI_WriteData(0x53);
	SPI_WriteData(0x25);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x66);
	SPI_WriteData(0x14);
	SPI_WriteData(0x03);

	SPI_WriteComm(0xD1);
	SPI_WriteData(0x14);
	SPI_WriteData(0x01);
	SPI_WriteData(0x53);
	SPI_WriteData(0x07);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x66);
	SPI_WriteData(0x14);
	SPI_WriteData(0x03);



	SPI_WriteComm(0xD2);
	SPI_WriteData(0x14);
	SPI_WriteData(0x01);
	SPI_WriteData(0x53);
	SPI_WriteData(0x25);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x66);
	SPI_WriteData(0x14);
	SPI_WriteData(0x03);

	SPI_WriteComm(0xD3);
	SPI_WriteData(0x14);
	SPI_WriteData(0x01);
	SPI_WriteData(0x53);
	SPI_WriteData(0x07);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x66);
	SPI_WriteData(0x14);
	SPI_WriteData(0x03);


	SPI_WriteComm(0xD4);
	SPI_WriteData(0x14);
	SPI_WriteData(0x01);
	SPI_WriteData(0x53);
	SPI_WriteData(0x25);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x66);
	SPI_WriteData(0x14);
	SPI_WriteData(0x03);

	SPI_WriteComm(0xD5);
	SPI_WriteData(0x14);
	SPI_WriteData(0x01);
	SPI_WriteData(0x53);
	SPI_WriteData(0x07);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x66);
	SPI_WriteData(0x14);
	SPI_WriteData(0x03);

	//DISPLAY ON
	SPI_WriteComm(0x11);
	LCD_Delay(10);
	
	SPI_WriteComm(0x3A); SPI_WriteData(0x77);//set_pixel_format
	SPI_WriteComm(0x36); SPI_WriteData(0x0A);    
}


void LCD_Fill_Pic(INT16U Sx, INT16U Sy,INT16U PicWidth, INT16U PicHight, INT16U* Pic)
{
	LTDC_Color_Fill(Sx, Sy, Sx+PicWidth, Sy+PicHight, Pic);
}

void LCD_ShowChar0816(INT16U Sx, INT16U Sy, INT8U ASCII, INT16U FontColor, INT16U BackColor, BOOLEAN OpenFontBk)
{
    INT8U Temp;
    INT16U Position,ii,jj;
    
    ASCII -= ' ';
    ii = ASCII * 16;
    
	for(Position=0;Position<16;Position++)
    {
        Temp=FontAscii0816[ii+Position];	
        for(jj=0;jj<8;jj++)
       {
            if(Temp&0x80)
            {
                LTDC_Draw_Point(Sx+jj,Sy,FontColor);
            }
            else
            {
                if(OpenFontBk)
                {
                    LTDC_Draw_Point(Sx+jj,Sy,BackColor);
                }
            }	
            Temp<<=1;             
        }
         Sy++;
    }
}

void LCD_ShowChar1632(INT16U Sx, INT16U Sy, INT8U ASCII, INT16U FontColor, INT16U BackColor, BOOLEAN OpenFontBk)
{
    INT8U Temp;
    INT16U Position,ii,jj;
    
    ASCII -= ' ';
    ii = ASCII * 64;
    
	for(Position=0;Position<32;Position++)
    {
        Temp=FontAscii1632[ii+(Position*2)];	
        for(jj=0;jj<8;jj++)
        {
            if(Temp&0x80)
            {
                LTDC_Draw_Point(Sx+jj,Sy,FontColor);
            }
            else
            {
                if(OpenFontBk)
                {
                    LTDC_Draw_Point(Sx+jj,Sy,BackColor);
                }
            }	
            Temp<<=1;             
        }
        Temp=FontAscii1632[ii+(Position*2)+1];	
        for(jj=8;jj<16;jj++)
        {
            if(Temp&0x80)
            {
                LTDC_Draw_Point(Sx+jj,Sy,FontColor);
            }
            else
            {
                if(OpenFontBk)
                {
                    LTDC_Draw_Point(Sx+jj,Sy,BackColor);
                }
            }	
            Temp<<=1;             
        }
        Sy++;
    }
}

//Font_08_16
//Font_16_32
void LCD_ShowString(INT16U Line, INT8U *String, FontSize_e Font, INT16U FontColor, INT16U BackColor)
{
    INT16U Lenth = strlen((const char *)String);
    INT16U X,Y;
    
    if(Font == Font_08_16)
    {
        X = 0;
        Y = Line * 16;
        for(INT16U ii = 0 ; ii < Lenth ; ii++)
        {
            LCD_ShowChar0816(X+(ii*8), Y, String[ii], FontColor, BackColor, TRUE);
        }
    }
    else if(Font == Font_16_32)
    {
        X = 0;
        Y = Line * 32;
        for(INT16U ii = 0 ; ii < Lenth ; ii++)
        {
            LCD_ShowChar1632(X+(ii*16), Y, String[ii], FontColor, BackColor, TRUE);
        }
    }
}

