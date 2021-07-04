#include "LORA433.h"

static BOOLEAN LORASendAT(INT8U *AT, INT8U *Ack);


void LORA433Init(void)
{
    INT8U State = 0;
    
    LORA_AUX_LOW();
    LORA_MD0_HIGH();
    
    LORA_DelayMS(5);
    
    if(LORASendAT((INT8U *)"AT\r\n", (INT8U *)"OK"))
    {
        LORASendAT((INT8U *)"AT+RESET\r\n", (INT8U *)"OK");
        LORA_DelayMS(15);
        
        State = LORASendAT((INT8U *)"AT+FLASH=0\r\n", (INT8U *)"OK");   //不保存
        State = LORASendAT((INT8U *)"AT+ADDR=00,00\r\n", (INT8U *)"OK");//地址0
        State = LORASendAT((INT8U *)"AT+TPOWER=3\r\n", (INT8U *)"OK");  //最大功率
        State = LORASendAT((INT8U *)"AT+CWMODE=0\r\n", (INT8U *)"OK");  //一般模式
        State = LORASendAT((INT8U *)"AT+TMODE=0\r\n", (INT8U *)"OK");   //透明传输
        State = LORASendAT((INT8U *)"AT+WLRATE=23,5\r\n", (INT8U *)"OK");//空速19.2K，433mhz
        State = LORASendAT((INT8U *)"AT+UART=7,0\r\n", (INT8U *)"OK");  //串口115200无校验
        
        LORA_MD0_LOW();
        LORA_DelayMS(1);
    }
}

void LORASendData(INT8U *Data, INT16U Len)
{
    HAL_UART_Transmit(&huart4, Data, Len, 1);
}

static BOOLEAN LORASendAT(INT8U *AT, INT8U *Ack)
{
    INT8U *Tail;
    INT16U Len;
    INT8U RxBuff[30] = {0};
    
    Tail = (INT8U *)strchr((const char *)AT, '\n');
    if(Tail == NULL)
    {
        return 0;
    }
    else
    {
        Len = Tail - AT + 1;
    }
    
    HAL_UART_Transmit(&huart4, AT, Len, 100);
    HAL_UART_Receive(&huart4, RxBuff, 30, 5);
    
    Tail = (INT8U *)strstr((const char *)RxBuff, (const char *)Ack);
    
    if(Tail == NULL)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}