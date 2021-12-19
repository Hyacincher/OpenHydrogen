#ifndef COMMUNI_H
#define COMMUNI_H

#include "NRF24L01.h"

typedef struct
{
    struct
    {
    }Status;
    INT8U TaskStage;
    INT32U TaskTime;
    
}CommuniInfo_t;

void CommnuniInit(void);
void CommnuniTask(void);

#endif

