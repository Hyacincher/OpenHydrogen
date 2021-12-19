#include "Communi.h"

volatile CommuniInfo_t  g_CommuniCtrlMsg;

void CommnuniInit(void)
{
    memset((void *)&g_CommuniCtrlMsg, 0, sizeof(g_CommuniCtrlMsg));
    
    NRFInit();
}

void CommnuniTask(void)
{
    switch(g_CommuniCtrlMsg.TaskStage)
    {
        case 0:
            g_CommuniCtrlMsg.TaskTime = g_SystemTime;
            g_CommuniCtrlMsg.TaskStage++;
            break;
        case 1:
            if((g_SystemTime - g_CommuniCtrlMsg.TaskTime) > 0)
            {
                NRFTransmitData();
                g_CommuniCtrlMsg.TaskStage++;
            }
            break;
        case 2:
            g_CommuniCtrlMsg.TaskStage = 0;
            break;
        default:
            g_CommuniCtrlMsg.TaskStage = 0;
            g_CommuniCtrlMsg.TaskTime = g_SystemTime;
            break;
    }
}

