#include "main.h"
#include "FreeRTOS.h"
#include "string.h"
#include "task.h"
#include "httpapi.h"
#include "freertosconfig.h"
#include "stdio.h"
#include "app.h"
#include "motor.h"
#include "battery.h"
#include "listRunTask.h"
#include "hardware.h"

int GetPositionHttpApi( float *pos )
{
    GetPosition( pos );
    return 0;
}
int GetSpeedHttpApi( float *sp )
{
    GetSpeed( sp );
    return 0;
}
int SetPositionHttpApi( float pos )
{
    NavigationOperationStd navigationOperationData;
    navigationOperationData.cmd = 3;
    navigationOperationData.Data.posTo = pos;
    if( xQueueSend( NavigationOperationQue, &navigationOperationData, 100 ) == pdPASS )
    {
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}
int SetOpHttpApi( float position, int cmd, float data )
{
    /*
    1: 设置速度
    2: 伸摆杆
    3：收摆杆
    4：伸摆杆到位确认
    5：收摆杆到位确认
    6：清零
    */
    NavigationOperationStd navigationOperationData;
    navigationOperationData.cmd =  4;
    navigationOperationData.Data.op = cmd;
    navigationOperationData.Data.posTo = position;
    if( cmd == 1 )
        navigationOperationData.Data.speedTo = data;
    else
        navigationOperationData.Data.speedTo = 0;
    if( xQueueSend( NavigationOperationQue, &navigationOperationData, 100 ) == pdPASS )
    {
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}
void beltOpHttpApi( int cmd )
{
    switch( cmd )
    {
        case 0:
            HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( OUT_6_GPIO_Port, OUT_6_Pin, GPIO_PIN_RESET );
        break;
        case 1:
            HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_SET );
            HAL_GPIO_WritePin( OUT_6_GPIO_Port, OUT_6_Pin, GPIO_PIN_SET );
        break;
        case 2:
            HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( OUT_6_GPIO_Port, OUT_6_Pin, GPIO_PIN_SET );
        break;
        case 3:
            HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_SET );
            HAL_GPIO_WritePin( OUT_6_GPIO_Port, OUT_6_Pin, GPIO_PIN_RESET );
        break;
        default:
            break;
    }
}
int InOutStatus()
{
    return getSwitchStatus();
}

void getTaskStatus( uint8_t * buff )
{
    const char * const pcHeader = "Task            Abs Time      % Time\r\n****************************************\r\n";
    strcpy( (char*)buff, pcHeader );
    vTaskGetRunTimeStats( (char*)buff + strlen( (char *) buff ) );
}
void getTaskStack( uint8_t *buff )
{
    const char *const pcHeader = "Task          State  Priority  Stack	#\r\n**************************************\r\n";
    strcpy( (char *)buff, pcHeader );
    vTaskList( (char *)buff + strlen( (char *)buff ) );
}
void getRamStatus( uint8_t *buff )
{
    size_t free = xPortGetFreeHeapSize();
    size_t freeMin = xPortGetMinimumEverFreeHeapSize();
    sprintf( (char *)buff, "Type\tTotal\tFree\tFreeMin\r\n**************************************\r\n" );
    sprintf( (char *)(buff + strlen( (char *)buff ) ), "RAM:\t%u kb\t%u kb\t%u kb\r\n", configTOTAL_HEAP_SIZE / 1024, free/1024, freeMin/1024 );
}