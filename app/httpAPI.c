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
#include "app.h"

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
int GetMaxSpeedHttpApi( float *sp )
{
    GetMaxSpeed( sp );
    return 0;
}
int getMilagesHttpApi ( double * mils )
{
    *mils = GetMilage();
    return 0;
}
int SetPositionHttpApi( float pos )
{
    NavigationOperationStd navigationOperationData;
    navigationOperationData.cmd = Enum_SendNavigation;
    navigationOperationData.Data.posTo = pos;
    if( xQueueSend( NavigationOperationQue, &navigationOperationData, 100 ) == pdPASS )
    {
        debugOut( 0, "[\t%d] Set Position HttpApi: %0.2f\r\n", pos );
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}
int setMotorDisable(int status )
{
    NavigationOperationStd navData;
    navData.cmd = Enum_disableMotor;
    if( status )
        navData.Data.op = 1;
    else
        navData.Data.op = 0;
    return xQueueSend( NavigationOperationQue, &navData, 100 );
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
    navigationOperationData.cmd = Enum_sendOperation;
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
int SetHandSpeedModeHttoApi( int isHandMode )
{
    NavigationOperationStd navigationOperationData;
    navigationOperationData.cmd = Enum_setHandSpeedMode;
    navigationOperationData.Data.op = isHandMode;
    if( xQueueSend( NavigationOperationQue, &navigationOperationData, 100 ) == pdPASS )
    {
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}
int SetHandSpeedModeSpeedHttpApi( int speed )
{
    NavigationOperationStd navigationOperationData;
    navigationOperationData.cmd = Enum_SetHandSpeed;
    navigationOperationData.Data.speedTo = speed;
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
int GetBatteryVoltageHttpApi( float *Voltage )
{
    *Voltage = Battery.Voltage;
    return 0;
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
    sprintf( (char *)(buff + strlen( (char *)buff ) ), "RAM:\t%u b\t%u b\t%u b\r\n", configTOTAL_HEAP_SIZE, free, freeMin );
}