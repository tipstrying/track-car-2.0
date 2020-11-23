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
int GetPosHttpApi(float *pos)
{
    GetNextPiont(pos);
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
int GetMotionStatusHttpApi()
{
    int status = GetMotionStatus();
    return status;
}
int getMilagesHttpApi ( double * mils )
{
    *mils = GetMilage();
    return 0;
}
int GetMotorCurrentHttpApi( float *current )
{
    short c1;
    GetMotorCurrent( &c1 );

    if( c1 < 0 )
    {
        c1 = -c1;
    }

    *current = ((float)c1) / 55.7;
    return 0;
}
void GetIOStatusHttpApi(uint8_t *IOString)
{
    GPIO_TypeDef * InPort[] = { IN_1_GPIO_Port, IN_2_GPIO_Port, IN_3_GPIO_Port, IN_4_GPIO_Port, IN_5_GPIO_Port, IN_6_GPIO_Port, IN_7_GPIO_Port, IN_8_GPIO_Port, IN_9_GPIO_Port, IN_10_GPIO_Port, IN_11_GPIO_Port, IN_12_GPIO_Port, IN_13_GPIO_Port, IN_14_GPIO_Port, IN_15_GPIO_Port, IN_16_GPIO_Port, IN_17_GPIO_Port, IN_18_GPIO_Port, IN_19_GPIO_Port, IN_20_GPIO_Port };
    uint16_t       InPin[] = { IN_1_Pin, IN_2_Pin, IN_3_Pin, IN_4_Pin, IN_5_Pin, IN_6_Pin, IN_7_Pin, IN_8_Pin, IN_9_Pin, IN_10_Pin, IN_11_Pin, IN_12_Pin, IN_13_Pin, IN_14_Pin, IN_15_Pin, IN_16_Pin, IN_17_Pin, IN_18_Pin, IN_19_Pin, IN_20_Pin };
    GPIO_TypeDef * OutPort[] = { OUT_1_GPIO_Port, OUT_2_GPIO_Port, OUT_3_GPIO_Port, OUT_4_GPIO_Port, OUT_5_GPIO_Port, OUT_6_GPIO_Port, OUT_7_GPIO_Port, OUT_8_GPIO_Port, OUT_9_GPIO_Port, OUT_10_GPIO_Port };
    uint16_t       OutPin[] = { OUT_1_Pin, OUT_2_Pin, OUT_3_Pin, OUT_4_Pin, OUT_5_Pin, OUT_6_Pin, OUT_7_Pin, OUT_8_Pin, OUT_9_Pin, OUT_10_Pin };
    //static int finish = 0;
    //sprintf( IOString, "IO Status\r\n" );
    sprintf( (char *)IOString + strlen( (char *)IOString ), "InPut:\r\n" );

    for( int i = 0; i < ( sizeof( InPort) / sizeof( InPort[0] ) ); i++ )
    {
        sprintf( (char *)IOString + strlen( (char *)IOString ), "IN %d -> %d\t", i + 1, HAL_GPIO_ReadPin( InPort[i], InPin[i] ) );
    }

    sprintf( (char *)IOString + strlen( (char *)IOString ), "\r\nOutPut:\r\n" );

    for( int i = 0; i < ( sizeof( OutPort) / sizeof( OutPort[0] ) ); i++ )
    {
        sprintf( (char *)IOString + strlen( (char *)IOString ), "Out %d -> %d\t", i + 1, HAL_GPIO_ReadPin( OutPort[i], OutPin[i] ) );
    }

    sprintf( (char *)IOString + strlen( (char *)IOString ), "\r\n" );
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
    {
        navData.Data.op = 1;
    }
    else
    {
        navData.Data.op = 0;
    }

    return xQueueSend( NavigationOperationQue, &navData, 100 );
}
int clearMotoralarmHttpApi()
{
    ClearMotorAlarm();
    return 0;
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
    {
        navigationOperationData.Data.speedTo = data;
    }
    else
    {
        navigationOperationData.Data.speedTo = 0;
    }

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
int SetSleepModeHttpApi( int isSleep )
{
    NavigationOperationStd navigationOperationData;

    if( isSleep )
    {
        navigationOperationData.cmd = Enum_SetSleep;
        navigationOperationData.Data.op = 1;
    }
    else
    {
        navigationOperationData.cmd = Enum_SetSleep;
        navigationOperationData.Data.op = 0;
    }

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
//int InOutStatus()
//{
//    return getSwitchStatus();
//}
int GetBatteryVoltageHttpApi( float *Voltage )
{
    /* Battery.Voltage; */
    unsigned short v;
    GetMotorVoltage( &v );
    *Voltage = v;
    return 0;
}
int cancelNavigationHttpApi()
{
    NavigationOperationStd navData;
    navData.cmd = Enum_CancelNavigation;
    navData.Data.op = 1;

    if( xQueueSend( NavigationOperationQue, &navData, 100 ) == pdPASS )
    {
        return 0;
    }
    else
    {
        return 1;
    }
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