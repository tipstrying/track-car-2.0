#include "gpio.h"
#include "spi.h"
#include "w5500.h"
#include "socket.h"
#include "string.h"
#include "usart.h"
#include "tim.h"
#include "ff.h"
#include "fatfs.h"
#include "battery.h"
#include "app.h"
#include "motor.h"
#include "stdio.h"
#include "hardware.h"

QueueHandle_t NavigationOperationQue = 0;                            // 网络消息命令
DebugOutCtlDef DebugCtrl;
QueueHandle_t forkOperationQue = 0;
QueueHandle_t chargeOperationQue = 0;

// StatusStd mySelf;                                       // 小车状�
FATFS fatfs;                                            // 文件系统
FIL file;                                               // 文件句柄
int fatfsstatus = 0;                                    // 上电默认文件系统未挂�
void modbusTask( void const * arg );
void uart2RecTask( void const *arg );

void InitTask( void const * parment )
{
    HAL_GPIO_WritePin( OUT_2_GPIO_Port, OUT_2_Pin, GPIO_PIN_SET );

    if( NavigationOperationQue == 0 )
        NavigationOperationQue = xQueueCreate( 5, sizeof( NavigationOperationStd ) );

    taskENTER_CRITICAL();
    {
        printf( "Boot Up Ok\r\n" );
    }
    taskEXIT_CRITICAL();

    osThreadDef( u2RecService, uart2RecTask, osPriorityHigh, 0, 256 );
    osThreadCreate( osThread( u2RecService ), NULL);

    osThreadDef( BatteryTask, UartTask, osPriorityHigh, 0, 256 );
    osThreadCreate( osThread( BatteryTask ), NULL);

    osThreadDef(MotionTask, MotionTask, osPriorityRealtime, 0, 4096 );
    osThreadCreate(osThread(MotionTask), NULL);

    osThreadDef( EthernetTask, W5500Task, osPriorityHigh, 0, 2000  );
    osThreadCreate( osThread( EthernetTask ), NULL);

    osThreadDef( modbus, modbusTask, osPriorityHigh, 0, 1280 );
    osThreadCreate( osThread( modbus ), NULL);



    uint32_t tick = osKernelSysTick();

    vTaskDelete( NULL );
    for( ;; )
    {
        osDelay( 1000 );
    }
}

void StartPwm( void )
{
    ;
}

/*
    FreeRTOS TickHook
*/
void vApplicationTickHook( void )
{
    static int i = 0;
    if( i < 1000 )
    {
        i++;
    }
    else
    {
        /* 心跳�*/
        HAL_GPIO_TogglePin( SLEEP_GPIO_Port, SLEEP_Pin );
        HAL_GPIO_TogglePin( OUT_2_GPIO_Port, OUT_2_Pin );
        i = 0;
    }
}

unsigned long getRunTimeCounterValue(void)
{
    return HAL_GetTick();
}

void *malloc( unsigned int size )
{
    return pvPortMalloc( size );
}

void free( void *p )
{
    vPortFree( p );
}
