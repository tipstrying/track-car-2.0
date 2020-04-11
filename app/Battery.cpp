#include "battery.h"
#include "app.h"
#include "usart.h"
#include "fifo.h"
#include "stdio.h"
#include "string.h"
#include "motor.h"
#include "hardware.h"
/*
#define CHARGE_CONNECT              0x01            //充电连接
#define CHARGE_OVERC                0x02            //充电过流
#define DISCHARGE                   0x08            //正常放电
#define DISCHARGE_OVERC             0x10            //放电过流
#define DISCHARGE_SHORT_CI          0x20            //放电短路
*/
/* BITs */
#define BIT1 0x01
#define BIT2 0x02
#define BIT3 0x04
#define BIT4 0x08
#define BIT5 0x10
#define BIT6 0x20
#define BIT7 0x40
#define BIT8 0x80

union {
    int data;
    uint8_t Hex[4];
} IntToHex;

QueueHandle_t BatterQueRx = 0;
SemaphoreHandle_t U3_Sema_Rx = 0;
SemaphoreHandle_t U2_Sema_Rx = 0;
BattreyStd Battery;

uint16_t voltageRealTime = 0;

void UartTask(void const *par)
{
    unsigned char U3_485_buff[100];
    uint16_t Volatage_every_buff[100];
    int i = 0;
    //uint8_t checksum;
    char rambuff[20];
    //uint8_t SendBuff[] = { 0x7f, 0x10, 0x02, 0x06, 0x10, 0x59 };
    uint8_t SendBuff[] = {0x01, 0x03, 0x00, 0x56, 0x00, 0x01, 0x64, 0x1a};
    U3_Sema_Rx = xSemaphoreCreateBinary();
    uint8_t Status;
    uint16_t voltageRealTime = 0;

    if (DebugCtrl.enableStartUp)
    {
        taskENTER_CRITICAL();
        {
            printf("[\t%d] Info: Battery Task Start Ok\r\n", osKernelSysTick());
        }
        taskEXIT_CRITICAL();
    }
    //__HAL_UART_ENABLE_IT( &huart7, UART_IT_IDLE );
    for (;;)
    {
        HAL_UART_Receive_DMA(&huart2, U3_485_buff, 7);
        HAL_GPIO_WritePin(USART2_RD_GPIO_Port, USART2_RD_Pin, GPIO_PIN_SET);
        // HAL_UART_Transmit_DMA( &huart2, SendBuff, sizeof( SendBuff ) );
        HAL_UART_Transmit(&huart2, SendBuff, sizeof(SendBuff), 1000);
        HAL_GPIO_WritePin(USART2_RD_GPIO_Port, USART2_RD_Pin, GPIO_PIN_RESET);
        if (xSemaphoreTake(U3_Sema_Rx, 100) == pdPASS)
        {

            voltageRealTime = U3_485_buff[3] * 255 + U3_485_buff[4];
            voltageRealTime = voltageRealTime * 10;
            Volatage_every_buff[i++] = voltageRealTime;
            voltageRealTime = 0;
            if(i >= 100)
                i = 0;
            for(int j = 0; j < 100; j++)
            {
                voltageRealTime += Volatage_every_buff[j];
            }
            voltageRealTime = voltageRealTime / 100;
            Battery.Voltage = voltageRealTime;

            /*
            if( U3_485_buff[0] == 0xDD )
            {
                Status = 0;
                for( ; Status < 13; )
                {
                    switch( Status )
                    {
                    case 0:
                        if( U3_485_buff[0] == 0xDD )
                        {
                            Status = 1;
                        }
                        else
                            Status = 255;
                        break;
                    case 1:
                        if( U3_485_buff[1] == 0x03 )
                            Status = 2;
                        else
                            Status = 255;
                        break;
                    case 2:
                        if( U3_485_buff[2] == 0x00)
                            Status = 3;
                        else
                            Status = 255;
                        break;
                    case 3:
                        if(U3_485_buff[3] == 0X1B)
                            Status = 4;
                        break;
                    case 4:
                        //if( U3_485_buff[4] == 0x10 )
                        Battery.Voltage = 0;
                        Battery.Voltage = ((U3_485_buff[4] << 8) + U3_485_buff[5])*10;

                        Battery.Battery = ((U3_485_buff[8] << 8) + U3_485_buff[9])*10;

                        Battery.BatteryFull = ((U3_485_buff[10] << 8) + U3_485_buff[11])*10;
                        Battery.BatterStatus = 0;
                        Status = 255;
                        break;
                    case 255:
                        break;
                    default:
                        Status = 15;
                        break;
                    }
                }
                memset( U3_485_buff,0, sizeof(U3_485_buff));
            }
            else
            {
                Battery.BatterStatus ++;
                HAL_UART_AbortReceive_IT( &huart2 );
                if( DebugCtrl.enableBattery )
                {
                    taskENTER_CRITICAL();
                    {
                        printf("[\t%d] Battery Data Error Header\r\n Data: ", osKernelSysTick() );
                        for( int i = 0; i < 100; i++ )
                        {
                            printf( "0x%X ", U3_485_buff[i] );
                        }
                        printf( "\r\n" );
                    }
                    taskEXIT_CRITICAL();
                }
            }
            */
        }
        else
        {
            // HAL_UART_AbortReceive_IT(&huart2);
            HAL_UART_AbortReceive(&huart2);
            if (DebugCtrl.enableBattery)
            {
                taskENTER_CRITICAL();
                {
                    printf("[\t%d] Rec TimeOut\r\n", osKernelSysTick());
                }
                taskEXIT_CRITICAL();
            }
        }

        osDelay(1000);
    }
}

QueueHandle_t U1_Sema_Rx = 0;
static uint8_t u1Buff[100];

extern "C"
{
    void USART1_IRQHandler(void);
}
void USART1_IRQHandler(void)
{
    uint8_t data;
    data = huart1.Instance->SR;
    data = huart1.Instance->DR;
    __HAL_DMA_DISABLE(huart1.hdmarx);
    __HAL_DMA_ENABLE(huart1.hdmarx);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static BaseType_t pxHigherPriorityTaskWoken;
    char *c = 0;
    if (huart == &huart2)
    {
        if (U3_Sema_Rx)
            xSemaphoreGiveFromISR(U3_Sema_Rx, &pxHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}
