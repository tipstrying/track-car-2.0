/*
 * MODBUS Library: AT91SAM7X port
 * Copyright (c) 2010 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbportserial.c,v 1.1 2010-05-22 22:31:33 embedded-so.embedded-solutions.1 Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "common/mbtypes.h"
#include "common/mbportlayer.h"
#include "common/mbframe.h"
#include "common/mbutils.h"
#include "usart.h"
#include "semphr.h"
#include "FreeRTOS.h"

#include "app.h"

/* ----------------------- Defines ------------------------------------------*/

#define USART_INTERRUPT_LEVEL           ( AT91C_AIC_PRIOR_HIGHEST )
#define NUARTS                          ( 2 )
#define USART_USART0_IDX                ( 0 )
#define USART_USART1_IDX                ( 1 )

#define IDX_INVALID                     ( 255 )
#define UART_BAUDRATE_MIN               ( 300 )
#define UART_BAUDRATE_MAX               ( 115200 )

#define UART_INIT( ubIdx )      do { \
    if( USART_USART0_IDX == ubIdx ) \
    { \
        AT91F_PIO_CfgPeriph( AT91C_BASE_PIOA, AT91C_PA0_RXD0 | AT91C_PA1_TXD0 | AT91C_PA3_RTS0, 0 ); \
    } \
    else if( USART_USART1_IDX == ubIdx ) \
    { \
        AT91F_PIO_CfgPeriph( AT91C_BASE_PIOA, AT91C_PA5_RXD1 | AT91C_PA6_TXD1 | AT91C_PA8_RTS1, 0 ); \
    } \
    else \
    { \
        MBP_ASSERT( 0 ); \
    } \
} while( 0 )

#define HDL_RESET( x ) do { \
	( x )->ubIdx = IDX_INVALID; \
	( x )->pbMBPTransmitterEmptyFN = NULL; \
	( x )->pvMBPReceiveFN = NULL; \
	( x )->xMBHdl = MB_HDL_INVALID; \
} while( 0 );

/* ----------------------- Function prototypes ------------------------------*/
#if defined( __ICCARM__ ) && ( __ICCARM__ == 1 )
extern void     vUSART0ISRWrapper( void );
__arm void      vUSART0ISR( void );
extern void     vUSART1ISRWrapper( void );
__arm void      vUSART1ISR( void );
#endif

/* ----------------------- Type definitions ---------------------------------*/

typedef struct
{
    UBYTE           ubIdx;
    pbMBPSerialTransmitterEmptyAPIV2CB pbMBPTransmitterEmptyFN;
    pvMBPSerialReceiverAPIV2CB pvMBPReceiveFN;
    xMBHandle       xMBHdl;
} xMBPSerialIntHandle;

struct
{
    unsigned int    uiAT91C_ID_USX;
    void            ( *pvIRQHandlerFN ) ( void );
}
const           xMBPSerialHW[NUARTS];// = {
//};

/* ----------------------- Static variables ---------------------------------*/
STATIC xMBPSerialIntHandle xSerialHdls[NUARTS];
STATIC BOOL     bIsInitalized = FALSE;

/* ----------------------- Static functions ---------------------------------*/
#if defined( __ICCARM__ ) && ( __ICCARM__ == 1 )
__arm STATIC portBASE_TYPE xUSARTIRQHandler( UBYTE ubIdx );
#endif

/* ----------------------- Start implementation -----------------------------*/

eMBErrorCode
eMBPSerialInit( xMBPSerialHandle * pxSerialHdl, UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits,
                eMBSerialParity eParity, UCHAR ucStopBits, xMBHandle xMBHdl )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    xMBPSerialIntHandle *pxSerialIntHdl;
    unsigned int    uiUARTMode;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( !bIsInitalized )
    {
        HDL_RESET( &xSerialHdls[USART_USART0_IDX] );
        HDL_RESET( &xSerialHdls[USART_USART1_IDX] );
        bIsInitalized = TRUE;
    }

    pxSerialIntHdl = NULL;
    if( NULL != pxSerialHdl )
    {
        if( MB_ENOERR == eStatus )
        {
            pxSerialIntHdl = &xSerialHdls[ucPort];
            pxSerialIntHdl->ubIdx = ucPort;
            if( NULL != pxSerialIntHdl )
            {
                pxSerialIntHdl->xMBHdl = xMBHdl;
                *pxSerialHdl = pxSerialIntHdl;
                eStatus = MB_ENOERR;
            }
            else
            {
                eStatus = MB_ENORES;
            }
        }
    }
    else
    {
        eStatus = MB_EINVAL;
    }

    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialClose( xMBPSerialHandle xSerialHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBPSerialIntHandle *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        if( ( pxSerialIntHdl->pbMBPTransmitterEmptyFN == NULL ) && ( pxSerialIntHdl->pvMBPReceiveFN == NULL ) )
        {
            HDL_RESET( pxSerialIntHdl );
            eStatus = MB_ENOERR;
        }
        else
        {
            eStatus = MB_EAGAIN;
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialTxEnable( xMBPSerialHandle xSerialHdl, pbMBPSerialTransmitterEmptyCB pbMBPTransmitterEmptyFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBPSerialIntHandle *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        eStatus = MB_ENOERR;
        if( NULL != pbMBPTransmitterEmptyFN )
        {
            pxSerialIntHdl->pbMBPTransmitterEmptyFN = ( pbMBPSerialTransmitterEmptyAPIV2CB ) pbMBPTransmitterEmptyFN;
            UBYTE data[100];
            USHORT BytesToSend = 0;
            HAL_GPIO_WritePin( UART7_RD_GPIO_Port, UART7_RD_Pin, GPIO_PIN_SET );

            while( pxSerialIntHdl->pbMBPTransmitterEmptyFN( pxSerialIntHdl->xMBHdl, data, sizeof(data), &BytesToSend ) )
            {
/*
                debugOut(0, "[\t%d] Modbus:", osKernelSysTick() );
                for( int i = 0; i < BytesToSend; i++ )
                {
                    debugOut(0, "0x%X ", data[i] );
                }
                debugOut(0, "\r\n" );
*/
                HAL_UART_Transmit( &huart7, data, BytesToSend, 100 );
            }
            HAL_GPIO_WritePin( UART7_RD_GPIO_Port, UART7_RD_Pin, GPIO_PIN_RESET );
        }
        else
        {
            pxSerialIntHdl->pbMBPTransmitterEmptyFN = NULL;
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialRxEnable( xMBPSerialHandle xSerialHdl, pvMBPSerialReceiverCB pvMBPReceiveFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBPSerialIntHandle *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        eStatus = MB_ENOERR;
        if( NULL != pvMBPReceiveFN )
        {
            MBP_ASSERT( NULL == pxSerialIntHdl->pvMBPReceiveFN );
            HAL_GPIO_WritePin( UART7_RD_GPIO_Port, UART7_RD_Pin, GPIO_PIN_RESET );
            __HAL_UART_ENABLE_IT( &huart7, UART_IT_RXNE );
            pxSerialIntHdl->pvMBPReceiveFN = ( pvMBPSerialReceiverAPIV2CB ) pvMBPReceiveFN;
        }
        else
        {
            pxSerialIntHdl->pvMBPReceiveFN = NULL;
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}
static QueueHandle_t u2Seamp = 0;

void uart2RecTask( void const *arg )
{
    u2Seamp = xSemaphoreCreateBinary();
    UBYTE data[100];
    USHORT len = 0;
    __HAL_UART_ENABLE_IT( &huart7, UART_IT_IDLE );
    HAL_UART_Receive_DMA( &huart7, data, sizeof(data) );
    for( ;; )
    {
        if( xSemaphoreTake( u2Seamp, portMAX_DELAY ) == pdPASS )
        {
            len = sizeof(data) - huart7.hdmarx->Instance->NDTR;
            if( xSerialHdls[0].pvMBPReceiveFN )
                xSerialHdls[0].pvMBPReceiveFN( xSerialHdls[0].xMBHdl, data, len );
            HAL_UART_AbortReceive( &huart7 );
            HAL_UART_Receive_DMA( &huart7, data, sizeof(data) );
          /*  
            debugOut(0, "[\t%d] Modbus RecData:", osKernelSysTick() );
            for( int i = 0; i < len; i++ )
            {
                debugOut(0, "0x%X ", data[i] );
            }
            debugOut(0, "\r\n" );
            */
        }
    }
}
void UART7_IRQHandler(void)
{
    UBYTE data;
    BaseType_t next = 0;
    data = huart7.Instance->SR;
    data = huart7.Instance->DR;
    /*
    if( xSerialHdls[0].pvMBPReceiveFN )
    {
        xSerialHdls[0].pvMBPReceiveFN(xSerialHdls[0].xMBHdl, data );
    }
    */
    if( u2Seamp )
    {
        xSemaphoreGiveFromISR( u2Seamp, &next );
    }

}
