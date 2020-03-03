/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* ----------------------- System includes ----------------------------------*/
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

/* ----------------------- Platform includes --------------------------------*/
#include "usart.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbport.h"
#include "mbm.h"
#include "common/mbportlayer.h"

#define MBM_SERIAL_PORT                 ( 0 )
#define MBM_SERIAL_BAUDRATE             ( 38400 )
#define MBM_PARITY                      ( MB_PAR_NONE )

void modbusTask( void const * arg )
{
    USHORT          usRegCnt = 0;
    eMBErrorCode    eStatus, eStatus2;
    xMBHandle       xMBMMaster;
    USHORT          usNRegs[5];



    if( MB_ENOERR ==
            ( eStatus = eMBMSerialInit( &xMBMMaster, MB_RTU, MBM_SERIAL_PORT, MBM_SERIAL_BAUDRATE, MBM_PARITY ) ) )
    {
        do
        {
#if 1
            /* Write an incrementing counter to register address 0. */
            /*
             if( MB_ENOERR != ( eStatus2 = eMBMWriteSingleRegister( xMBMMaster, 1, 0, usRegCnt++ ) ) )
             {
                 eStatus = eStatus2;
             }
             vTaskDelay( 100 );
             */
            /* Read holding register from adress 5 - 10, increment them by one and store
             * them at address 10.
             */

            if( MB_ENOERR != ( eStatus2 = eMBMReadHoldingRegisters( xMBMMaster, 1, 0x3600, 1, usNRegs ) ) )
            {
                eStatus = eStatus2;
            }

#endif
            switch ( eStatus )
            {
            case MB_ENOERR:
                break;
            default:
                break;
            }
            vTaskDelay( 1 );
        }
        while( TRUE );
    }
    else
    {
        MBP_ASSERT( 0 );
    }
    if( MB_ENOERR != ( eStatus = eMBMClose( xMBMMaster ) ) )
    {
        MBP_ASSERT( 0 );
    }
}