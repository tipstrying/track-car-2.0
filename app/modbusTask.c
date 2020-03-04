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
#include "app.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbport.h"
#include "mbm.h"
#include "common/mbportlayer.h"

#define MBM_SERIAL_PORT                 ( 0 )
#define MBM_SERIAL_BAUDRATE             ( 38400 )
#define MBM_PARITY                      ( MB_PAR_NONE )

void modbusTask( void const * arg )
{
    xMBHandle       xMBMMaster;
    USHORT modbusReadBackRegs[10];
    if( MB_ENOERR == eMBMSerialInit( &xMBMMaster, MB_RTU, MBM_SERIAL_PORT, MBM_SERIAL_BAUDRATE, MBM_PARITY ) )
    {
        debugOut(0, "[\t%d] Modbus task start up [ok]\r\n", osKernelSysTick() );
       /*
        while( Battery.Voltage < 25000 )
        {
            osDelay(10);
        }
        */
        debugOut(0, "[\t%d] Battery voltage up to 25000mV [ok]\r\n", osKernelSysTick() );
        for( ;; )
        {
            while( MB_ENOERR != eMBMReadHoldingRegisters( xMBMMaster, 1, 0x3600, 1, modbusReadBackRegs ) )
            {
                osDelay(3);
            }
            if( modbusReadBackRegs[0] != 3 )
            {
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, 1, 0x3100, 0x80 ) )
                {
                    osDelay(2);
                }
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, 1, 0x3100, 0x06 ) )
                {
                    osDelay(2);
                }
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, 1, 0x3500, 0x03 ) )
                {
                    osDelay(2);
                }
            }
            else
            {
                debugOut(0, "[\t%d] set belt motor status 3 [ok]\r\n", osKernelSysTick() );
                break;
            }
        }
        union
        {
            int iData;
            USHORT uData[2];
        }iToUShortData;
        
        for( ;; )
        {
            iToUShortData.iData = 8192000;
            debugOut(0, "[\t%d] set belt speed 100RPM\r\n", osKernelSysTick() );
            eMBMWriteMultipleRegisters( xMBMMaster, 1, 0x6f00, 2, iToUShortData.uData );
            osDelay( 1000 );
            iToUShortData.iData = 0;
            debugOut(0, "[\t%d] set belt speed 0RPM\r\n", osKernelSysTick() );
            eMBMWriteMultipleRegisters( xMBMMaster, 1, 0x6f00, 2, iToUShortData.uData );
            osDelay( 5000 );
        }
    }
    else
    {
        MBP_ASSERT( 0 );
    }
    if( MB_ENOERR != eMBMClose( xMBMMaster ) )
    {
        MBP_ASSERT( 0 );
    }
}