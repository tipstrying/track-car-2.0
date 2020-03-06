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
#include "hardware.h"

#define MBM_SERIAL_PORT                 ( 0 )
#define MBM_SERIAL_BAUDRATE             ( 38400 )
#define MBM_PARITY                      ( MB_PAR_NONE )

#define BeltAddr    2
#define SwitchAddr  3

QueueHandle_t SwitchBeltTaskQue = 0;
int switchRunning = 0;
InOutSwitch target;

void modbusTask( void const * arg )
{
    xMBHandle       xMBMMaster;
    USHORT modbusReadBackRegs[10];
    NavigationOperationStd switchBeltTaskData;

    debugOut(0, "[\t%d] Modbus task start up [ok]\r\n", osKernelSysTick() );
    debugOut(0, "[\t%d] Start Create Switch and Belt Task QueueHandle\r\n", osKernelSysTick() );
    do
    {
        SwitchBeltTaskQue = xQueueCreate( 10, sizeof( NavigationOperationStd ) );
    } while( !SwitchBeltTaskQue );
    debugOut(0, "[\t%d] Create Switch and Belt Task QueueHandle [ok]\r\n", osKernelSysTick() );
    debugOut(0, "[\t%d] Start init Modbus\r\n", osKernelSysTick() );
    if( MB_ENOERR == eMBMSerialInit( &xMBMMaster, MB_RTU, MBM_SERIAL_PORT, MBM_SERIAL_BAUDRATE, MBM_PARITY ) )
    {
        /*
         while( Battery.Voltage < 25000 )
         {
             osDelay(10);
         }
         */
        debugOut(0, "[\t%d] Battery voltage up to 25000mV [ok]\r\n", osKernelSysTick() );
        debugOut(0, "[\t%d] Start set up Belt Motor [ok]\r\n", osKernelSysTick() );
        for( ;; )
        {
            while( MB_ENOERR != eMBMReadHoldingRegisters( xMBMMaster, BeltAddr, 0x3600, 1, modbusReadBackRegs ) )
            {
                osDelay(3);
            }
            if( modbusReadBackRegs[0] != 3 )
            {
                debugOut(0, "[\t%d] Belt Motor Clean alarm\r\n", osKernelSysTick() );
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x80 ) )
                {
                    osDelay(2);
                }
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x06 ) )
                {
                    osDelay(2);
                }
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x0f ) )
                {
                    osDelay(2);
                }
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3500, 0x03 ) )
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

        debugOut(0, "[\t%d] Start set up Switch Motor [ok]\r\n", osKernelSysTick() );
        for( ;; )
        {
            union
            {
                int iData;
                USHORT uData[2];
            } iToUShortData;
            iToUShortData.iData = 16384000 * 2;
            while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x4a00, 2, iToUShortData.uData ) )
            {
                osDelay(2);
            }
            while( MB_ENOERR != eMBMReadHoldingRegisters( xMBMMaster, SwitchAddr, 0x3600, 1, modbusReadBackRegs ) )
            {
                osDelay(3);
            }
            if( modbusReadBackRegs[0] != 3 )
            {
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, SwitchAddr, 0x3100, 0x80 ) )
                {
                    osDelay(2);
                }
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, SwitchAddr, 0x3100, 0x06 ) )
                {
                    osDelay(2);
                }
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, SwitchAddr, 0x3100, 0xaf ) )
                {
                    osDelay(2);
                }
                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, SwitchAddr, 0x3500, 0x03 ) )
                {
                    osDelay(2);
                }
                iToUShortData.iData = 16384000 * 2;
                while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x4a00, 2, iToUShortData.uData ) )
                {
                    osDelay(2);
                }
            }
            else
            {
                debugOut(0, "[\t%d] set Switch motor status 3 [ok]\r\n", osKernelSysTick() );
                break;
            }
        }

        target = getSwitchStatus();
        if( target == InOutSwitchUnknow )
        {
            target = InOutSwitchIn;
            union
            {
                int iData;
                USHORT uData[2];
            } iToUShortData;

            iToUShortData.iData = 491520;

            while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x6f00, 2, iToUShortData.uData ) )
            {
                osDelay(2);
            }
            for( ;; )
            {
                if( getSwitchStatus() == InOutSwitchIn )
                {
                    iToUShortData.iData = 0;
                    while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x6f00, 2, iToUShortData.uData ) )
                    {
                        osDelay(2);
                    }
                    break;
                }
            }
        }
        // if go here, switch is at right pos, in or out all right.

        int switchStatus = 0; // switch status: 0->preInit 1->initOk 2->NewTargePositionStep 3->NewTargetSpeedStep 4->Waitting 5->targetSwitchFinish
        // Belt Ctrl:
        union
        {
            int iData;
            USHORT uData[2];
        } iToUShortData;

        int switchMotorMode = 0;
        InOutSwitch targetGoing = target;

        for( ;; )
        {
            if( xQueueReceive( SwitchBeltTaskQue, &switchBeltTaskData, 5 ) == pdPASS )
            {
                switch( switchBeltTaskData.cmd )
                {
                case 1: // new switch target message
                    // target = switchBeltTaskData.Data.op;
                    break;
                case 2: // thing IO message

                    break;
                case 3: // Belt operation message
                    switch( switchBeltTaskData.Data.op )
                    {
                    default:
                    case 0:
                        iToUShortData.iData = 0;
                        while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, BeltAddr, 0x6f00, 2, iToUShortData.uData ) )
                            osDelay(2);
                        break;
                    case -1:
                        iToUShortData.iData = -8912000;
                        while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, BeltAddr, 0x6f00, 2, iToUShortData.uData ) )
                            osDelay(2);
                        break;
                    case 1:
                        iToUShortData.iData = 8912000;
                        while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, BeltAddr, 0x6f00, 2, iToUShortData.uData ) )
                            osDelay(2);
                        break;
                    }
                default:
                    break;

                }
            }
            else
            {
                switch( switchStatus )
                {
                case 0:
                    switchStatus = 5;
                    break;
                case 1:
                    switchStatus = 5;
                    break;
                case 2:
                    debugOut(0, "[\t%d] Switch Case 2 target->%d\r\n", osKernelSysTick(), targetGoing );
                    if( switchMotorMode != 1 )
                    {
                        modbusReadBackRegs[0] = 0;
                        while( MB_ENOERR != eMBMReadHoldingRegisters( xMBMMaster, SwitchAddr, 0x3600, 1, modbusReadBackRegs ) )
                            osDelay(2);
                        debugOut(0, "[\t%d] Switch Mode Read\r\n", osKernelSysTick() );
                        if( modbusReadBackRegs[0] != 1 )
                        {
                            debugOut(0, "[\t%d] Switch Motor Mode charge\r\n", osKernelSysTick() );
                            do
                            {
                                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x80 ) )
                                {
                                    osDelay(2);
                                }
                                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x06 ) )
                                {
                                    osDelay(2);
                                }
                                while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x0f ) )
                                {
                                    osDelay(2);
                                }
                                /*
                                    Be careful!!!!!!!
                                    Motor Charge to Position Mode from Speed Mode will auto run once, if 0x4000 REG not 0 !!!! ;

                                */
                                iToUShortData.iData = 0;
                                while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x4000, 2, iToUShortData.uData ) )
                                    osDelay(2);
                                eMBMWriteSingleRegister( xMBMMaster, SwitchAddr, 0x3500, 1 );
                                while( MB_ENOERR != eMBMReadHoldingRegisters( xMBMMaster, SwitchAddr, 0x3600, 1, modbusReadBackRegs ) )
                                    osDelay(2);
                                iToUShortData.iData = 16384000 * 2;
                                while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x4a00, 2, iToUShortData.uData ) )
                                {
                                    osDelay(2);
                                }
                            } while( modbusReadBackRegs[0] != 1 );
                        }
                        switchMotorMode = 1;
                    }
                    debugOut(0, "[\t%d] Switch Mode Checked Ok\r\n", osKernelSysTick() );
                    if( targetGoing == InOutSwitchIn )
                    {
                        iToUShortData.iData = 167550;
                    }
                    else
                    {
                        iToUShortData.iData = -167550;
                    }
                    while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x4000, 2, iToUShortData.uData ) )
                        osDelay(2);
                    debugOut(0, "[\t%d] Switch Send Position Ok\r\n", osKernelSysTick() ) ;

                    while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, SwitchAddr, 0x3100, 0x6f ) )
                        osDelay(2);

                    while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, SwitchAddr, 0x3100, 0x7f ) )
                        osDelay(2);
                    debugOut(0, "[\t%d] Switch Ctrl Word Send Ok\r\n", osKernelSysTick() );
                    switchStatus = 3;
                    osDelay( 100 );
                    break;
                case 3:
                    debugOut(0, "[\t%d] Switch Case 3\r\n", osKernelSysTick() );
                    if( targetGoing != getSwitchStatus() )
                    {
                        do
                        {
                            while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x06 ) )
                            {
                                osDelay(2);
                            }
                            while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x0f ) )
                            {
                                osDelay(2);
                            }

                            while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, SwitchAddr, 0x3500, 3 ) )
                                osDelay(2);
                            while( MB_ENOERR != eMBMReadHoldingRegisters( xMBMMaster, SwitchAddr, 0x3600, 1, modbusReadBackRegs ) )
                            {
                                osDelay(3);
                            }
                        } while( modbusReadBackRegs[0] != 3 );
                        switchMotorMode = 3;
                        if( targetGoing == InOutSwitchIn )
                            iToUShortData.iData = 491520;
                        else
                            iToUShortData.iData = -491520;

                        while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x6f00, 2, iToUShortData.uData ) )
                            osDelay(2);
                        switchStatus = 4;
                    }
                    else
                        switchStatus = 5;
                    break;
                case 4:
                    if( targetGoing == getSwitchStatus() )
                    {
                        iToUShortData.iData = 0;
                        while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x6f00, 2, iToUShortData.uData ) )
                            osDelay(2);
                        do
                        {
                            while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x06 ) )
                            {
                                osDelay(2);
                            }
                            while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x6f ) )
                            {
                                osDelay(2);
                            }
                            /*
                                Be careful!!!!!!!
                                Motor Charge to Position Mode from Speed Mode will auto run once, if 0x4000 REG not 0 !!!! ;

                            */
                            iToUShortData.iData = 0;
                            while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x4000, 2, iToUShortData.uData ) )
                                osDelay(2);
                            while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, SwitchAddr, 0x3500, 1 ) )
                                osDelay(2);
                            while( MB_ENOERR != eMBMReadHoldingRegisters( xMBMMaster, SwitchAddr, 0x3600, 1, modbusReadBackRegs ) )
                                osDelay(2);
                            iToUShortData.iData = 16384000 * 2;
                            while( MB_ENOERR != eMBMWriteMultipleRegisters( xMBMMaster, SwitchAddr, 0x4a00, 2, iToUShortData.uData ) )
                            {
                                osDelay(2);
                            }
                        } while( modbusReadBackRegs[0] != 1 );
                        switchMotorMode = 1;
                        switchStatus = 5;
                    }
                    break;

                case 5:
                    /* for auto test */
                    //debugOut(0, "[\t%d] Switch Case 5\r\n", osKernelSysTick() );
                    //target = target == InOutSwitchIn ? InOutSwitchOut : InOutSwitchIn;
                    /* for auto test end */
                    if( target == InOutSwitchIn || target == InOutSwitchOut )
                        targetGoing = target;
                    if( target != getSwitchStatus() )
                    {

                        switchStatus = 2;
                        switchRunning = 1;
                    }
                    else
                        switchRunning = 0;
                    break;

                }

            }
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

int beltCtrl( int isRun, BeltDirectionDef dir, int speed )
{
    union
    {
        int iData;
        USHORT uData[2];
    } iToUShortData;

    static int count = 0;
    static BeltDirectionDef dirBak;
    static int isRunBak;
    NavigationOperationStd BeltCmdData;

    if( (dir != dirBak) || (isRunBak != isRun) )
    {
        dirBak = dir;
        isRunBak = isRun;
        if( isRun )
        {
            if( dir == BeltRev )
            {
                BeltCmdData.cmd = 3;
                BeltCmdData.Data.op = 1;
                xQueueSend( SwitchBeltTaskQue, &BeltCmdData, 10 );
                /*
                iToUShortData.iData = 8912000;
                eMBMWriteMultipleRegisters( xMBMMaster, BeltAddr, 0x6f00, 2, iToUShortData.uData );
                //     HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_SET );
                //     HAL_GPIO_WritePin( OUT_6_GPIO_Port, OUT_6_Pin, GPIO_PIN_SET );
                */
            }
            else
            {
                BeltCmdData.cmd = 3;
                BeltCmdData.Data.op = -1;
                xQueueSend( SwitchBeltTaskQue, &BeltCmdData, 10 );
                /*
                iToUShortData.iData = -8912000;
                eMBMWriteMultipleRegisters( xMBMMaster, BeltAddr, 0x6f00, 2, iToUShortData.uData );
                //HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_RESET );
                //HAL_GPIO_WritePin( OUT_6_GPIO_Port, OUT_6_Pin, GPIO_PIN_SET );
                */

            }
        }
        else
        {
            BeltCmdData.cmd = 3;
            BeltCmdData.Data.op = 0;
            xQueueSend( SwitchBeltTaskQue, &BeltCmdData, 10 );
            /*
            iToUShortData.iData = 0;
            eMBMWriteMultipleRegisters( xMBMMaster, BeltAddr, 0x6f00, 2, iToUShortData.uData );
            */
            /*
            HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( OUT_6_GPIO_Port, OUT_6_Pin, GPIO_PIN_RESET );
            */
        }
    }
}