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

#define MBM_SERIAL_PORT (0)
#define MBM_SERIAL_BAUDRATE (38400)
#define MBM_PARITY (MB_PAR_NONE)

#define BeltAddr 2
#define SwitchAddr 3

extern QueueHandle_t SwitchIN6Semap;
extern QueueHandle_t SwitchIN7Semap;

QueueHandle_t SwitchBeltTaskQue = 0;
int switchReach = -1;
InOutSwitch target;
InOutSwitch targetLast;
static float BeltSpeed = 2.0;
float getBeltSpeed()
{
    return BeltSpeed;
}
void setBeltSpeed(float speed)
{
    BeltSpeed = speed;
}
static struct
{
    BaseType_t timeStart;
    BaseType_t timeOut;
    int boolTimeout;
} timeOutData;
static int switchType = 0;


int beltCtrl(int isRun, BeltDirectionDef dir, int speed)
{
    union {
        int iData;
        USHORT uData[2];
    } iToUShortData;

    static int count = 0;
    static BeltDirectionDef dirBak;
    static int isRunBak;
    NavigationOperationStd BeltCmdData;

    if ((dir != dirBak) || (isRunBak != isRun))
    {
        dirBak = dir;
        isRunBak = isRun;
        if (isRun)
        {
            if (dir == BeltRev)
            {
                BeltCmdData.cmd = 3;
                BeltCmdData.Data.op = 1;
                xQueueSend(SwitchBeltTaskQue, &BeltCmdData, 10);
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
                xQueueSend(SwitchBeltTaskQue, &BeltCmdData, 10);
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
            xQueueSend(SwitchBeltTaskQue, &BeltCmdData, 10);
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