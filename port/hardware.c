#include "hardware.h"
#include "main.h"
#include "tim.h"
#include "rtc.h"

#include "mbport.h"
#include "mbm.h"
#include "common/mbportlayer.h"
#include "usart.h"
#include "app.h"

#define OUT_UpDownC_Port    OUT_9_GPIO_Port
#define OUT_UpDownC_Pin     OUT_9_Pin

#define OUT_UpDownB_Port    OUT_10_GPIO_Port
#define OUT_UpDownB_Pin     OUT_10_Pin

#define OUT_Motor_Enable_Port   OUT_6_GPIO_Port
#define OUT_Motor_Enable_Pin    OUT_6_Pin

#define IN_T1_Port  IN_2_GPIO_Port
#define IN_T1_Pin   IN_2_Pin

#define IN_T2_Port  IN_3_GPIO_Port
#define IN_T2_Pin   IN_3_Pin

#define IN_T3_Port  IN_4_GPIO_Port
#define IN_T3_Pin   IN_4_Pin

#define BeltAddr    2
#define SwitchAddr  3

static xMBHandle       xMBMMaster;
int prvInitHardwares ()
{
    USHORT modbusReadBackRegs[10];
    debugOut(0, "[\t%d] Waitting for Battery Voltage bigger then 25000mV\r\n", osKernelSysTick() );
    while( Battery.Voltage < 25000 )
    {
        osDelay(10);
    }
    debugOut(0, "[\t%d] Battery voltage up to 25000mV [ok]\r\n", osKernelSysTick() );

    debugOut(0, "[\t%d] Start hardware setup [ok]\r\n", osKernelSysTick() );
    if( MB_ENOERR == eMBMSerialInit( &xMBMMaster, MB_RTU, 0, 38400, MB_PAR_NONE ) )
    {

    }
    else
    {
        debugOut(0, "[\t%d] Modbus init error\r\n",osKernelSysTick() );
        return -1;
    }
    debugOut(0, "[\t%d] Start set up Belt Motor [ok]\r\n", osKernelSysTick() );
    for( ;; )
    {
        while( MB_ENOERR != eMBMReadHoldingRegisters( xMBMMaster, BeltAddr, 0x3600, 1, modbusReadBackRegs ) )
        {
            osDelay(3);
        }
        if( modbusReadBackRegs[0] != 3 )
        {
            while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x80 ) )
            {
                osDelay(2);
            }
            while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0x06 ) )
            {
                osDelay(2);
            }
            while( MB_ENOERR != eMBMWriteSingleRegister( xMBMMaster, BeltAddr, 0x3100, 0xaf ) )
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
        while( MB_ENOERR != eMBMReadHoldingRegisters( xMBMMaster, SwitchAddr, 0x3600, 1, modbusReadBackRegs ) )
        {
            osDelay(3);
        }
        if( modbusReadBackRegs[0] != 3 )
        {
            union
            {
                int iData;
                USHORT uData[2];
            } iToUShortData;

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
        }
        else
        {
            debugOut(0, "[\t%d] set Switch motor status 3 [ok]\r\n", osKernelSysTick() );
            break;
        }
    }

    return 0;
}

OnOffDef getThingSensor( int ID, int maxDelay )
{
    static int delay[3];
    switch( ID )
    {
    case 1:
        if( HAL_GPIO_ReadPin( IN_T1_Port, IN_T1_Pin ) )
        {
            if( delay[0] > 0 )
            {
                delay[0] --;
                return off;
            }
            else
                return on;
        }
        else
        {
            delay[0] = maxDelay;
            return off;
        }
    case 2:
        if( HAL_GPIO_ReadPin( IN_T2_Port, IN_T2_Pin ) )
        {
            if( delay[1] > 0 )
            {
                delay[1]--;
                return off;
            }
            else
                return on;
        }
        else
        {
            delay[1] = maxDelay;
            return off;
        }
    case 3:
        if( HAL_GPIO_ReadPin( IN_T3_Port, IN_T3_Pin ) )
        {
            if( delay[2] > 0 )
            {
                delay[2]--;
                return off;
            }
            else
                return on;
        }
        else
        {
            delay[2] = maxDelay;
            return off;
        }
    default:
        return off;
    }
}
int getThingSensorStatus( int type )
{
    if( type )
        return HAL_GPIO_ReadPin( IN_T2_Port, IN_T2_Pin );
    else
        return ( HAL_GPIO_ReadPin( IN_T1_Port, IN_T1_Pin )  || HAL_GPIO_ReadPin( IN_T2_Port, IN_T2_Pin ) || HAL_GPIO_ReadPin( IN_T3_Port, IN_T3_Pin ) );
}


int writePosToBKP( float position, double mils )
{
    union {
        float fData;
        uint32_t uData;
        uint8_t Hex[4];
    } u32Tof32;

    union {
        double dData;
        uint32_t uData[2];
        uint8_t Hex[8];
    } u64ToHex;
    u64ToHex.dData = mils;
    u32Tof32.fData = position;
    uint32_t sum = 0;
    for( int i = 0; i < 4; i++ )
    {
        sum += u32Tof32.Hex[i];
    }
    for( int i = 0; i < 8; i++ )
    {
        sum += u64ToHex.Hex[i];
    }
    HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR10, u32Tof32.uData );
    HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR11, u64ToHex.uData[0] );
    HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR12, u64ToHex.uData[1] );
    HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR13, sum );

}
/* BKP map:
DR0: bootloader flag-> "appM" load app
DR1: IP addr
DR2: MAC addr
DR3: Last IT Source
DR10: real-time position
DR11-DR12: milages
DR13: sum of real-time position and milages
DR14: switchType
*/
int readPosFromBKP( float *position, double *mils )
{
    union {
        float fData;
        uint32_t uData;
        uint8_t Hex[4];

    } u32Tof32;

    union {
        double dData;
        uint32_t uData[2];
        uint8_t Hex[8];
    } u64ToHex;

    u32Tof32.uData = HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR10 );
    u64ToHex.uData[0] = HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR11 );
    u64ToHex.uData[1] = HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR12 );

    uint32_t sum = 0;
    for( int i = 0; i < 4; i++ )
    {
        sum += u32Tof32.Hex[i];
    }
    for( int i = 0; i < 8; i++ )
    {
        sum += u64ToHex.Hex[i];
    }

    if( sum == HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR13 ) )
    {
        if( position && mils )
        {
            *position = u32Tof32.fData;
            *mils = u64ToHex.dData;
        }
        return 1;
    }
    else
        return 0;
}
void setITFlag( ISREnumDef isr )
{
    hrtc.Instance->BKP3R = isr;
}

int readSwitchTypeFromBKP(int *type)
{
    union {
        float fData;
        uint32_t uData;
        char Hex[4];
    } u32Tof32;

    char buff[10];
    memset( buff, 0, sizeof(buff) );
    u32Tof32.uData = HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR14 );
    memcpy( buff, u32Tof32.Hex, 4 );
    if( strcmp( "1Ser", buff ) == 0 )
    {
        *type = 1;
    }
    else if( strcmp( "2Ser", buff ) == 0 )
    {
        *type = 2;
    }
    else
        *type = 0;
    if( *type )
        return 1;
    else
        return 0;
}

int writeSwitchTypeFromBKP(int type)
{
    union {
        float fData;
        uint32_t uData;
        char Hex[4];
    } u32Tof32;

    switch( type )
    {
    case 1:
        u32Tof32.Hex[0] = '1';
        u32Tof32.Hex[1] = 'S';
        u32Tof32.Hex[2] = 'e';
        u32Tof32.Hex[3] = 'r';
        break;
    case 2:
        u32Tof32.Hex[0] = '2';
        u32Tof32.Hex[1] = 'S';
        u32Tof32.Hex[2] = 'e';
        u32Tof32.Hex[3] = 'r';
        break;
    default:
        u32Tof32.uData = 0;
        break;
    }
    HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR14, u32Tof32.uData );
    return 1;
}

InOutSwitch getSwitchStatus()
{
    static int type = -1;
    if( type == -1 )
    {
        if( readSwitchTypeFromBKP( &type ) == 0 )
            type = 2;
    }
    if( type == 2 )
    {
        if( HAL_GPIO_ReadPin( IN_6_GPIO_Port, IN_6_Pin ) && HAL_GPIO_ReadPin( IN_7_GPIO_Port, IN_7_Pin ) )
        {
            return InOutSwitchIn;
        }
        if( HAL_GPIO_ReadPin( IN_7_GPIO_Port, IN_7_Pin ) )
        {
            return InOutSwitchOut;
        }
        return InOutSwitchUnknow;
    }
    else if( type == 1 )
    {
        if( HAL_GPIO_ReadPin( IN_6_GPIO_Port, IN_6_Pin ) )
        {
            return InOutSwitchIn;
        }
        if( HAL_GPIO_ReadPin( IN_7_GPIO_Port, IN_7_Pin ) )
        {
            return InOutSwitchOut;
        }
        return InOutSwitchUnknow;
    }
    else
    {
        static int isDebugOut = 0;
        if( !isDebugOut )
        {
            isDebugOut = 1;
            debugOut(0, "[\t%d] Unknow Switch Sensor Type!!!!!!!\r\n", osKernelSysTick() );
        }
    }
}

OnOffDef getEmergencyKey()
{
    static struct {
        int OnOff;
        int timeOut;
    } KeyStatus;
    if( KeyStatus.OnOff == on )
    {
        if( !HAL_GPIO_ReadPin( IN_Emergency_Port, IN_Emergency_Pin ) )
        {
            KeyStatus.OnOff = on;
            KeyStatus.timeOut = 10;
        }
        else
        {
            if( KeyStatus.timeOut > 0 )
                KeyStatus.timeOut --;
            else
            {
                KeyStatus.OnOff = off;
                KeyStatus.timeOut = 10;
            }
        }
    }
    else
    {
        if( !HAL_GPIO_ReadPin( IN_Emergency_Port, IN_Emergency_Pin ) )
        {
            if( KeyStatus.timeOut > 0 )
                KeyStatus.timeOut --;
            else
            {
                KeyStatus.OnOff = on;
                KeyStatus.timeOut = 10;
            }
        }
        else
        {
            KeyStatus.OnOff = off;
            KeyStatus.timeOut = 10;
        }
    }
    return KeyStatus.OnOff;
}
static OnOffDef getPowerKey()
{
    static int firstBootUp = 1;
    if( firstBootUp )
    {
        if( !HAL_GPIO_ReadPin( IN_18_GPIO_Port, IN_18_Pin ) )
        {
            firstBootUp = 0;
        }
        else
            return off;
    }
    if( HAL_GPIO_ReadPin( IN_18_GPIO_Port, IN_18_Pin ) )
    {
        return on;
    }
    else
    {
        return off;
    }
}
OnOffDef powerKeyWork( uint32_t clock)
{
    static struct
    {
        uint32_t clock;
        int flag;
        int Poweroff;
        int savePowerStatus;
    } PowerCount;
    PowerCount.Poweroff = 0;
    PowerCount.savePowerStatus = 0;
    PowerCount.flag = 0;
    if (getPowerKey() == on)
    {
        if (PowerCount.flag)
        {
            if (clock - PowerCount.clock > 500)
            {
                PowerCount.Poweroff = 1;
            }
        }
        else
        {
            PowerCount.flag = 1;
            PowerCount.clock = clock;
        }
    }
    else
    {
        if (PowerCount.Poweroff)
        {
            //setPowerKey(off);
            return on;
        }
        PowerCount.flag = 0;
    }
    return off;
}
/*
void setPowerKey( OnOffDef status )
{
    if( status == on )
    {
        HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_RESET );
        HAL_GPIO_WritePin( OUT_2_GPIO_Port, OUT_2_Pin, GPIO_PIN_SET );
    }
    else
    {
        HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_SET );
        HAL_GPIO_WritePin( OUT_2_GPIO_Port, OUT_2_Pin, GPIO_PIN_RESET );
    }
}
*/
