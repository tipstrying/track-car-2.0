#include "hardware.h"
#include "main.h"
#include "tim.h"
#include "rtc.h"

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

int init ()
{
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
int isPackOnCar()
{
    if( HAL_GPIO_ReadPin( IN_T1_Port, IN_T1_Pin ) || HAL_GPIO_ReadPin( IN_T2_Port, IN_T2_Pin ) || HAL_GPIO_ReadPin( IN_T3_Port, IN_T3_Pin ) )
        return 1;
    else
        return 0;
}

int beltCtrl( int isRun, BeltDirectionDef dir, int speed )
{
    if( isRun )
    {
        if( dir == BeltRev )
        {
            HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_SET );
            HAL_GPIO_WritePin( OUT_6_GPIO_Port, OUT_6_Pin, GPIO_PIN_SET );
        }
        else
        {
            HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( OUT_6_GPIO_Port, OUT_6_Pin, GPIO_PIN_SET );

        }
    }
    else
    {
        HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_RESET );
        HAL_GPIO_WritePin( OUT_6_GPIO_Port, OUT_6_Pin, GPIO_PIN_RESET );
    }
}
int writePosToBKP( float position )
{
    union {
        float fData;
        uint32_t uData;

    } u32Tof32;
    union {
        uint32_t Data;
        uint8_t Hex[4];
    } u32ToHex;

    u32Tof32.fData = position;
    u32ToHex.Data = u32Tof32.uData;
    uint32_t sum = 0;
    for( int i = 0; i < 4; i++ )
    {
        sum += u32ToHex.Hex[i];
    }
    HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR10, u32Tof32.uData );
    HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR11, sum );

}
int readPosFromBKP( float *position )
{
    union {
        float fData;
        uint32_t uData;

    } u32Tof32;
    union {
        uint32_t Data;
        uint8_t Hex[4];
    } u32ToHex;

    u32Tof32.uData = HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR10 );
    u32ToHex.Data = u32Tof32.uData;
    uint32_t sum = 0;
    for( int i = 0; i < 4; i++ )
    {
        sum += u32ToHex.Hex[i];
    }
    if( sum == HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR11 ) )
    {
        if( position )
            *position = u32Tof32.fData;
        return 1;
    }
    else
        return 0;
}

InOutSwitch getSwitchStatus()
{
    if( !HAL_GPIO_ReadPin( IN_6_GPIO_Port, IN_6_Pin ) && !HAL_GPIO_ReadPin( IN_7_GPIO_Port, IN_7_Pin ) )
    {
        return InOutSwitchIn;
    }
    if( !HAL_GPIO_ReadPin( IN_7_GPIO_Port, IN_7_Pin ) )
    {
        return InOutSwitchOut;
    }
    return InOutSwitchUnknow;
}

InOutSwitch setSwitch( InOutSwitch target )
{
    switch( target )
    {
    case InOutSwitchIn:
        if( getSwitchStatus() == target )
        {
            HAL_GPIO_WritePin( OUT_3_GPIO_Port, OUT_3_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( OUT_4_GPIO_Port, OUT_4_Pin, GPIO_PIN_RESET );
        }
        else
        {
            HAL_GPIO_WritePin( OUT_3_GPIO_Port, OUT_3_Pin, GPIO_PIN_SET );
            HAL_GPIO_WritePin( OUT_4_GPIO_Port, OUT_4_Pin, GPIO_PIN_SET );
        }
        break;
    case InOutSwitchOut:
        if( getSwitchStatus() == target )
        {
            HAL_GPIO_WritePin( OUT_3_GPIO_Port, OUT_3_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( OUT_4_GPIO_Port, OUT_4_Pin, GPIO_PIN_RESET );
        }
        else
        {
            HAL_GPIO_WritePin( OUT_3_GPIO_Port, OUT_3_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( OUT_4_GPIO_Port, OUT_4_Pin, GPIO_PIN_SET );
        }
        break;
    case InOutSwitchUnknow:
    default:
        HAL_GPIO_WritePin( OUT_3_GPIO_Port, OUT_3_Pin, GPIO_PIN_RESET );
        HAL_GPIO_WritePin( OUT_4_GPIO_Port, OUT_4_Pin, GPIO_PIN_RESET );
        break;
    }
    return InOutSwitchUnknow;
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
