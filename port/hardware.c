#include "hardware.h"
#include "main.h"
#include "tim.h"

#define OUT_UpDownC_Port    OUT_9_GPIO_Port
#define OUT_UpDownC_Pin     OUT_9_Pin

#define OUT_UpDownB_Port    OUT_10_GPIO_Port
#define OUT_UpDownB_Pin     OUT_10_Pin

#define OUT_Motor_Enable_Port   OUT_6_GPIO_Port
#define OUT_Motor_Enable_Pin    OUT_6_Pin

int init ()
{
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
//int beltCtrl( int isRun, BeltDirectionDef dir, int speed )
//{
//    uint32_t pscData = 400 / speed;

//    if( isRun )
//    {
//        htim3.Instance->PSC = pscData ;
//        if( dir == BeltFront )
//        {
//            HAL_GPIO_WritePin( OUT_Belt_For_Port, OUT_Belt_For_Pin, GPIO_PIN_SET );
//            HAL_GPIO_WritePin( OUT_Belt_Rev_Port, OUT_Belt_Rev_Pin, GPIO_PIN_RESET );
//        }
//        else
//        {
//            HAL_GPIO_WritePin( OUT_Belt_For_Port, OUT_Belt_For_Pin, GPIO_PIN_RESET );
//            HAL_GPIO_WritePin( OUT_Belt_Rev_Port, OUT_Belt_Rev_Pin, GPIO_PIN_SET );
//        }
//    }
//    else
//    {
//        HAL_GPIO_WritePin( OUT_Belt_For_Port, OUT_Belt_For_Pin, GPIO_PIN_RESET );
//        HAL_GPIO_WritePin( OUT_Belt_Rev_Port, OUT_Belt_Rev_Pin, GPIO_PIN_RESET );
//    }

//    return 0;
//}
//OnOffDef getThingSensor( int ID )
//{
//    switch( ID )
//    {
//    case 1:
//        if( HAL_GPIO_ReadPin( IN_T1_Port, IN_T1_Pin ) )
//        {
//            return on;
//        }
//        else
//        {
//            return off;
//        }
//        break;
//    case 2:
//        if( HAL_GPIO_ReadPin( IN_T2_Port, IN_T2_Pin ) )
//        {
//            return on;
//        }
//        else
//            return off;
//        break;
//    case 3:
//        if( HAL_GPIO_ReadPin( IN_T3_Port, IN_T3_Pin ) )
//        {
//            return on;
//        }
//        else
//        {
//            return off;
//        }
//        break;
//    default:
//        return off;
//    }
//    return off;
//}
void setExti( int enable )
{
    if( enable )
    {
        /* EXTI interrupt init*/
        HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI3_IRQn);

        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
    else
    {
        HAL_NVIC_DisableIRQ( EXTI3_IRQn );
        HAL_NVIC_DisableIRQ( EXTI9_5_IRQn );
        HAL_NVIC_DisableIRQ( EXTI15_10_IRQn );
    }
}

OnOffDef setChargeKey( OnOffDef status )
{
    if( status == on )
    {
        HAL_GPIO_WritePin( OUT_Charge_C_Port, OUT_Charge_C_Pin, GPIO_PIN_SET );
    }
    else
    {
        HAL_GPIO_WritePin( OUT_Charge_C_Port, OUT_Charge_C_Pin, GPIO_PIN_RESET );
    }
}
/*
OnOffDef setChargeSensor( OnOffDef status )
{
    if( status == on )
    {
        HAL_GPIO_WritePin( OUT_Charge_L_Port, OUT_Charge_L_Pin, GPIO_PIN_SET );
    }
    else
    {
        HAL_GPIO_WritePin( OUT_Charge_L_Port, OUT_Charge_L_Pin, GPIO_PIN_RESET );
    }
}
*/
OnOffDef getChargeKey()
{
    if( HAL_GPIO_ReadPin( OUT_Charge_C_Port, OUT_Charge_C_Pin ) )
        return on;
    else
        return off;
}
char isTouchPie()
{
    /* return 0; */
    static struct {
        int Touch;
        int timeOut;
    } PieTouchStatus;

    if( HAL_GPIO_ReadPin( OUT_Charge_C_Port, OUT_Charge_C_Pin ))
    {
        PieTouchStatus.Touch = 1;
        PieTouchStatus.timeOut = 10;
        return 1;
    }
    if( PieTouchStatus.Touch )
    {
        if( HAL_GPIO_ReadPin( IN_19_GPIO_Port, IN_19_Pin ) )
        {
            PieTouchStatus.timeOut = 10;
            PieTouchStatus.Touch = 1;
        }
        else
        {
            if( PieTouchStatus.timeOut > 0 )
                PieTouchStatus.timeOut --;
            else
                PieTouchStatus.Touch = 0;
        }
    }
    else
    {
        if( HAL_GPIO_ReadPin( IN_19_GPIO_Port, IN_19_Pin ) )
        {
            if( PieTouchStatus.timeOut > 0 )
            {
                PieTouchStatus.timeOut --;
            }
            else
            {
                PieTouchStatus.Touch = 1;
                PieTouchStatus.timeOut = 10;
            }
        }
        else
        {
            PieTouchStatus.timeOut = 10;
            PieTouchStatus.Touch = 0;
        }
    }
    return PieTouchStatus.Touch;
}
