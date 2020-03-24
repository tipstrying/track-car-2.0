/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

#include "string.h"
#include "motor.h"
#include "app.h"
#include "stdio.h"
#include "w5500.h"
#include "socket.h"
#include "hardware.h"
#include "math.h"
#include "battery.h"
#include "rtc.h"

void startCLITask()
{
    void CLITask( void const * parment );
    void protocolRun( void const *para );
    osThreadDef( CLITask, CLITask, osPriorityAboveNormal, 0, 1024 );
    osThreadCreate( osThread( CLITask ), NULL);
    osThreadDef( protocolRun, protocolRun, osPriorityAboveNormal, 0, 1024 );
    osThreadCreate( osThread( protocolRun ), NULL);
}
static void prvSaveTraceFile( void );

static BaseType_t prvRunTimeStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvStartStopTraceCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xRunTimeStats =
{
    "run-time-stats", /* The command string to type. */
    "\r\nrun-time-stats: Displays a table showing how much processing time each FreeRTOS task has used",
    prvRunTimeStatsCommand, /* The function to run. */
    0 /* No parameters are expected. */
};

static BaseType_t prvTaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xTaskStats =
{
    "task-stats", /* The command string to type. */
    "\r\ntask-stats: Displays a table showing the state of each FreeRTOS task",
    prvTaskStatsCommand, /* The function to run. */
    0 /* No parameters are expected. */
};

static BaseType_t GetMilageCmd(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterGetMilages =
{
    "milages",
    "\r\nmilages: Displays milages after this boot up",
    GetMilageCmd,
    0
};
static BaseType_t GetPositionCmd( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterGetPosioion =
{
    "position",
    "\r\nposition: Display position now",
    GetPositionCmd,
    0
};
static BaseType_t GetTcpConnectingCmd( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xparameterGetTcpConnect =
{
    "tcp-status",
    "\r\ntcp-status: Display Main Socket status now",
    GetTcpConnectingCmd,
    0
};
static BaseType_t GetBatteryCmd( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterGetBattery =
{
    "battery",
    "\r\nbattery: Display battery info",
    GetBatteryCmd,
    0
};

static BaseType_t GetInOutStatus( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterGetInOutStatus =
{
    "inout-stats",
    "\r\ninout-stats: Display IN OUT port status",
    GetInOutStatus,
    0
};
static BaseType_t GetRealTimeSpeed( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterGetRealSpeed =
{
    "speed",
    "\r\nspeed: Display real-time speed",
    GetRealTimeSpeed,
    0
};
static BaseType_t Exit( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterExit =
{
    "exit",
    "\r\nexit: Logout\r\n",
    Exit,
    0
};
static BaseType_t LogCtl( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterLogCtl =
{
    "logctl",
    "\r\nlogctl: Enable/Disable log output\r\n\tUsage:logctl < enable | disable > < logType >\r\n\tlogType:RealTimeSpeed\tRealTimeEcode\tXYExti\tNavigation\tOperation\tStartUp\tCanRawData\tSetMaxSpeed\
    \tCharge\tSwitch\tBattery\tMotionDebug\r\n\tUsage:logctl show all",
    LogCtl,
    -1
};
static BaseType_t MotionCtl( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterMotionCtl =
{
    "motionctl",
    "\r\nmotionctl: Set Motion Parameter online\r\n\
    Usage:\r\n\
    \tmotionctl [ <set k=? j=? a_max=? v_max=?> ]\r\n\
    \tmotionctl [ <set manual-mode=<enable|disable>> ]\r\n\
    \tmotionctl [ <move-free x=? y=?>]\r\n\
    \tmotionctl [ <move-lattice x+?|x-?|y+?|y-?> ]\r\n\
    \tmotionctl [ <switch to x> | <switch to y> ]\r\n\
    \tmotionctl [ <init x=? y=?> ]\r\n"\
    ,
    MotionCtl,
    -1,
};
static BaseType_t reboot( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterReboot =
{
    "reboot",
    "\r\nreboot: reboot car mcu\r\n\treboot rec: reboot to bootloader\r\n",
    reboot,
    -1,
};
static BaseType_t getRamFree( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterFree =
{
    "free",
    "\r\nfree: RAM use status",
    getRamFree,
    0,
};

static BaseType_t setSwitchCli( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterSetSwitch =
{
    "switchTest",
    "",
    setSwitchCli,
    -1
};
static BaseType_t setIpMac( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t xParameterSetIpMac =
{
    "ip",
    "\r\nip [show] [set ip:192.168.%d.%d] [set mac:%d.%d]",
    setIpMac,
    -1
};
/*-----------------------------------------------------------*/

void vRegisterCLICommands( void )
{
    /* Register all the command line commands defined immediately above. */
    FreeRTOS_CLIRegisterCommand( &xTaskStats );
    FreeRTOS_CLIRegisterCommand( &xRunTimeStats );
    FreeRTOS_CLIRegisterCommand( &xParameterGetMilages );
    FreeRTOS_CLIRegisterCommand( &xParameterGetBattery );
    FreeRTOS_CLIRegisterCommand( &xParameterGetPosioion );
    FreeRTOS_CLIRegisterCommand( &xparameterGetTcpConnect );
    FreeRTOS_CLIRegisterCommand( &xParameterGetRealSpeed );
    FreeRTOS_CLIRegisterCommand( &xParameterExit );
    FreeRTOS_CLIRegisterCommand( &xParameterGetInOutStatus );
    FreeRTOS_CLIRegisterCommand( &xParameterLogCtl );
    FreeRTOS_CLIRegisterCommand( &xParameterMotionCtl );
    FreeRTOS_CLIRegisterCommand( &xParameterFree );
    FreeRTOS_CLIRegisterCommand( &xParameterReboot );
    FreeRTOS_CLIRegisterCommand( &xParameterSetIpMac );
}
static BaseType_t setIpMac( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char * arg1, *arg2;
    BaseType_t argLen;
    arg1 = FreeRTOS_CLIGetParameter( pcCommandString, 1, &argLen );
    arg2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &argLen );
    union
    {
        uint8_t Hex[4];
        uint32_t Data;
    } u32ToHex[2];
    uint32_t ip, mac;
    ip = HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR1 );
    mac = HAL_RTCEx_BKUPRead( &hrtc, RTC_BKP_DR2 );
    u32ToHex[0].Data = ip;
    u32ToHex[1].Data = mac;
    uint32_t tmp[2];

    if( strstr( arg1, "show" ) != 0 )
    {
        sprintf( pcWriteBuffer, "IP:192.168.%d.%d\tMAC:0x00 0x08 0xDC 0x11 0x%X 0x%X\r\n", u32ToHex[0].Hex[0], u32ToHex[0].Hex[1], u32ToHex[1].Hex[0], u32ToHex[1].Hex[1] );
        return pdFALSE;
    }
    else
    {
        if( sscanf( arg1, "set ip:192.168.%d.%d", tmp, tmp + 1 ) == 2 )
        {
            u32ToHex[0].Hex[0] = tmp[0];
            u32ToHex[0].Hex[1] = tmp[1];
            u32ToHex[0].Hex[2] = u32ToHex[0].Hex[0] + u32ToHex[0].Hex[1];
            HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR1, u32ToHex[0].Data );
            sprintf( pcWriteBuffer, "\r\nip:192.168.%d.%d\r\n", u32ToHex[0].Hex[0], u32ToHex[0].Hex[1] );
        }
        else if( sscanf( arg1, "set mac:%d.%d", tmp, tmp + 1 ) == 2 )
        {
            u32ToHex[0].Hex[0] = tmp[0];
            u32ToHex[0].Hex[1] = tmp[1];
            u32ToHex[0].Hex[2] = u32ToHex[0].Hex[0] + u32ToHex[0].Hex[1];
            HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR2, u32ToHex[0].Data );
            sprintf( pcWriteBuffer, "\r\nmac:%d.%d\r\n", u32ToHex[0].Hex[0], u32ToHex[0].Hex[1] );
        }
        else
        {
            sprintf( pcWriteBuffer, "unknow args\r\n" );
        }
    }
    return pdFALSE;
}
static BaseType_t setSwitchCli( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{

}
static BaseType_t getRamFree( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    BaseType_t freeSize = xPortGetFreeHeapSize() / 1024;
    BaseType_t totalSize = configTOTAL_HEAP_SIZE / 1024;
    BaseType_t freeMin = xPortGetMinimumEverFreeHeapSize() / 1024;
    BaseType_t canQueFree = -1;
    BaseType_t navigationQueFree = -1;
    BaseType_t canQueTotal = -1;
    BaseType_t navQueTotal = -1;
    extern QueueHandle_t can1Queue;
    if( NavigationOperationQue )
    {
        navigationQueFree = uxQueueSpacesAvailable( NavigationOperationQue );
        navQueTotal = navigationQueFree + uxQueueMessagesWaiting( NavigationOperationQue );
    }
    if( can1Queue )
    {
        canQueFree = uxQueueSpacesAvailable( can1Queue );
        canQueTotal = canQueFree + uxQueueMessagesWaiting( can1Queue );
    }
    sprintf( pcWriteBuffer, "Type\tTotal\tFree\tFreeMin\r\n****************************************************\r\n" );
    sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "RAM:\t%ld kb\t%ld kb\t%ld kb\r\n", totalSize, freeSize, freeMin );
    sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "NavQue:\t%ld\t%ld\t%ld\r\n", navQueTotal, navigationQueFree, navigationQueFree );
    sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "CanQue:\t%ld\t%ld\t%ld\r\n", canQueTotal, canQueFree, canQueFree );
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t reboot( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter1;
    BaseType_t parameterLen1;
    union {
        char Hex[4];
        uint32_t Data;
    } u32ToHex;

    pcParameter1 = FreeRTOS_CLIGetParameter( pcCommandString, 1, &parameterLen1 );
    if( pcParameter1 )
    {
        if( strcmp( pcParameter1, "rec" ) == 0 )
        {
            u32ToHex.Hex[0] = 'r';
            u32ToHex.Hex[1] = 'e';
            u32ToHex.Hex[2] = 'c';
            u32ToHex.Hex[3] = 'M';
            HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR0, u32ToHex.Data );
        }
        else
        {
            HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR0, 0 );
        }
    }
    HAL_NVIC_SystemReset();
    return pdFALSE;
}
static BaseType_t GetMilageCmd(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    double milages = GetMilage();
    sprintf( pcWriteBuffer, "Milages:%lf mm\r\n",milages );
    return pdFALSE;
}
static BaseType_t GetPositionCmd( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    return pdFALSE;
}
static BaseType_t GetTcpConnectingCmd( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    if( getSn_SR( 0 ) == SOCK_ESTABLISHED )
    {
        uint8_t ipBuff[4];
        uint16_t ipPort =  getSn_DPORT(0);
        getSn_DIPR(0, ipBuff );
        sprintf( pcWriteBuffer, "Main Socket is connecting,IP: %u.%u.%u.%u:%d\r\n\r\n", ipBuff[0], ipBuff[1], ipBuff[2], ipBuff[3], ipPort );
    }
    else
    {
        sprintf(pcWriteBuffer,  "Main Socket is not connect\r\n\r\n" );
    }
    return pdFALSE;
}
static BaseType_t GetBatteryCmd( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    int usabeBattery = 0;
    sprintf( pcWriteBuffer, "Voltage :%d mV\t", Battery.Voltage );
    sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "[" );
    usabeBattery = (int) ( ( ( Battery.Voltage - 40000 ) / 200 ) );
    for( int i = 0; i < usabeBattery / 10; i++ )
    {
        sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "|" );
    }
    for( int i = 0; i < 10 - usabeBattery / 10; i++ )
    {
        sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), " " );
    }
    sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "} %d%%\r\n", usabeBattery );
    return pdFALSE;
}

static BaseType_t GetInOutStatus( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    GPIO_TypeDef * InPort[] = { IN_1_GPIO_Port, IN_2_GPIO_Port, IN_3_GPIO_Port, IN_4_GPIO_Port, IN_5_GPIO_Port, IN_6_GPIO_Port, IN_7_GPIO_Port, IN_8_GPIO_Port, IN_9_GPIO_Port, IN_10_GPIO_Port, IN_11_GPIO_Port, IN_12_GPIO_Port, IN_13_GPIO_Port, IN_14_GPIO_Port, IN_15_GPIO_Port, IN_16_GPIO_Port, IN_17_GPIO_Port, IN_18_GPIO_Port, IN_19_GPIO_Port, IN_20_GPIO_Port };
    uint16_t       InPin[] = { IN_1_Pin, IN_2_Pin, IN_3_Pin, IN_4_Pin, IN_5_Pin, IN_6_Pin, IN_7_Pin, IN_8_Pin, IN_9_Pin, IN_10_Pin, IN_11_Pin, IN_12_Pin, IN_13_Pin, IN_14_Pin, IN_15_Pin, IN_16_Pin, IN_17_Pin, IN_18_Pin, IN_19_Pin, IN_20_Pin };
    GPIO_TypeDef * OutPort[] = { OUT_1_GPIO_Port, OUT_2_GPIO_Port, OUT_3_GPIO_Port, OUT_4_GPIO_Port, OUT_5_GPIO_Port, OUT_6_GPIO_Port, OUT_7_GPIO_Port, OUT_8_GPIO_Port, OUT_9_GPIO_Port, OUT_10_GPIO_Port };
    uint16_t       OutPin[] = { OUT_1_Pin, OUT_2_Pin, OUT_3_Pin, OUT_4_Pin, OUT_5_Pin, OUT_6_Pin, OUT_7_Pin, OUT_8_Pin, OUT_9_Pin, OUT_10_Pin };

    //static int finish = 0;
    sprintf( pcWriteBuffer, "IO Status\r\n" );
    sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "InPut:\r\n" );
    for( int i = 0; i < ( sizeof( InPort) / sizeof( InPort[0] ) ); i++ )
    {
        sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "IN %d -> %d\t", i + 1, HAL_GPIO_ReadPin( InPort[i], InPin[i] ) );
    }
    sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\r\nOutPut:\r\n" );
    for( int i = 0; i < ( sizeof( OutPort) / sizeof( OutPort[0] ) ); i++ )
    {
        sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "Out %d -> %d\t", i + 1, HAL_GPIO_ReadPin( OutPort[i], OutPin[i] ) );
    }
    sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\r\n" );
    return pdFALSE;
}

static BaseType_t GetRealTimeSpeed( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    float speed;
    GetSpeed( &speed );
    sprintf( pcWriteBuffer, "Speed -> %0.2f mm/s\r\n", speed );
    return pdFALSE;
}
static BaseType_t Exit( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    sprintf( pcWriteBuffer, "Please use 'CTRL+]' switch to telnet cmd mode and type 'q' or 'quite' to exit\r\n" );
    return pdFALSE;
}
static BaseType_t LogCtl( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter1, *pcParameter2, *pcParameter3;
    BaseType_t parameterLen1, parameterLen2, parameterLen3;
    char parameterBuff[20];
    pcParameter1 = FreeRTOS_CLIGetParameter( pcCommandString, 1, &parameterLen1 );
    pcParameter2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &parameterLen2 );
    if( parameterLen1 > 20 || parameterLen2 > 20)
    {
        sprintf( pcWriteBuffer, "Parameter Too Long\r\n" );
        return pdFALSE;
    }
    if( pcParameter1 && pcParameter2 )
    {
        memcpy( parameterBuff, pcParameter1, parameterLen1 );
        parameterBuff[parameterLen1] = 0;
        if( strcmp( parameterBuff, "enable" ) == 0 )
        {
            memcpy( parameterBuff, pcParameter2, parameterLen2 );
            parameterBuff[parameterLen2] = 0;
            if( strcmp( parameterBuff, "RealTimeEcode" ) == 0 )
            {
                DebugCtrl.enableRealTimeEcode = 1;
                sprintf( pcWriteBuffer, "Set RealTimeEcode on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "RealTimeSpeed" ) == 0 )
            {
                DebugCtrl.enableRealTimeSpeed = 1;
                sprintf( pcWriteBuffer, "Set RealTimeSpeed on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "XYExti" ) == 0 )
            {
                DebugCtrl.enableXYExti = 1;
                sprintf( pcWriteBuffer, "Set XYExti on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "Navigation" ) == 0 )
            {
                DebugCtrl.enableNavigation = 1;
                sprintf( pcWriteBuffer, "Set Navigation on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "Operation" ) == 0 )
            {
                DebugCtrl.enableOperation = 1;
                sprintf( pcWriteBuffer, "Set Operation on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "StartUp" ) == 0 )
            {
                DebugCtrl.enableStartUp = 1;
                sprintf( pcWriteBuffer, "Set StartUp on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "CanRawData" ) == 0 )
            {
                DebugCtrl.enableCanRawData = 1;
                sprintf( pcWriteBuffer, "Set CanRawData on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "SetMaxSpeed" ) == 0 )
            {
                DebugCtrl.enableSetMaxSpeed = 1;
                sprintf( pcWriteBuffer, "Set SetMaxSpeed on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "Charge" ) == 0 )
            {
                DebugCtrl.enableCharge = 1;
                sprintf( pcWriteBuffer, "Set Charge on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "Switch" ) == 0 )
            {
                DebugCtrl.enableSwitch = 1;
                sprintf( pcWriteBuffer, "Set Switch on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "Battery" ) == 0 )
            {
                DebugCtrl.enableBattery = 1;
                sprintf( pcWriteBuffer, "Set Battery on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "MotionDebug" ) == 0 )
            {
                DebugCtrl.enableMotionDebug = 1;
                sprintf( pcWriteBuffer, "Set MotionDebug on\r\n" );
                return pdFALSE;
            }
            if( strcmp( parameterBuff, "RealTimeSpeed" ) == 0 )
            {
                DebugCtrl.enableRealTimeSpeed = 1;
                sprintf( pcWriteBuffer, "Set MotionDebug on\r\n" );
            }
            return pdFALSE;
        }
        else if( strcmp( parameterBuff, "disable" ) == 0 )
        {
            memcpy( parameterBuff, pcParameter2, parameterLen2 );
            parameterBuff[parameterLen2] = 0;
            if( strcmp( parameterBuff, "RealTimeEcode" ) == 0 )
            {
                DebugCtrl.enableRealTimeEcode = 0;
            }
            if( strcmp( parameterBuff, "RealTimeSpeed" ) == 0 )
            {
                DebugCtrl.enableRealTimeSpeed = 0;
            }
            if( strcmp( parameterBuff, "XYExti" ) == 0 )
            {
                DebugCtrl.enableXYExti = 0;
            }
            if( strcmp( parameterBuff, "Navigation" ) == 0 )
            {
                DebugCtrl.enableNavigation = 0;
            }
            if( strcmp( parameterBuff, "Operation" ) == 0 )
            {
                DebugCtrl.enableOperation = 0;
            }
            if( strcmp( parameterBuff, "StartUp" ) == 0 )
            {
                DebugCtrl.enableStartUp = 0;
            }
            if( strcmp( parameterBuff, "CanRawData" ) == 0 )
            {
                DebugCtrl.enableCanRawData = 0;
            }
            if( strcmp( parameterBuff, "SetMaxSpeed" ) == 0 )
            {
                DebugCtrl.enableSetMaxSpeed = 0;
            }
            if( strcmp( parameterBuff, "Charge" ) == 0 )
            {
                DebugCtrl.enableCharge = 0;
            }
            if( strcmp( parameterBuff, "Switch" ) == 0 )
            {
                DebugCtrl.enableSwitch = 0;
            }
            if( strcmp( parameterBuff, "Battery" ) == 0 )
            {
                DebugCtrl.enableBattery = 0;
            }
            if( strcmp( parameterBuff, "MotionDebug" ) == 0 )
            {
                DebugCtrl.enableMotionDebug = 0;
            }
            if( strcmp( parameterBuff, "RealTimeSpeed" ) == 0 )
            {
                DebugCtrl.enableRealTimeSpeed = 0;
            }
            if( strcmp( parameterBuff, "all" ) == 0 )
            {
                DebugCtrl.enableBattery = 0;
                DebugCtrl.enableCanRawData = 0;
                DebugCtrl.enableCharge = 0;
                DebugCtrl.enableMotion = 0;
                DebugCtrl.enableNavigation = 0;
                DebugCtrl.enableOperation = 0;
                DebugCtrl.enableRealTimeEcode = 0;
                DebugCtrl.enableSecondLoaclizationTimeOut = 0;
                DebugCtrl.enableSetMaxSpeed = 0;
                DebugCtrl.enableStartUp = 0;
                DebugCtrl.enableSwitch = 0;
                DebugCtrl.enableXYExti = 0;
                DebugCtrl.enableMotionDebug = 0;
                DebugCtrl.enableRealTimeSpeed = 0;
            }
            sprintf( pcWriteBuffer, "" );
            return pdFALSE;
        }
        else if( strcmp( parameterBuff, "show" ) == 0 )
        {
            memcpy( parameterBuff, pcParameter2, parameterLen2 );
            parameterBuff[parameterLen2] = 0;
            if( strcmp( parameterBuff, "all" ) == 0 )
            {
                sprintf( pcWriteBuffer, "Log status:" );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tBattery: %s ", (DebugCtrl.enableBattery == 1 ? "on":"off") );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tCanRawData: %s ", (DebugCtrl.enableCanRawData == 1 ? "on":"off") );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tCharge: %s ", (DebugCtrl.enableCharge == 1 ? "on":"off") );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tNavigation: %s ", (DebugCtrl.enableNavigation == 1 ? "on":"off") );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tOperation: %s ", (DebugCtrl.enableOperation == 1 ? "on":"off") );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tReal-Time EnCode: %s ", (DebugCtrl.enableRealTimeEcode == 1 ? "on":"off") );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tReal-Time Speed: %s ", (DebugCtrl.enableRealTimeSpeed == 1 ? "on":"off") );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tSet MaxSpeed: %s ", (DebugCtrl.enableSetMaxSpeed == 1 ? "on":"off") );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tStartUp: %s ", (DebugCtrl.enableStartUp == 1 ? "on":"off") );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tRun-Photoelectric: %s ", (DebugCtrl.enableXYExti == 1 ? "on":"off") );
                sprintf( pcWriteBuffer + strlen( pcWriteBuffer ), "\tSwitch X/Y: %s ", (DebugCtrl.enableSwitch == 1 ? "on":"off") );
            }
            return pdFALSE;
        }
        else
        {
            sprintf( pcWriteBuffer, "first parment must be \"enable\" or \"disable\"\r\n" );
            return pdFALSE;
        }
    }
    else
    {
        sprintf( pcWriteBuffer, "usage error\r\n" );
        return pdFALSE;
    }
}

static BaseType_t MotionCtl( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char * parment[5] = {0,0,0,0,0 };
    BaseType_t parameterLen[5] = {0, 0, 0, 0, 0 };
    float ts_k, ts_j, ts_a, ts_v;
    float x,y;
    float x1,y1;
    int xCount, yCount;
    parment[0] = FreeRTOS_CLIGetParameter( pcCommandString, 1, &( parameterLen[0] ) );
    if( parment[0] )
    {
        if( sscanf( parment[0], "set k=%f j=%f a=%f v=%f", &ts_k, &ts_j, &ts_a, &ts_v ) == 4 )
        {
            sprintf( pcWriteBuffer, "set k=%f j=%f a=%f v=%f\r\n", ts_k, ts_j, ts_a, ts_v );
        }
        else
        {

        }
    }
    else
    {
        sprintf( pcWriteBuffer, "Motion parameter: k=%f j=%f a=%f v=%f\r\n", ts_k, ts_j, ts_a, ts_v );
    }
    return pdFALSE;
}
static BaseType_t prvTaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *const pcHeader = "Task          State  Priority  Stack	#\r\n************************************************\r\n";

    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Generate a table of task stats. */
    strcpy( pcWriteBuffer, pcHeader );
    vTaskList( pcWriteBuffer + strlen( pcHeader ) );

    /* There is no more data to return after this single string, so return
    pdFALSE. */
    return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvRunTimeStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char * const pcHeader = "Task            Abs Time      % Time\r\n****************************************\r\n";

    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Generate a table of task stats. */
    strcpy( pcWriteBuffer, pcHeader );
    vTaskGetRunTimeStats( pcWriteBuffer + strlen( pcHeader ) );

    /* There is no more data to return after this single string, so return
    pdFALSE. */
    return pdFALSE;
}
/*-----------------------------------------------------------*/
#if 0
static BaseType_t prvStartStopTraceCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t lParameterStringLength;

    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Obtain the parameter string. */
    pcParameter = FreeRTOS_CLIGetParameter
                  (
                      pcCommandString,		/* The command string itself. */
                      1,						/* Return the first parameter. */
                      &lParameterStringLength	/* Store the parameter string length. */
                  );

    /* Sanity check something was returned. */
    configASSERT( pcParameter );

    /* There are only two valid parameter values. */
    if( strncmp( pcParameter, "start", strlen( "start" ) ) == 0 )
    {
        /* Start or restart the trace. */
        vTraceStop();
        vTraceClear();
        uiTraceStart();

        sprintf( pcWriteBuffer, "Trace recording (re)started.\r\n" );
    }
    else if( strncmp( pcParameter, "stop", strlen( "stop" ) ) == 0 )
    {
        /* End the trace, if one is running. */
        vTraceStop();
        sprintf( pcWriteBuffer, "Stopping trace recording and dumping log to disk.\r\n" );
        prvSaveTraceFile();
    }
    else
    {
        sprintf( pcWriteBuffer, "Valid parameters are 'start' and 'stop'.\r\n" );
    }

    /* There is no more data to return after this single string, so return
    pdFALSE. */
    return pdFALSE;
}
#endif
/*-----------------------------------------------------------*/


