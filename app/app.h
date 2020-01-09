#ifndef __APP_H
#define __APP_H
#ifdef __cplusplus
extern "C" {
#endif
#include "cmsis_os.h"
#include "string.h"
#include "ff.h"
#include "fatfs.h"

/*
typedef struct {
    char buff[100];
} DebugBuffStd;
*/
typedef struct {
    char enableRealTimeEcode;
    char enableRealTimeSpeed;
    char enableXYExti;
    char enableNavigation;
    char enableOperation;
    char enableStartUp;
    char enableCanRawData;
    char enableSetMaxSpeed;
    char enableCharge;
    char enableSwitch;
    char enableBattery;
    char enableMotion;
    char enableSecondLoaclizationTimeOut;
    char enableMotionDebug;
    char runSensorCheck;
    char RFIDUpdate;
    char MotorTemperature;
    char enableMotorAlarm;
}DebugOutCtlDef;

extern DebugOutCtlDef DebugCtrl;
    
typedef struct {
    uint8_t  BatterStatus;
    uint16_t Battery;
    uint16_t BatteryFull;
    uint16_t   Current;
    uint8_t Temperature;
    uint16_t  Voltage;

} BattreyStd;

typedef struct {
    int cmd;
    struct {
        float posTo;
        float speedTo;
        int op;
    } Data;
} NavigationOperationStd;

//extern QueueHandle_t BatterQueRx;
extern QueueHandle_t NavigationOperationQue;

extern BattreyStd Battery;

void W5500Task( void const * par );
void UartTask( void const * par );
int debugOut( int isISR, char *fmt, ... );

#ifdef __cplusplus
}
#endif
#endif
