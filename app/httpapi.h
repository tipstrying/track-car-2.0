#ifndef __HTTPAPI_H
#define __HTTPAPI_H
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

void getTaskStatus(uint8_t *buff);
void getTaskStack(uint8_t *buff);
void getRamStatus(uint8_t *buff);
int GetPositionHttpApi( float *pos );
int SetPositionHttpApi( float pos );
int GetSpeedHttpApi( float *sp );
int SetOpHttpApi( float position, int cmd, float data );
void beltOpHttpApi( int cmd );
int InOutStatus();
int GetBatteryVoltageHttpApi( float *Voltage );
int setMotorDisable(int status );
int GetMaxSpeedHttpApi( float *sp );
int getMilagesHttpApi ( double * mils );
int SetHandSpeedModeHttoApi( int isHandMode );
int SetHandSpeedModeSpeedHttpApi( int speed );
int SetSleepModeHttpApi( int isSleep );
int cancelNavigationHttpApi();
int GetMotorCurrentHttpApi( float *current );
int clearMotoralarmHttpApi();
    
#ifdef __cplusplus
}
#endif
#endif