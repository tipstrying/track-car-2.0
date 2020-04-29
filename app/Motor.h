#ifndef __MOTOR_H
#define __MOTOR_H
#ifdef __cplusplus
extern "C" {
#endif
#include "app.h"
#include "motorcan.h"
#include "tim.h"

typedef struct
{
    char cmd;
    char parment;
} ChargeQueDef;
typedef struct
{
    int ID;
    float x;
    float y;
} map;
typedef struct
{
    struct {
        int ID;
        int RegSize;
        int Index;
        int SubIndex;
        int Data;
    } CanDataDef;
    struct {
        int id;
        int dlc;
        union  {
            long long data;
            uint8_t hex[8];
        } uint64ToHex;
    } CanDataShort;
    int ManualMode;
    int Type;
} CanManualDef;

void MotionTask( void const * parment );
void GetPosition( float * X );
int SetSelfPosition( float X );
int setUnSet();
int IsArriver( void );
void SetiEmergency( int S );
float GetStop_Accuracy(void);
void setManualMode( int status );
void GetMaxSpeed(float *xSpeed);

void SetMaxSpeed( float xSpeeed );
void GetSpeed( float *xSpeed);
void SetPositionMoveTo( float x );
void GetRealTimePosition( float *x );
void GetNextPiont( float *X);
double GetMilage( void );
void cancelNavigate();
char isTouchPie();
void GetMotorCurrent( short *current );

void setRunAcc( float acc );
float getRunAcc();
void ClearMotorAlarm();
uint8_t GetMotionStatus();
void GetMotorVoltage( unsigned short *Voltage );

void getTS_CURVEPAR( float *k, float *j, float *a, float *v );
void setTS_CURVEPAR( float k, float j, float a, float v );

#ifdef __cplusplus
}
#endif
#endif
