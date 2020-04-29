#ifndef __HARDWARE_H
#define __HARDWARE_H
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/*ÊäÈë¶Ë¿ÚÓ³Éä*/

#define IN_Emergency_Port   IN_1_GPIO_Port
#define IN_Emergency_Pin    IN_1_Pin

//Êä³ö¶Ë¿ÚÓ³Éä
#define OUT_Charge_C_Port   OUT_1_GPIO_Port
#define OUT_Charge_C_Pin    OUT_1_Pin

#define OUT_LED_KEY_Port    OUT_2_GPIO_Port
#define OUT_LED_KEY_Pin     OUT_2_Pin


typedef enum {
    InOutSwitchUnknow = 0,
    InOutSwitchIn = 1,
    InOutSwitchOut = 2,
} InOutSwitch;

typedef enum {
    BeltFront = 0,
    BeltRev = 1
} BeltDirectionDef;

typedef enum {
    on = 1,
    off = 0
} OnOffDef;

typedef enum {
    uart2Isr = 1,
    uart7Isr = 2,
    gpioIsr = 3,
    canIsr = 4,
    unknowIsr = 5,
    NMIIsr = 6,
    HardFault = 7,
}ISREnumDef;
    

int prvInitHardwares ();
    
OnOffDef getEmergencyKey();
OnOffDef getPowerKey();
int beltCtrl( int isRun, BeltDirectionDef dir, int speed );
void setPowerKey( OnOffDef status );
OnOffDef getThingSensor( int ID, int maxDelay );

InOutSwitch getSwitchStatus();
InOutSwitch setSwitch( InOutSwitch target );
OnOffDef powerKeyWork( uint32_t clock);
int writePosToBKP( float position, double mils );
int readPosFromBKP( float *position, double *mils );
 int getThingSensorStatus( int type);
int readSwitchTypeFromBKP(int *type);
int writeSwitchTypeFromBKP(int type);
void setITFlag( ISREnumDef isr );

#ifdef __cplusplus
}
#endif
#endif