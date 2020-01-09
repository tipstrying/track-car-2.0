#ifndef __MOTORCAN_H
#define __MOTORCAN_H
#ifdef __cplusplus
extern "C" {
#endif
#include "app.h"
#include "can.h"

typedef struct {
    CAN_TxHeaderTypeDef TxHeader;
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t Data[8];
    uint8_t IsRecive;
} can1DataStd;

typedef struct {
    can1DataStd can1data[10];
    int indexRead, indexWrite;
}Can1DataDef;
extern Can1DataDef can1Data;

//extern can1DataStd Can1Data;
//extern struct Can1DataS;
int Can1Init( void );
int SendBuffToCan1( uint8_t *buff, int ID, int dlc);
void UpdateEncoderValue( int ID, uint32_t Position );
#ifdef __cplusplus
}
#endif
#endif
