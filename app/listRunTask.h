#ifndef __LISTRUNTASK_H
#define __LISTRUNTASK_H
#include <stdlib.h>
#include <stdint.h>
#include "freertos.h"
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct IOBoardDataStruct {
    int ID;
    float position;
    int cmd;
    uint32_t clock;
    union {
        float    fData;
        uint32_t uData;
        int      iData;
    } data;
    struct IOBoardDataStruct * next;
} RunTaskDef;

typedef enum {
    setSpeed = 1,
    stickOut = 2,
    stickIn = 3,
    stickOutOk = 4,
    stickInOk = 5,
    setZero = 6,
} opEnumDef;

void deleteList( RunTaskDef * header );
int listAdd( RunTaskDef * header, RunTaskDef dataIn );
int listGetItemByID( RunTaskDef * header, int ID, RunTaskDef * dataOut);
int listUpdateItemByID( RunTaskDef *header, int ID, RunTaskDef dataIn );
int listDeleteItemByIndex( RunTaskDef *header, int Index );
int listConverToJson( RunTaskDef *header, char *buff );
int listGetItemByCMD( RunTaskDef *header, int cmd, RunTaskDef *dataOut );
int listDeleteItemByCMD( RunTaskDef *header, int ID );

#ifdef __cplusplus
}
#endif
#endif
