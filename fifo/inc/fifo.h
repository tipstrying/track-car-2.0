#ifndef __PROTOCOLBUFF_H
#define __PROTOCOLBUFF_H
#include "stdint.h"
/* fifo buff for data receive */
class FifoClass
{
private:
    /* buff */
    uint8_t buffData[1024 * 2 ];
    /* valid data start and end piont */
    uint8_t *pStart, *pEnd;
    /* start and end piont of buff */
    uint8_t * pBuffMax, *pBuffMin;
    /* overflow flag */
    /* true: data buff has overflowed */
    bool overflow;
    /* get free size of buff */
    int getFreeSize();
    /* write data to buff */
    int writeToBuff(uint8_t *, int);
    /* read data from buff */
    /* bool: true */
    int readFromBuff(uint8_t *, int, bool);

public:
    /* data to fifo */
    int pushData(uint8_t * data, int len);
    int pushData( int data );
    /* get data from fifo */
    int popData( uint8_t * data, int len);
    int popData( int * data);
    int pickData( uint8_t *, int );
    int available();
    /* set fifo clean  */
    void clean();
    void displayData();
    uint8_t back();
    uint8_t front();
    FifoClass(/* args */);
    ~FifoClass();
};

#endif
