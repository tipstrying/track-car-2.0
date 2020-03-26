#include "fifo.h"

#include "stdio.h"
#include "string.h"

FifoClass::FifoClass(/* args */)
{
    pStart = buffData;
    pEnd = buffData;
    pBuffMax = buffData + sizeof( buffData );
    pBuffMin = buffData;

    overflow = false;
    memset(buffData,0,sizeof(buffData));
}
void FifoClass::clean()
{
    pStart = buffData;
    pEnd = buffData;
    pBuffMax = buffData + sizeof( buffData );
    pBuffMin = buffData;

    overflow = false;
}
FifoClass::~FifoClass()
{
}
uint8_t FifoClass::back()
{
    if( pEnd > buffData )
        return *(pEnd - 1 );
    else
        return *buffData;
}
uint8_t FifoClass::front()
{
    return *pStart;
}

int FifoClass::available()
{
    if( pEnd >= pStart )
        /* when pEnd >= pStart, it means all available data at pStart to pEnd */
    {
        if( overflow )
            /* at overflow, pStart always equal pEnd */
        {
            return sizeof( buffData );
        }
        else
        {
            return pEnd - pStart;
        }
    }
    else
        /* when pEnd < pStart, it means all available data are outside pStart to pEnd*/
    {
        return sizeof( buffData ) - ( pStart - pEnd );
    }
}
int FifoClass::getFreeSize()
{
    if (overflow)
        return 0;
    else
    {
        if( pEnd >= pStart )
        {
            return sizeof(buffData) - (pEnd - pStart );
        }
        else
        {
            return pStart - pEnd;
        }

    }
}
int FifoClass::writeToBuff(uint8_t *data, int len)
{
    if (len >= getFreeSize())
        overflow = true;
    if (buffData + sizeof(buffData) - pEnd > len)
    {
        memcpy(pEnd, data, len);
        pEnd = pEnd + len;
        if (overflow)
            pStart = pEnd;
        return len;
    }
    else
    {
        int remainingLen = buffData + sizeof(buffData) - pEnd;
        memcpy(pEnd, data, remainingLen);
        pEnd = buffData;
        memcpy(pEnd, data + remainingLen, len - remainingLen);
        pEnd = buffData + (len - remainingLen);
        if (overflow)
            pStart = pEnd;
        return len;
    }
}
int FifoClass::readFromBuff( uint8_t *data, int len, bool iSThrowData )
{
    if (available() >= len)
    {
        if (pBuffMax - pStart >= len)
        {
            if( data != NULL )
                memcpy(data, pStart, len);
            if (iSThrowData)
            {
                pStart += len;
                overflow = false;
            }
            return len;
        }
        else
        {
            int remaining = pBuffMax - pStart;
            if( data != NULL )
            {
                memcpy(data, pStart, remaining);
                memcpy(data + remaining, pBuffMin, len - remaining);
            }
            if (iSThrowData)
            {
                pStart = pBuffMin + (len - remaining);
                overflow = false;
            }
            return len;
        }
    }
    else
    {
        return 0;
    }
}
int FifoClass::pushData(uint8_t *data, int len)
{
    if (len > getFreeSize())
    {
        overflow = true;
        writeToBuff( data, len);
        pStart = pEnd;
    }
    else
    {
        writeToBuff(data,len);
    }
    return len;
}
int FifoClass::pushData( int data )
{
    return pushData( (uint8_t *) &data, 4 );

}
int FifoClass::pickData( uint8_t *data, int len)
{
    return readFromBuff(data, len, false);
}
int FifoClass::popData(uint8_t *data, int len)
{
    return readFromBuff(data,len,true);
}
int FifoClass::popData( int *data )
{
    return popData( (uint8_t *) data, 4 );
}
void FifoClass::displayData() {
    uint8_t buff[sizeof(buffData)];
    memset( buff,0, sizeof(buff));
    pickData(buff, available());
    printf("Overflow:%d\tAvailable:%d\tData:%s\r\n",overflow ? 1 : 0, available(), buff);
}
