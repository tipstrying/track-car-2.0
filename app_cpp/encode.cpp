#include "encode.h"
void encodeClass::clear()
{
    this->encode.status = 0;
    this->encode.A = 0;
    this->encode.B = 0;
}
encodeClass::encodeClass()
{
    this->encode.status = 0;
    this->encode.A = 0;
    this->encode.B = 0;
}
int encodeClass::input( encodeClass::encodeIODef io )
{
    switch( io )
    {
    case encodeA:
        encode.A = 1;
        break;
    case encodeB:
        encode.B = 1;
        break;
    case encodeAB:
        encode.A = 1;
        encode.B = 1;
        break;
    default:
        break;
    }
    if( encode.A && encode.B )
        return 1;
    else
        return 0;
}
int encodeClass::getValue()
{
    if( encode.A && encode.B )
    {
        return 1;
    }
    else
        return 0;
}