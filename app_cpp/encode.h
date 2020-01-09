#ifndef __ENCODE_H
#define __ENCODE_H
class encodeClass
{
public:
    typedef enum {
        encodeA = 1,
        encodeB = 2,
        encodeAB = 3
    } encodeIODef;

    int input( encodeIODef IO );
    int getValue();
    encodeClass();
    void clear();
private:
    int rValue;
    struct {
        char A;
        char B;
        int status;
    } encode;
};
#endif