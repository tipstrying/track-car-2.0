#include "fifo.h"
#include "stdio.h"
#include "string.h"

int main()
{
    uint8_t buff[10];
    char c;
    FifoClass pbuff;
    while (1)
    {
        int i = 0;

        while (c = getchar())
        {
            if (c == '\n')
            {
                break;
            }
            if( c == 'P')
            {
                memset( buff, 0, sizeof( buff ));

                pbuff.pickData(buff, 7 );
                printf( "Pick Data:%s\r\n", buff);
                break;
            }
            if( c == 'p')
            {
                memset( buff, 0, sizeof( buff ));
                pbuff.popData(buff, 7);
                printf("PoP Data:%s\r\n", buff);
                break;
            }
            buff[i] = c;
            i++;
            if (i >= 10)
                break;
        }
        pbuff.pushData(buff, i);
        pbuff.displayData();
    }
}