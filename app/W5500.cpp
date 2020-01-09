extern "C" {
#include "cmsis_os.h"
#include "gpio.h"
#include "spi.h"
#include "w5500.h"
#include "socket.h"
#include "string.h"
#include "usart.h"
#include "tim.h"
#include "ff.h"

#include "app.h"
#include "Motor.h"
#include "stdio.h"
#include "queue.h"
    /* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"
    void CLITask( void const * parment );
    void protocolRun( void const *para );
    void vRegisterCLICommands( void );
    void startCLITask();
}
#include "fifo.h"
#include "hardware.h"
#include "math.h"
#include "buildTime.h"
#include "httpServer.h"
#include "cmsis_os.h"
#include <stdarg.h>

#define DEBUGENABLE 1
#define MAX_SOCK_NUM    8

typedef struct _CONFIG_MSG
{
    uint8_t op[4];//header: FIND;SETT;FACT...
    uint8_t mac[6];
    uint8_t sw_ver[2];
    uint8_t lip[4];
    uint8_t sub[4];
    uint8_t gw[4];
    uint8_t dns[4];
    uint8_t dhcp;
    uint8_t debug;
    uint16_t fw_len;
    uint8_t state;

} CONFIG_MSG;

void spi4readwritebyte( uint8_t write )
{
    HAL_SPI_Transmit( &hspi4, &write, 1, 100 );
}
uint8_t spi4readbyte( void )
{
    uint8_t bytes;
    HAL_SPI_Receive( &hspi4, &bytes, 1, 100 );
    return bytes;
}
void spi4select()
{
    HAL_GPIO_WritePin( SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_RESET );
}
void spi4deselect()
{
    HAL_GPIO_WritePin( SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_SET ) ;
}

#define DATA_BUF_SIZE   2048

uint8_t RX_BUF[DATA_BUF_SIZE];
uint8_t TX_BUF[DATA_BUF_SIZE];
uint8_t socknumlist[] = {4, 5, 6, 7};
#define MAX_HTTPSOCK	4

// 返回结构体示例
CONFIG_MSG SetNetWorkParment( void )
{
    CONFIG_MSG networkconfig;
    networkconfig.mac[0] = 0x00;
    networkconfig.mac[1] = 0x08;
    networkconfig.mac[2] = 0x0dc;
    networkconfig.mac[3] = 0x11;
    networkconfig.mac[4] = 0x11;
    networkconfig.mac[5] = 0x17;
    networkconfig.lip[0] = 192;
    networkconfig.lip[1] = 168;
    networkconfig.lip[2] = 1;
    networkconfig.lip[3] = 215;
    networkconfig.sub[0] = 255;
    networkconfig.sub[1] = 255;
    networkconfig.sub[2] = 255;
    networkconfig.sub[3] = 0;
    networkconfig.gw[0] = 192;
    networkconfig.gw[1] = 168;
    networkconfig.gw[2] = 1;
    networkconfig.gw[3] = 213;
    networkconfig.dhcp = 0;
    networkconfig.debug = 1;
    networkconfig.fw_len = 0;
    networkconfig.state = 0;
    networkconfig.sw_ver[0] = 1;
    networkconfig.sw_ver[1] = 0;
    return networkconfig;
}


static FifoClass debugFifoBuff;

static int sockertID[ MAX_SOCK_NUM - MAX_HTTPSOCK ] = {0};
typedef int (*fun_ptr)(int);
typedef struct {
    struct {
        int type;
        uint8_t ip[4];
        uint16_t port;
    } netPara;
    FifoClass *streamIn;
    FifoClass *streamOut;
    fun_ptr onConnect;
    fun_ptr onClose;

    int socketID;
    bool use;
    bool close;
} networkDef;

networkDef socketServer[4];
void socketServer_run(int i, FifoClass *in, FifoClass *out, uint16_t port, fun_ptr func1, fun_ptr func2);

void W5500Task( void const * par )
{
    uint8_t txsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};
    uint8_t rxsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};
    uint8_t ip[4];
    for( int i = 0; i < 5; i++ )
    {
        HAL_GPIO_WritePin(INTEL_REST_GPIO_Port, INTEL_REST_Pin, GPIO_PIN_RESET );
        osDelay( 10 );
        HAL_GPIO_WritePin(INTEL_REST_GPIO_Port, INTEL_REST_Pin, GPIO_PIN_SET );
        osDelay( 10 );
    }
    for( int i = 0; i < 4; i++ )
    {
        socketServer[i].use = false;
    }

    CONFIG_MSG networkconfig = SetNetWorkParment();
//    reg_wizchip_cris_cbfunc(  vPortEnterCritical, vPortExitCritical);
    reg_wizchip_spi_cbfunc( spi4readbyte, spi4readwritebyte );
    reg_wizchip_cs_cbfunc( spi4select, spi4deselect );

    wizchip_init( txsize, rxsize );
    setSHAR( networkconfig.mac );
    setSUBR( networkconfig.sub );
    setGAR( networkconfig.gw );
    setSIPR( networkconfig.lip );

    setRTR( 2000 );
    osDelay(10);
    setRCR( 3 );
    osDelay(10);

    httpServer_init(TX_BUF, RX_BUF, MAX_HTTPSOCK, socknumlist);
    startCLITask();
    
    for( ;; )
    {
        osDelay(1);

        getSIPR (ip);
        if( ( ip[0] != 192 ) || ( ip[1] != 168) || ( ip[2] != 1) || ( ip[3] != 215 ) )
        {
            setSHAR( networkconfig.mac );
            setSUBR( networkconfig.sub );
            setGAR( networkconfig.gw );
            setSIPR( networkconfig.lip );
            setRTR( 2000 );
            setRCR( 3 );
        }
        else
        {
            for( int i = 0 ; i < MAX_HTTPSOCK; i++ )
            {
                httpServer_run(i);
            }

            for( int i = 0; i < MAX_SOCK_NUM - MAX_HTTPSOCK; i++ )
            {
                if( socketServer[i].use )
                {
                    if( socketServer[i].close )
                        close( socketServer[i].socketID );
                    socketServer_run(socketServer[i].socketID, socketServer[i].streamIn, socketServer[i].streamOut, socketServer[i].netPara.port, socketServer[i].onConnect, socketServer[i].onClose);
                }
            }
        }
    }
}



/**********************************************************************************************************************************************************
CRC16
********************************************************************************************************************************************************/
unsigned short CRC16_MODBUS(unsigned char *buff, int len )
{
    unsigned short tmp = 0xffff;
    unsigned short ret1 = 0;
    for(int n = 0; n < len; n++)
    {   /*ՋԦք6 -- ҪУҩքλ˽Ϊ6ٶ*/
        tmp = buff[n] ^ tmp;
        for(int i = 0; i < 8; i++) { /*ՋԦք8 -- ָÿһٶchar`эԖ8bitìÿbitּҪԦm*/
            if(tmp & 0x01)
            {
                tmp = tmp >> 1;
                tmp = tmp ^ 0xa001;
            }
            else
            {
                tmp = tmp >> 1;
            }
        }
    }
    ret1 = tmp >> 8;
    ret1 = ret1 | (tmp << 8);
    return ret1;
}
/*************************************************************************************************************************************************************
数据打包
******************************************************************/
int makePack( uint8_t *packBuf, uint64_t packIndex, uint16_t packCmd, uint8_t packErrorCode, int packLen, uint8_t *data )
{
    union {
        uint64_t Data;
        uint8_t Hex[8];
    } Uint64ToHex;

    union {
        uint16_t Data;
        uint8_t Hex[2];
    } Uint16ToHex;

    union {
        uint32_t Data;
        uint8_t Hex[4];
    } Uint32ToHex;

    uint8_t * buff;
    buff = packBuf;

    *packBuf++ = 'l';
    *packBuf++ = 's';
    *packBuf++ = 'p';
    *packBuf++ = 'h';

    Uint64ToHex.Data = packIndex;

    for( int i = 0; i < 8; i++ )
    {
        *packBuf++ = Uint64ToHex.Hex[i];
    }

    Uint16ToHex.Data = packCmd;
    for( int i = 0; i < 2; i++ )
    {
        *packBuf++ = Uint16ToHex.Hex[i];
    }
    *packBuf++ = packErrorCode;
    Uint32ToHex.Data = packLen;

    for( int i = 0; i < 4; i++ )
    {
        *packBuf++ = Uint32ToHex.Hex[i];
    }

    for( int i = 0; i < packLen; i++ )
    {
        *packBuf++ = *data++;
    }

    unsigned short crc = CRC16_MODBUS( buff, packBuf - buff );
    Uint16ToHex.Data = crc;
    *packBuf++ = Uint16ToHex.Hex[0];
    *packBuf++ = Uint64ToHex.Hex[1];
    return packBuf - buff;
}
/***********************************************************************************************************************************************************
解包
*/////////

int decodePack( uint8_t *packBuff, int buffLen, uint64_t *packIndex, uint16_t *packCmd, uint8_t *packErrorCode, int *packLen, uint8_t *data )
{
    uint16_t crc = 0;
    union {
        uint32_t Data;
        uint8_t Hex[4];
    } Uint32ToHex;
    union {
        uint64_t Data;
        uint8_t Hex[8];
    } Uint64ToHex;
    union {
        uint16_t Data;
        uint8_t Hex[2];
    } Uint16ToHex;
    uint8_t *packPiont = packBuff;

    for( int i = 0; i < 4; i++ )
    {
        Uint32ToHex.Hex[i] = *packPiont++;
    }
    if( Uint32ToHex.Data != 0x6870736c )
        return -1;
    for( int i = 0; i < 8; i++ )
    {
        Uint64ToHex.Hex[i] = *packPiont++;
    }
    if( packIndex )
        *packIndex = Uint64ToHex.Data;

    Uint16ToHex.Hex[1] = *packPiont++;
    Uint16ToHex.Hex[0] = *packPiont++;
    if( packCmd )
        *packCmd = Uint16ToHex.Data;

    if( packErrorCode )
        *packErrorCode = *packPiont++;

    for( int i = 0; i < 4; i++ )
    {
        Uint32ToHex.Hex[i] = *packPiont++;
    }
    if( packLen )
        *packLen = Uint32ToHex.Data;
    if( *packLen + 21 > buffLen )
        return pdFALSE;

    crc = CRC16_MODBUS( packBuff, 19 + *packLen );
    Uint16ToHex.Hex[0] = *(packBuff + 19 + *packLen );
    Uint16ToHex.Hex[1] = *(packBuff + 20 + *packLen );

    if( crc == Uint16ToHex.Data )
        return pdTRUE;
    else
        return pdFALSE;
}
/**** ****************************************************************************************************************************************************
调度通信线程
* *********************************************************************************************************************************************************/
int createSocket(FifoClass *in, FifoClass *out, uint16_t port, fun_ptr onCreatep, fun_ptr onClosep );
void protocolRun( void const *para )
{
    static FifoClass dataIn, dataOut;
    uint64_t packIndex = 0;
    uint16_t packCMD = 0;
    uint8_t  errorCode;
    int PackLen;

    createSocket( &dataIn, &dataOut, 5000, 0, 0 );
    uint8_t buff[500];
    uint8_t data[10];

    for( ;; )
    {
        if( dataIn.available() >= 21 )
        {
            dataIn.pickData( buff, 21 );
            if( decodePack( buff, 21, &packIndex, &packCMD, &errorCode, &PackLen, data ) >= 0 )
            {
                if( dataIn.available() >= PackLen + 21 )
                {
                    dataIn.popData( buff, PackLen + 21 );
                    if( decodePack( buff, 21, &packIndex, &packCMD, &errorCode, &PackLen, data ) == pdTRUE )
                    {
                        /* 
                        解包成功
                        */
                    }
                }
            }
            else
            {
                dataIn.popData( buff, 1 );
            }
        }
        osDelay( 1 );
    }
}
/* ******************************************************************************
socket 服务线程
*************************************************************************************/
void socketServer_run(int i, FifoClass *in, FifoClass *out, uint16_t port, fun_ptr func1, fun_ptr func2)
{
    static uint8_t buffer[2048];
    int len = 0;
    switch(getSn_SR(i))
    {
    case SOCK_INIT:
        listen(i);
        if( func1 )
            func1(0);
        break;
    case SOCK_ESTABLISHED:
        if(getSn_IR(i) & Sn_IR_CON)
        {
            setSn_IR(i, Sn_IR_CON);
        }
        len=getSn_RX_RSR(i);
        if(len>0)
        {
            recv(i, buffer, len );
            if( in )
                in->pushData( buffer, len );
        }
        if( out )
        {
            len = out->available();
            if( len )
            {
                out->popData( buffer, len );
                send( i, buffer, len );
            }
        }
        break;
    case SOCK_CLOSE_WAIT:
        if( socketServer[i].streamIn )
            socketServer[i].streamIn->clean();
        if( socketServer[i].streamOut )
            socketServer[i].streamOut->clean();
        close(i);
        if( func2 )
            func2(1);
        break;
    case SOCK_CLOSED:
        socket(i,Sn_MR_TCP,port,Sn_MR_ND);
        break;
    default:
        break;
    }
}
int getFreeSocketID()
{
    for( int i = 0; i < 4; i++ )
    {
        if( !sockertID[i] )
            return i;
    }
    return -1;
}
int getFreeSocketData()
{
    for( int i = 0; i < 4; i++ )
    {
        if( !socketServer[i].use )
            return i;
    }
    return -1;
}
int createSocket(FifoClass *in, FifoClass *out, uint16_t port, fun_ptr onCreatep, fun_ptr onClosep )
{
    int socketDataid = getFreeSocketData();
    socketServer[socketDataid].socketID = getFreeSocketID();
    socketServer[socketDataid].streamIn = in;
    socketServer[socketDataid].streamOut = out;
    socketServer[socketDataid].netPara.port = port;
    socketServer[socketDataid].use = true;
    socketServer[socketDataid].close = false;
    socketServer[socketDataid].onConnect = onCreatep;
    sockertID[socketServer[socketDataid].socketID] = 1;

    return socketDataid;
}


/* ***************************************************************************************************/
// CLI Interface begin
static bool showBanner = true;
static int cliNetworkID = 0;
int cliSOcketConnect( int i )
{
    socketServer[cliNetworkID].streamOut->pushData( ( uint8_t * ) "Welcome to LS-RGV Command Line Interface\t", sizeof( "Welcome to LS-RGV Command Line Interface\t" ) );
    socketServer[cliNetworkID].streamOut->pushData( ( uint8_t * )"\r\nLS-RGV $ ", sizeof( "\r\nLS-RGV $ " ) );
}

void CLITask( void const * parment )
{
    BaseType_t xMoreDataToFollow;
    long lBytes;
    static char cInputString[ 100 ], cOutputString[ 1024 ];
    static FifoClass stdinBuff;
    static FifoClass stdoutBuff;

    cliNetworkID = createSocket( &stdinBuff, &stdoutBuff, 5002, cliSOcketConnect, NULL );
    createSocket( NULL, &debugFifoBuff, 5001, NULL, NULL );

    vRegisterCLICommands();
    stdinBuff.clean();

    for( ;; )
    {
        if( stdinBuff.available() )
        {
            if( stdinBuff.back() == '\n' )
            {
                lBytes = stdinBuff.popData( (uint8_t *)cInputString, stdinBuff.available() );
                cInputString[lBytes] = 0;
                if( cInputString[lBytes - 2] == '\r' )
                    cInputString[lBytes - 2] = 0;
                if( cInputString[lBytes - 1] == '\n' )
                    cInputString[lBytes - 1] = 0;
                do
                {
                    xMoreDataToFollow = FreeRTOS_CLIProcessCommand( ( const char * ) cInputString, cOutputString, (size_t) sizeof( cOutputString ) );
                    stdoutBuff.pushData( (uint8_t *) cOutputString, strlen( cOutputString ) );
                } while( xMoreDataToFollow != pdFAIL );
                stdoutBuff.pushData( ( uint8_t * ) "\r\nLS-RGV $ ", sizeof( "\r\nLS-RGV $ " ) );
            }
            else
                osDelay(1);
        }
        else
            osDelay(1);
    }
}
// CLI interfact end
/* *****************************************************************************************************************/
extern "C" {
    int fputc( int c, FILE *f );
    int debugOut( int isISR, char *fmt, ... );
}

int debugOut( int isISR, char *fmt, ... )
{
    if( !isISR )
    {
        taskENTER_CRITICAL();
    }
    va_list args;
    va_start( args, fmt );
    vprintf( fmt, args );
    va_end(args);
    if( !isISR )
        taskEXIT_CRITICAL();
    return 0;
}
int fputc( int c, FILE *f )
{
#if DEBUGENABLE
    debugFifoBuff.pushData( (uint8_t*) &c, 1 );
#endif
}