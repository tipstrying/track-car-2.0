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
    networkconfig.mac[2] = 0xdc;
    networkconfig.mac[3] = 0x11;
    networkconfig.mac[4] = 0x11;
    networkconfig.mac[5] = 0x13;
    networkconfig.lip[0] = 192;
    networkconfig.lip[1] = 168;
    networkconfig.lip[2] = 1;
    networkconfig.lip[3] = 11;
    networkconfig.sub[0] = 255;
    networkconfig.sub[1] = 255;
    networkconfig.sub[2] = 255;
    networkconfig.sub[3] = 0;
    networkconfig.gw[0] = 192;
    networkconfig.gw[1] = 168;
    networkconfig.gw[2] = 1;
    networkconfig.gw[3] = 1;
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
        if( ( ip[0] != 192 ) || ( ip[1] != 168) || ( ip[2] != 1) || ( ip[3] != 11 ) )
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
        *packBuf++ = Uint64ToHex.Hex[7-i];
    }

    Uint16ToHex.Data = packCmd;
    for( int i = 0; i < 2; i++ )
    {
        *packBuf++ = Uint16ToHex.Hex[1-i];
    }
    *packBuf++ = packErrorCode;
    Uint32ToHex.Data = packLen;

    for( int i = 0; i < 4; i++ )
    {
        *packBuf++ = Uint32ToHex.Hex[3-i];
    }

    for( int i = 0; i < packLen; i++ )
    {
        *packBuf++ = *data++;
    }

    unsigned short crc = CRC16_MODBUS( buff, packBuf - buff );
    Uint16ToHex.Data = crc;
    *packBuf++ = Uint16ToHex.Hex[0];
    *packBuf++ = Uint16ToHex.Hex[1];
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
        Uint64ToHex.Hex[7 - i] = *packPiont++;
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
        Uint32ToHex.Hex[3-i] = *packPiont++;
    }
    if( packLen )
        *packLen = Uint32ToHex.Data;
    if( *packLen + 21 > buffLen )
        return pdFALSE;

    for( int i = 0; i < *packLen; i++ )
        data[i] = *packPiont++;
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
    union {
        uint8_t Hex[4];
        int  Data;
    } i32ToHex;

    createSocket( &dataIn, &dataOut, 8802, 0, 0 );
    uint8_t buff[500];
    uint8_t data[100];
    NavigationOperationStd navData;
    for( ;; )
    {
        if( dataIn.available() >= 21 )
        {
            dataIn.pickData( buff, 21 );
            if( decodePack( buff, 21, &packIndex, &packCMD, &errorCode, &PackLen, data ) >= 0 )
            {
                if( dataIn.available() >= PackLen + 21 )
                {
                    PackLen = dataIn.popData( buff, PackLen + 21 );
                    if( decodePack( buff, PackLen, &packIndex, &packCMD, &errorCode, &PackLen, data ) == pdTRUE )
                    {
                        /*
                        解包成功
                        */
                        debugOut( 0, (char *)"Decode pack ok: index->%lld,cmd->%d,packLen->%d,date:", packIndex, packCMD, PackLen );
                        for( int i = 0; i < PackLen; i++ )
                        {
                            debugOut( 0, (char *)"0X%02X ", data[i] );
                        }
                        debugOut( 0, (char *)"\r\n" );

                        switch( packCMD )
                        {
                        case 2001:
                            if( 1 )
                            {
                                float pos, speed;
                                int posInt;
                                uint8_t status;
                                uint8_t batVol;
                                status = IsArriver();

                                union {
                                    uint8_t Hex[4];
                                    int     Data;
                                } i32ToHex;

                                union {
                                    uint8_t Hex[2];
                                    uint16_t Data;
                                } u16ToHex;

                                GetPosition( &pos );
                                GetSpeed( &speed );
                                i32ToHex.Data = (int) pos;
                                u16ToHex.Data = (uint16_t)speed;
                                status  = 0;
                                batVol = (uint8_t) (Battery.Voltage / 1000);
                                for( int i = 0; i < 4; i++ )
                                {
                                    data[i] = i32ToHex.Hex[3-i];
                                }
                                data[4] = u16ToHex.Hex[1];
                                data[5] = u16ToHex.Hex[0];

                                data[6] = status;

                                data[7] = batVol;

                                PackLen = makePack( buff, packIndex, 12001, 0, 8, data );
                                dataOut.pushData( buff, PackLen );
                            }

                            break;
                        case 2002:
                            navData.cmd = 2;
                            navData.Data.speedTo = data[0] + data[1] * 255;
                            if( xQueueSend( NavigationOperationQue, &navData, 1000 ) == pdPASS )
                            {
                                PackLen = makePack( buff, packIndex, 12002, 0, 0, NULL );
                                dataOut.pushData( buff, PackLen );
                            }
                            else
                            {
                                PackLen = makePack( buff, packIndex, 12002, 1, 0, NULL );
                                dataOut.pushData( buff, PackLen );
                            }

                            break;
                        case 2003:
                            if( 1 )
                            {
                                uint8_t trails = data[2];
                                int rValue = 0;
                                for( int i = 0; i < trails; i++ )
                                {
                                    if( rValue )
                                        break;
                                    for( int j = 0; j < 4; j++ )
                                    {
                                        i32ToHex.Hex[3-j] = data[ 3 + j + i * 7];
                                    }
                                    navData.cmd = 3;
                                    navData.Data.posTo = i32ToHex.Data;
                                    if( xQueueSend( NavigationOperationQue, &navData, 100 ) != pdPASS )
                                    {
                                        if( !rValue )
                                            rValue = 0;
                                    }

                                    if( data[ 7 + i * 7 ] + data[ 8 + i * 7 ] != 0 )
                                    {
                                        navData.cmd = 4;
                                        navData.Data.speedTo = data[ 7 + i * 7 ] + data[ 8 + i * 7 ] * 255;
                                        navData.Data.op = 1;
                                        navData.Data.posTo = i32ToHex.Data;
                                        if( xQueueSend( NavigationOperationQue, &navData, 100 ) == pdPASS )
                                        {
                                            if( !rValue )
                                                rValue = 0;
                                        }
                                    }
                                    if( data[ 9 + i * 7 ] )
                                    {
                                        navData.cmd = 4;
                                        navData.Data.posTo = i32ToHex.Data;
                                        navData.Data.op = data[ 9 + i * 7 ] + 1;
                                        if( xQueueSend( NavigationOperationQue, &navData, 100 ) == pdPASS )
                                        {
                                            if( !rValue )
                                                rValue = 0;
                                        }
                                    }
                                }
                                if( rValue )
                                {
                                    PackLen = makePack( buff, packIndex, 12003, 1, 0, 0 );
                                }
                                else
                                {
                                    PackLen = makePack( buff, packIndex, 12003, 0, 0, 0 );
                                }
                                dataOut.pushData( buff, PackLen );
                            }
                            break;
                        case 2004:
                            if( 1 )
                            {
                                int rValue = 0;
                                if( rValue )
                                {
                                    PackLen = makePack( buff, packIndex, 12004, 1, 0, 0 );
                                }
                                else
                                {
                                    PackLen = makePack( buff, packIndex, 12004, 0, 0, 0 );
                                }
                                dataOut.pushData( buff, PackLen );
                            }
                            break;
                        case 2005:
                            if( 1 )
                            {
                                int rValue = 0;
                                if( rValue )
                                {
                                    PackLen = makePack( buff, packIndex, 12005, 1, 0, 0 );
                                }
                                else
                                {
                                    PackLen = makePack( buff, packIndex, 12005, 0, 0, 0 );
                                }
                                dataOut.pushData( buff, PackLen );
                            }
                            break;
                        case 2006:
                            if( 1 )
                            {
                                int rValue = 0;
                                if( rValue )
                                {
                                    PackLen = makePack( buff, packIndex, 12006, 1, 0, 0 );
                                }
                                else
                                {
                                    PackLen = makePack( buff, packIndex, 12006, 0, 0, 0 );
                                }
                                dataOut.pushData( buff, PackLen );
                            }
                            break;
                        case 2007:
                            if( 1 )
                            {

                                int rValue = 0;
                                if( rValue )
                                {
                                    PackLen = makePack( buff, packIndex, 12007, 1, 0, 0 );
                                }
                                else
                                {
                                    PackLen = makePack( buff, packIndex, 12007, 0, 0, 0 );
                                }
                                dataOut.pushData( buff, PackLen );
                            }
                            break;
                        case 2008:
                            if( 1 )
                            {
                                uint8_t isThingOnCar = 0;
                                PackLen = makePack( buff, packIndex, 12008, 0, 1, &isThingOnCar );
                                dataOut.pushData( buff, PackLen );
                            }
                            break;
                        case 2010:
                            if( 1 )
                            {
                                PackLen = makePack( buff, packIndex, 12010, 0, 0, 0 );
                                dataOut.pushData( buff, PackLen );
                            }
                            break;
                        case 2011:
                            if( 1 )
                            {
                                navData.cmd = 1;
                                xQueueSend( NavigationOperationQue, &navData, 100 );
                                PackLen = makePack( buff, packIndex, 12011, 0, 0, 0 );
                                dataOut.pushData( buff, PackLen );
                            }

                            break;
                        case 2013:
                            if( 1 )
                            {
                                PackLen = makePack( buff, packIndex, 12013, 0, 0, 0 );
                                dataOut.pushData( buff, PackLen );
                            }
                            break;
                        case 2016:
                            if( data[0] )
                            {
                                HAL_GPIO_WritePin( OUT_9_GPIO_Port, OUT_9_Pin, GPIO_PIN_SET );
                            }
                            else
                                HAL_GPIO_WritePin( OUT_9_GPIO_Port, OUT_9_Pin, GPIO_PIN_SET );

                            PackLen = makePack( buff, packIndex, 12016, 0, 0, 0 );
                            dataOut.pushData( buff, PackLen );
                            break;
                        case 2017:
                            if( 1 )
                            {
                                uint8_t isChargeKeyOpen = HAL_GPIO_ReadPin( OUT_9_GPIO_Port, OUT_9_Pin );
                                PackLen = makePack( buff, packIndex, 12017, 0, 1, &isChargeKeyOpen );
                                dataOut.pushData( buff, PackLen );
                            }
                            break;
                        case 2018:
                            /*
                            navData.cmd = 5;
                            navData.Data.op = 1;
                            xQueueSend( NavigationOperationQue, &navData, 100 );
                            */
                            PackLen = makePack( buff, packIndex, 12018, 0, 0, 0 );
                            dataOut.pushData( buff, PackLen );
                            break;
                        case 2019:
                            navData.cmd = 5;
                            if( data[0] == 1 )

                                navData.Data.op = 1;
                            else if( data[0] == 2 )
                                navData.Data.op = 0;
                            else
                            {
                                PackLen = makePack( buff, packIndex, 12019, 1, 0, 0 );
                                dataOut.pushData( buff, PackLen );
                                break;
                            }

                            xQueueSend( NavigationOperationQue, &navData, 100 );

                            PackLen = makePack( buff, packIndex, 12019, 0, 0, 0 );
                            dataOut.pushData( buff, PackLen );
                            break;
                        }
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