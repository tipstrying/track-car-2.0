#include "cmsis_os.h"
#include "gpio.h"
#include "spi.h"
#include "w5500.h"
#include "socket.h"
#include "string.h"
#include "usart.h"
#include "tim.h"
#include "ff.h"
#include "battery.h"
#include "app.h"
#include "MotorTest.h"
#include "Motor.h"

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


extern int fatfsstatus;
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
CONFIG_MSG SetNetWorkParment( void )
{
    FIL DevicesInfo;
    char buff[100];
    CONFIG_MSG networkconfig;
    if( fatfsstatus )
    {
        if( f_open( &DevicesInfo, "0:info.txt", FA_READ ) == FR_OK )
        {
            UINT bytes;
            if( f_read( &DevicesInfo, buff, sizeof(buff),&bytes ) == FR_OK )
            {
                if( bytes > 0 )
                {
                    if( sscanf( buff, "IP:%hhu.%hhu.%hhu.%hhu\r\nIPMask:%hhu.%hhu.%hhu.%hhu\r\nGateWay:%hhu.%hhu.%hhu.%hhu\r\nMAC:%hhx,%hhx,%hhx,%hhx,%hhx,%hhx\r\n", networkconfig.lip, networkconfig.lip +1,\
                                networkconfig.lip + 2, networkconfig.lip + 3, networkconfig.sub, networkconfig.sub + 1, networkconfig.sub + 2, networkconfig.sub + 3, networkconfig.gw, networkconfig.gw + 1,\
                                networkconfig.gw + 2, networkconfig.gw + 3, networkconfig.mac, networkconfig.mac + 1, networkconfig.mac + 2, networkconfig.mac + 3, networkconfig.mac + 4, networkconfig.mac + 5 ) == 18 )
                    {
                        networkconfig.dhcp = 0;
                        networkconfig.debug = 1;
                        networkconfig.fw_len = 0;
                        networkconfig.state = 0;
                        networkconfig.sw_ver[0] = 1;
                        networkconfig.sw_ver[1] = 0;
                    }
                    else
                        goto ERRORFILE;
                }
                else
                    goto ERRORFILE;
            }
        }
        else
        {
ERRORFILE:
            networkconfig.mac[0] = 0x00;
            networkconfig.mac[1] = 0x08;
            networkconfig.mac[2] = 0x0dc;
            networkconfig.mac[3] = 0x11;
            networkconfig.mac[4] = 0x11;
            networkconfig.mac[5] = 0x11;
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
            networkconfig.gw[3] = 1;
            networkconfig.dhcp = 0;
            networkconfig.debug = 1;
            networkconfig.fw_len = 0;
            networkconfig.state = 0;
            networkconfig.sw_ver[0] = 1;
            networkconfig.sw_ver[1] = 0;
        }
    }
    else
    {
        networkconfig.mac[0] = 0x00;
        networkconfig.mac[1] = 0x08;
        networkconfig.mac[2] = 0x0dc;
        networkconfig.mac[3] = 0x11;
        networkconfig.mac[4] = 0x11;
        networkconfig.mac[5] = 0x11;
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
        networkconfig.gw[3] = 1;
        networkconfig.dhcp = 0;
        networkconfig.debug = 1;
        networkconfig.fw_len = 0;
        networkconfig.state = 0;
        networkconfig.sw_ver[0] = 1;
        networkconfig.sw_ver[1] = 0;
    }
    return networkconfig;
}
uint8_t buffer[1500];
uint8_t buffer1[1500];
int len, len1 ;
uint16_t local_port=5000;

int W5500SoftWatchDogCounter = 0;
static CONFIG_MSG networkconfig;
static DebugBuffStd msg;
void W5500Task( void const * par )
{
#define MAX_SOCK_NUM    8
    uint8_t txsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};		// ѡձ8ٶSocketÿٶSocketע̍ۺզքճСì՚w5500.cքvoid sysinit()Ԑʨ׃ڽԌ
    uint8_t rxsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};		// ѡձ8ٶSocketÿٶSocketޓ˕ۺզքճСì՚w5500.cքvoid sysinit()Ԑʨ׃ڽԌ

    uint8_t ip[4];
    // MotorStd motorcmdsend;
    networkconfig = SetNetWorkParment();
    reg_wizchip_cris_cbfunc(  vPortEnterCritical, vPortExitCritical);
    reg_wizchip_spi_cbfunc( spi4readbyte, spi4readwritebyte );
    reg_wizchip_cs_cbfunc( spi4select, spi4deselect );
    HAL_GPIO_WritePin(INTEL_REST_GPIO_Port, INTEL_REST_Pin, GPIO_PIN_RESET );
    osDelay( 100 );
    HAL_GPIO_WritePin(INTEL_REST_GPIO_Port, INTEL_REST_Pin, GPIO_PIN_SET );
    osDelay( 1000 );
    wizchip_init( txsize, rxsize );
    setSHAR( networkconfig.mac );
    setSUBR( networkconfig.sub );
    setGAR( networkconfig.gw );
    setSIPR( networkconfig.lip );

    setRTR( 2000 );
    osDelay(10);
    setRCR( 3 );
    osDelay(10);

//    MotorTestCmdDef MotorCmd;
//    MainCMDStd maincmd;
//    uint32_t DelayCount;

    union {
        double Data;
        uint8_t Hex[8];
    } DoubleToHex;

    union {
        float Data;
        uint8_t Hex[4];
    } FloatToHex;
    union {
        uint32_t Data;
        uint8_t Hex[4];
    } Uint32ToHex;
    float xPosition, yPosition;
    float xSpeed, ySpeed;
    uint8_t *p;
    uint8_t *pSendBuff;
    static uint32_t PackID;
    for( ;; )
    {
        osDelay(1);
        getSIPR (ip);
        if( ( ip[0] != 192 ) || ( ip[1] != 168) || ( ip[2] != 1) || ( ip[3] != 215 ) )
        {
            DEBUG_OUT( "W5500 Reset\r\n" );
            setSHAR( networkconfig.mac );
            setSUBR( networkconfig.sub );
            setGAR( networkconfig.gw );
            setSIPR( networkconfig.lip );
            setRTR( 2000 );
            setRCR( 3 );
        }
        W5500SoftWatchDogCounter = 0;
        switch(getSn_SR(0))
        {
        case SOCK_INIT:
            listen(0);
            break;
        case SOCK_ESTABLISHED:
            if(getSn_IR(0) & Sn_IR_CON)
            {
                setSn_IR(0, Sn_IR_CON);
            }
            len=getSn_RX_RSR(0);
            if(len>0)
            {
                if( len > 1500 )
                {
                    recv(0, buffer, 1500 );
                    break;
                }
                else
                {
                    recv(0, buffer, len );
                }
                for( int i = 0; i < 4; i++ )
                {
                    Uint32ToHex.Hex[i] = buffer[i];
                }
                if( Uint32ToHex.Data == 0xaaaa5555 )
                {
                    /* 通信协议 */
                    /* 包头           序号          命令                  数据 */
                    /* 0xaaaa5555      4bytes         1（读取状态）      X（ float ） Y （ float ） Xspeed （ float） Yspeed（ float ）*/

                    p = buffer + 4;
                    for( int i = 0; i < 4; i++ )
                    {
                        Uint32ToHex.Hex[i] = *p;
                        p++;
                    }
                    PackID = Uint32ToHex.Data;

                    switch( *p )
                    {
                    case 1:
                        p++;
                        if( len == 9 )
                        {
                            pSendBuff = buffer1;
                            Uint32ToHex.Data = 0xaaaa5555;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            Uint32ToHex.Data = PackID;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            *pSendBuff = 1;
                            pSendBuff++;
                            GetPosition( &xPosition, &yPosition );
                            FloatToHex.Data = xPosition;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = FloatToHex.Hex[i];
                                pSendBuff++;
                            }
                            FloatToHex.Data = yPosition;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = FloatToHex.Hex[i];
                                pSendBuff++;
                            }
                            GetSpeed( &xSpeed, &ySpeed );
                            FloatToHex.Data = xSpeed;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = FloatToHex.Hex[i];
                                pSendBuff++;
                                // buffer1[13+i] = FloatToHex.Hex[i];
                            }
                            FloatToHex.Data = ySpeed;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = FloatToHex.Hex[i];
                                pSendBuff++;
                                // buffer1[17+i] = FloatToHex.Hex[i];
                            }
                            DoubleToHex.Data = GetMilage();
                            for( int i = 0; i < 8; i++ )
                            {
                                *pSendBuff = DoubleToHex.Hex[i];
                                pSendBuff++;
                                //buffer1[21+i] = DoubleToHex.Hex[i];
                            }
                            Uint32ToHex.Data = IsArriver();
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                                //  buffer1[29+i] = Uint32ToHex.Hex[i];
                            }
                            //buffer1[33] = IsX();
                            // buffer1[33] = BeltOperatingStatus();
                            *pSendBuff = BeltOperatingStatus();
                            pSendBuff++;
                            if( HAL_GPIO_ReadPin( IN_T1_Port, IN_T1_Pin ) || HAL_GPIO_ReadPin( IN_T2_Port, IN_T2_Pin ) || HAL_GPIO_ReadPin( IN_T3_Port, IN_T3_Pin ))
                            {
                                *pSendBuff = 1;
                                pSendBuff++;
                                // buffer1[34] = 1;
                            }
                            else
                            {
                                *pSendBuff = 0;
                                pSendBuff++;
                                //  buffer1[34] = 0;
                            }
                            if( HAL_GPIO_ReadPin( OUT_Charge_C_Port, OUT_Charge_C_Pin ) )
                                // buffer1[35] = 1;
                                *pSendBuff = 1;
                            else
                                //  buffer1[35] = 0;
                                *pSendBuff = 0;
                            pSendBuff++;
                            //  Uint32ToHex.Data = Battery.Battery;
                            *pSendBuff = Battery.Battery;
                            pSendBuff++;
                            *pSendBuff = Battery.Battery >> 8;
                            pSendBuff++;
                            *pSendBuff = rfidData.ID[11];
                            pSendBuff++;
                            *pSendBuff = rfidData.ID[10];
                            pSendBuff++;
                            *pSendBuff = rfidData.ID[9];
                            pSendBuff++;
                            *pSendBuff = rfidData.ID[8];
                            pSendBuff++;

                            //Battery.
                            send( 0, buffer1, pSendBuff - buffer1);
                        }
                        break;
                    case 2:
                        p++;
                        if( len == 17 )
                        {
                            pSendBuff = buffer1;
                            for( int i = 0; i < 4; i++ )
                            {
                                FloatToHex.Hex[i] = *p; //buffer[5+i];
                                p++;
                            }
                            SetMaxSpeed( FloatToHex.Data, FloatToHex.Data );
                            Uint32ToHex.Data = 0xaaaa5555;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            Uint32ToHex.Data = PackID;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            *pSendBuff = 2;
                            pSendBuff++;
                            *pSendBuff = 0;
                            pSendBuff++;
                            send( 0, buffer1, pSendBuff - buffer1 );
                        }

                        break;
                    case 3:
                        p++;
                        if( len == 17 )
                        {
                            pSendBuff = buffer1;
                            for( int i = 0; i < 4; i++ )
                            {
                                FloatToHex.Hex[i] = *p;
                                p++;
                            }
                            xPosition = FloatToHex.Data;
                            for( int i = 0; i < 4; i++ )
                            {
                                FloatToHex.Hex[i] = *p;
                                p++;
                            }
                            yPosition = FloatToHex.Data;
                            SetPositionMoveTo( xPosition, yPosition );
                            Uint32ToHex.Data = 0xaaaa5555;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            Uint32ToHex.Data = PackID;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            *pSendBuff = 3;
                            pSendBuff++;
                            *pSendBuff = 0;
                            pSendBuff++;
                            send( 0, buffer1, pSendBuff - buffer1 );
                        }

                        break;
                    case 5:
                        p++;
                        if( len == 10 )
                        {
                            pSendBuff = buffer1;
                            Uint32ToHex.Data = 0xaaaa5555;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            Uint32ToHex.Data = PackID;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            *pSendBuff = 5;
                            pSendBuff++;
                            *pSendBuff = 0;
                            pSendBuff++;
                            if( *p )
                            {
                                ChargeQueDef chargecmd;
                                chargecmd.cmd = 1;
                                if( ChargeOperatingQue )
                                {
                                    xQueueSend( ChargeOperatingQue, &chargecmd, 10 );
                                }
                            }
                            else
                            {
                                ChargeQueDef chargecmd;
                                chargecmd.cmd = 0;
                                if( ChargeOperatingQue )
                                {
                                    xQueueSend( ChargeOperatingQue, &chargecmd, 10 );
                                }
                            }
                            send( 0, buffer1, pSendBuff - buffer1 );
                        }

                        break;
                    case 7:
                        p++;
                        if( len == 5 )
                        {
                            //  ClearError(1, 10 );
                            //  ClearError(4, 10 );
                            SetMoveToSelf();
                        }
                        break;
                    case 8:
                        if( DebugQue )
                        {
                            msg.buff[0] = 0;
                            if( HAL_GPIO_ReadPin( IN_X1_Port, IN_X1_Pin ) ) // x+
                            {
                                sprintf( msg.buff + strlen( msg.buff), "X+\t");
                            }
                            if( HAL_GPIO_ReadPin( IN_X2_Port, IN_X2_Pin ) ) // x-
                            {
                                sprintf( msg.buff + strlen( msg.buff), "X-\t");
                            }
                            if( HAL_GPIO_ReadPin( IN_Y1_Port, IN_Y1_Pin ) ) //Y+
                            {
                                sprintf( msg.buff + strlen( msg.buff), "Y+\t");
                            }
                            if( HAL_GPIO_ReadPin( IN_Y2_Port, IN_Y2_Pin ) ) //Y-
                            {
                                sprintf( msg.buff + strlen( msg.buff), "Y-\t");
                            }
                            sprintf( msg.buff + strlen( msg.buff), "\r\n");
                            DEBUG_OUT( msg.buff );
                        }
                        break;
                    case 9:
                        GetNextPiont();
                        break;
                    case 10:
                        p++;
                        if( len == 14 )
                        {
                            pSendBuff = buffer1;
                            Uint32ToHex.Data = 0xaaaa5555;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            Uint32ToHex.Data = PackID;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            *pSendBuff = 10;
                            pSendBuff++;
                            if( 1 )
                            {
                                int op = *p;
                                p++;
                                for( int i = 0; i < 4; i++ )
                                {
                                    Uint32ToHex.Hex[i] = *p;
                                    p++;
                                }
                                BeltOperatingConfig( (int)op, Uint32ToHex.Data );

                            }
                            *pSendBuff = 0;
                            pSendBuff++;
                            send( 0, buffer1, pSendBuff - buffer1 );
                        }
                        break;
                    case 11:
                        p++;
                        if( len == 10 )
                        {
                            pSendBuff = buffer1;
                            if( *p == 0 )
                            {
                                SwitchXY( 1 );
                            }
                            else
                            {
                                SwitchXY( 0 );
                            }
                            Uint32ToHex.Data = 0xaaaa5555;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            Uint32ToHex.Data = PackID;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            *pSendBuff = 11;
                            pSendBuff++;
                            *pSendBuff = 0;
                            pSendBuff++;
                            send( 0, buffer1, pSendBuff - buffer1 );
                        }
                        break;
                    case 12:
                    {
                        GPIO_TypeDef * INPort[] = {IN_1_GPIO_Port, IN_2_GPIO_Port, IN_3_GPIO_Port,IN_4_GPIO_Port, IN_5_GPIO_Port, IN_6_GPIO_Port, IN_7_GPIO_Port, IN_8_GPIO_Port, IN_9_GPIO_Port, IN_10_GPIO_Port, IN_11_GPIO_Port, IN_12_GPIO_Port, IN_13_GPIO_Port, IN_14_GPIO_Port, IN_15_GPIO_Port, IN_16_GPIO_Port, IN_17_GPIO_Port, IN_18_GPIO_Port, IN_19_GPIO_Port, IN_20_GPIO_Port };
                        uint16_t inpIN[] = {IN_1_Pin, IN_2_Pin, IN_3_Pin, IN_4_Pin, IN_5_Pin, IN_6_Pin, IN_7_Pin, IN_8_Pin, IN_9_Pin, IN_10_Pin, IN_11_Pin, IN_12_Pin, IN_13_Pin, IN_14_Pin, IN_15_Pin, IN_16_Pin, IN_17_Pin, IN_18_Pin, IN_19_Pin, IN_20_Pin };
                        msg.buff[0] = 0;
                        for( int i = 0; i < 20; i++ )
                        {
                            if( HAL_GPIO_ReadPin( INPort[i], inpIN[i] ) )
                            {
                                sprintf( msg.buff, "IN%d:on\t", i + 1 );
                            }
                            else
                            {
                                sprintf( msg.buff, "IN%d:off\t", i + 1 );
                            }
                            DEBUG_OUT( msg.buff );
                        }
                    }
                    break;
                    case 13:
                        if( len == 9 )
                        {
                            NVIC_SystemReset();
                        }
                        break;
                    case 14:
                        if( len == 9 )
                        {
                            cancelNavigate();
                            pSendBuff = buffer1;
                            Uint32ToHex.Data = 0xaaaa5555;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            Uint32ToHex.Data = PackID;
                            for( int i = 0; i < 4; i++ )
                            {
                                *pSendBuff = Uint32ToHex.Hex[i];
                                pSendBuff++;
                            }
                            *pSendBuff = 14;
                            pSendBuff++;
                            *pSendBuff = 0;
                            pSendBuff++;
                            send( 0, buffer1, pSendBuff - buffer1 );
                        }
                        break;
                    }
                }

            }
            break;
        case SOCK_CLOSE_WAIT:
            close(0);
            break;
        case SOCK_CLOSED:
            socket(0,Sn_MR_TCP,local_port,Sn_MR_ND);
            break;
        default:
            break;
        }

        switch(getSn_SR(1))
        {
        case SOCK_INIT:
            listen(1);
            break;
        case SOCK_ESTABLISHED:
            if(getSn_IR(1) & Sn_IR_CON)
            {
                setSn_IR(1, Sn_IR_CON);
            }
            len1=getSn_RX_RSR(1);
            if(len1>0)
            {
                if( len1 > sizeof( buffer1 ) )
                {
                    recv( 1, buffer1, sizeof( buffer1 ) );
                }
                else
                {
                    recv(1,buffer1,len1);
                }
                if( len1 > 9 )
                {

                    for( int i = 0; i < 4; i++ )
                    {
                        Uint32ToHex.Hex[i] = buffer1[i];
                    }
                    if( Uint32ToHex.Data == 0xaaaa5555 )
                    {
                        
                    }
                }
                if( buffer1[0] == 0xaa )
                {
                    HAL_GPIO_WritePin( OUT_5_GPIO_Port, OUT_5_Pin, GPIO_PIN_SET ) ;
                }
            }
            if( DebugQue )
            {
                if( xQueueReceive( DebugQue, &msg, 0 ) == pdPASS )
                {
                    send( 1, msg.buff, strlen( msg.buff ) );
                }
            }
            break;
        case SOCK_CLOSE_WAIT:
            close(1);
            break;
        case SOCK_CLOSED:
            socket(1,Sn_MR_TCP,local_port+1,Sn_MR_ND);
            break;
        default:
            break;
        }
    }
}
