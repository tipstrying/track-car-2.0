extern "C"
{
#include "cmsis_os.h"
#include "gpio.h"
#include "spi.h"
#include "w5500.h"
#include "socket.h"
#include "string.h"
#include "usart.h"
#include "tim.h"
#include "ff.h"
#include "rtc.h"

#include "app.h"
#include "Motor.h"
#include "stdio.h"
#include "queue.h"
    /* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"
    void CLITask(void const *parment);
    void protocolRun(void const *para);
    void vRegisterCLICommands(void);
    void startCLITask();
    int isPackOnCar();
}
#include "fifo.h"
#include "hardware.h"
#include "math.h"
#include "buildTime.h"
#include "httpServer.h"
#include "cmsis_os.h"
#include <stdarg.h>
#include "cmsis_armcc.h"

#define DEBUGENABLE 1
#define MAX_SOCK_NUM 8

typedef struct _CONFIG_MSG
{
    uint8_t op[4]; //header: FIND;SETT;FACT...
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

void spi4readwritebyte(uint8_t write)
{
    HAL_SPI_Transmit(&hspi4, &write, 1, 100);
}
uint8_t spi4readbyte(void)
{
    uint8_t bytes;
    HAL_SPI_Receive(&hspi4, &bytes, 1, 100);
    return bytes;
}
void spi4select()
{
    HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_RESET);
}
void spi4deselect()
{
    HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_SET);
}

#define DATA_BUF_SIZE 2048

uint8_t RX_BUF[DATA_BUF_SIZE];
uint8_t TX_BUF[DATA_BUF_SIZE];
uint8_t socknumlist[] = {4, 5, 6, 7};
#define MAX_HTTPSOCK 4

// 返回结构体示例
CONFIG_MSG SetNetWorkParment(void)
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
    networkconfig.lip[2] = 0;
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
    return networkconfig;
}

static FifoClass debugFifoBuff;

static int sockertID[MAX_SOCK_NUM - MAX_HTTPSOCK] = {0};
typedef int (*fun_ptr)(int);
typedef struct
{
    struct
    {
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
    BaseType_t lastUseTime;
    BaseType_t timeOut;
} networkDef;

networkDef socketServer[4];
static CONFIG_MSG networkconfig = SetNetWorkParment();
void socketServer_run(int i, FifoClass *in, FifoClass *out, uint16_t port, fun_ptr func1, fun_ptr func2, int FD);
int createSocket(FifoClass *in, FifoClass *out, uint16_t port, fun_ptr onCreatep, fun_ptr onClosep, BaseType_t timeout);
static int serverConnectOk(int ip)
{
    debugOut(0, "[\t%d] Server connect ok\r\n", osKernelSysTick());
}
static int serverDisconnect(int ip)
{
    if( ip == 1 )
    {
        debugOut(0, "[\t%d] Server disconnect\r\n", osKernelSysTick());
    }
    else
    {
        debugOut(0, "[\t%d] Server disconnect by timeout\r\n", osKernelSysTick());
    }
}

void W5500Task(void const *par)
{
    uint8_t txsize[MAX_SOCK_NUM] = {2, 2, 2, 2, 2, 2, 2, 2};
    uint8_t rxsize[MAX_SOCK_NUM] = {2, 2, 2, 2, 2, 2, 2, 2};
    uint8_t ip[4];

    for (int i = 0; i < 5; i++)
    {
        HAL_GPIO_WritePin(INTEL_REST_GPIO_Port, INTEL_REST_Pin, GPIO_PIN_RESET);
        osDelay(10);
        HAL_GPIO_WritePin(INTEL_REST_GPIO_Port, INTEL_REST_Pin, GPIO_PIN_SET);
        osDelay(10);
    }

    for (int i = 0; i < 4; i++)
    {
        socketServer[i].use = false;
    }

    union
    {
        uint8_t Hex[4];
        uint32_t Data;
    } u32ToHex;

    /*
        u32ToHex.Data = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
        int sum = 0;
        sum += u32ToHex.Hex[0];
        sum += u32ToHex.Hex[1];

        if (u32ToHex.Hex[2] == sum)
        {
            if (sum != 0)
            {
                networkconfig.lip[2] = u32ToHex.Hex[0];
                networkconfig.lip[3] = u32ToHex.Hex[1];
            }
        }

        u32ToHex.Data = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
        sum = 0;
        sum += u32ToHex.Hex[0];
        sum += u32ToHex.Hex[1];

        if (sum == u32ToHex.Hex[2])
        {
            networkconfig.mac[4] = u32ToHex.Hex[0];
            networkconfig.mac[5] = u32ToHex.Hex[1];
        }
    */
    //    reg_wizchip_cris_cbfunc(  vPortEnterCritical, vPortExitCritical);
    reg_wizchip_spi_cbfunc(spi4readbyte, spi4readwritebyte);
    reg_wizchip_cs_cbfunc(spi4select, spi4deselect);
    wizchip_init(txsize, rxsize);
    setSHAR(networkconfig.mac);
    setSUBR(networkconfig.sub);
    setGAR(networkconfig.gw);
    setSIPR(networkconfig.lip);
    setRTR(2000);
    osDelay(10);
    setRCR(3);
    osDelay(10);
    httpServer_init(TX_BUF, RX_BUF, MAX_HTTPSOCK, socknumlist);
    startCLITask();
    static FifoClass dataIn, dataOut;
    createSocket(&dataIn, &dataOut, 8802, serverConnectOk, serverDisconnect, 3000);

    for (;;)
    {
        osDelay(1);
        getSIPR(ip);

        if ((ip[0] != networkconfig.lip[0]) || (ip[1] != networkconfig.lip[1]) || (ip[2] != networkconfig.lip[2]) || (ip[3] != networkconfig.lip[3]))
        {
            setSHAR(networkconfig.mac);
            setSUBR(networkconfig.sub);
            setGAR(networkconfig.gw);
            setSIPR(networkconfig.lip);
            setRTR(2000);
            setRCR(3);
        }
        else
        {
            for (int i = 0; i < MAX_HTTPSOCK; i++)
            {
                httpServer_run(i);
            }

            for (int i = 0; i < MAX_SOCK_NUM - MAX_HTTPSOCK; i++)
            {
                if (socketServer[i].use)
                {
                    if (socketServer[i].close)
                    {
                        close(socketServer[i].socketID);
                    }

                    socketServer_run(socketServer[i].socketID, socketServer[i].streamIn, socketServer[i].streamOut, socketServer[i].netPara.port, socketServer[i].onConnect, socketServer[i].onClose, i);
                }
            }
        }
    }
}

/**********************************************************************************************************************************************************
CRC16
********************************************************************************************************************************************************/
unsigned short CRC16_MODBUS(unsigned char *buff, int len)
{
    unsigned short tmp = 0xffff;
    unsigned short ret1 = 0;

    for (int n = 0; n < len; n++)
    {
        /*ՋԦք6 -- ҪУҩքλ˽Ϊ6ٶ*/
        tmp = buff[n] ^ tmp;

        for (int i = 0; i < 8; i++)
        {
            /*ՋԦք8 -- ָÿһٶchar`эԖ8bitìÿbitּҪԦm*/
            if (tmp & 0x01)
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
int makePack(uint8_t *packBuf, uint64_t packIndex, uint16_t packCmd, uint8_t packErrorCode, int packLen, uint8_t *data)
{
    union
    {
        uint64_t Data;
        uint8_t Hex[8];
    } Uint64ToHex;
    union
    {
        uint16_t Data;
        uint8_t Hex[2];
    } Uint16ToHex;
    union
    {
        uint32_t Data;
        uint8_t Hex[4];
    } Uint32ToHex;
    uint8_t *buff;
    buff = packBuf;
    *packBuf++ = 'l';
    *packBuf++ = 's';
    *packBuf++ = 'p';
    *packBuf++ = 'h';
    Uint64ToHex.Data = packIndex;

    for (int i = 0; i < 8; i++)
    {
        *packBuf++ = Uint64ToHex.Hex[7 - i];
    }

    Uint16ToHex.Data = packCmd;

    for (int i = 0; i < 2; i++)
    {
        *packBuf++ = Uint16ToHex.Hex[1 - i];
    }

    *packBuf++ = packErrorCode;
    Uint32ToHex.Data = packLen;

    for (int i = 0; i < 4; i++)
    {
        *packBuf++ = Uint32ToHex.Hex[3 - i];
    }

    for (int i = 0; i < packLen; i++)
    {
        *packBuf++ = *data++;
    }

    unsigned short crc = CRC16_MODBUS(buff, packBuf - buff);
    Uint16ToHex.Data = crc;
    *packBuf++ = Uint16ToHex.Hex[0];
    *packBuf++ = Uint16ToHex.Hex[1];
    return packBuf - buff;
}
/***********************************************************************************************************************************************************
解包
*/
////////

int decodePack(uint8_t *packBuff, int buffLen, uint64_t *packIndex, uint16_t *packCmd, uint8_t *packErrorCode, int *packLen, uint8_t *data)
{
    uint16_t crc = 0;
    union
    {
        uint32_t Data;
        uint8_t Hex[4];
    } Uint32ToHex;
    union
    {
        uint64_t Data;
        uint8_t Hex[8];
    } Uint64ToHex;
    union
    {
        uint16_t Data;
        uint8_t Hex[2];
    } Uint16ToHex;
    uint8_t *packPiont = packBuff;

    for (int i = 0; i < 4; i++)
    {
        Uint32ToHex.Hex[i] = *packPiont++;
    }

    if (Uint32ToHex.Data != 0x6870736c)
    {
        return -1;
    }

    for (int i = 0; i < 8; i++)
    {
        Uint64ToHex.Hex[7 - i] = *packPiont++;
    }

    if (packIndex)
    {
        *packIndex = Uint64ToHex.Data;
    }

    Uint16ToHex.Hex[1] = *packPiont++;
    Uint16ToHex.Hex[0] = *packPiont++;

    if (packCmd)
    {
        *packCmd = Uint16ToHex.Data;
    }

    if (packErrorCode)
    {
        *packErrorCode = *packPiont++;
    }

    for (int i = 0; i < 4; i++)
    {
        Uint32ToHex.Hex[3 - i] = *packPiont++;
    }

    if (packLen)
    {
        *packLen = Uint32ToHex.Data;
    }

    if (*packLen + 22 > 500)
    {
        return -1;
    }

    if (*packLen + 21 > buffLen)
    {
        return pdFALSE;
    }

    for (int i = 0; i < *packLen; i++)
    {
        data[i] = *packPiont++;
    }

    crc = CRC16_MODBUS(packBuff, 19 + *packLen);
    Uint16ToHex.Hex[0] = *(packBuff + 19 + *packLen);
    Uint16ToHex.Hex[1] = *(packBuff + 20 + *packLen);

    if (crc == Uint16ToHex.Data)
    {
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}
/**** ****************************************************************************************************************************************************
调度通信线程
* *********************************************************************************************************************************************************/
int IsMotorAlarm();
extern int switchReach;




/*void protocolRun(void const *para)
{
    static FifoClass dataIn, dataOut;
    uint64_t packIndex = -1, packIndexBak = -1;
    uint16_t packCMD = 0;
    uint8_t errorCode;
    int PackLen;
    union
    {
        uint8_t Hex[4];
        int Data;
    } i32ToHex;
    createSocket(&dataIn, &dataOut, 8802, serverConnectOk, serverDisconnect, 3000);

    for (;;)
    {
        osDelay(1);
    }
}*/
/* ******************************************************************************
socket 服务线程
*************************************************************************************/
void socketServer_run(int i, FifoClass *in, FifoClass *out, uint16_t port, fun_ptr func1, fun_ptr func2, int FD)
{
    static uint8_t buffer[2048];
    int len = 0;
    uint8_t buff[500];
    uint8_t data[100];
    uint64_t packIndex = -1, packIndexBak = -1;
    uint16_t packCMD = 0;
    uint8_t errorCode;
    int PackLen;
    NavigationOperationStd navData;
    union
    {
        uint8_t Hex[4];
        int Data;
    } i32ToHex;

    switch (getSn_SR(i))
    {
        case SOCK_INIT:
            socketServer[FD].lastUseTime = 0;
            listen(i);

            if (func1)
            {
                func1(0);
            }

            break;

        case SOCK_ESTABLISHED:
            if (getSn_IR(i) & Sn_IR_CON)
            {
                setSn_IR(i, Sn_IR_CON);
            }

            len = getSn_RX_RSR(i);

            if (len > 0)
            {
                socketServer[FD].lastUseTime = osKernelSysTick();
                recv(i, buffer, len);

                if (in)
                {
                    in->pushData(buffer, len);
                }
            }

            if (out)
            {
                len = out->available();

                if (len)
                {
                    out->popData(buffer, len);
                    send(i, buffer, len);
                }
            }

            if( socketServer[FD].lastUseTime <= osKernelSysTick() )
            {
                if( socketServer[FD].lastUseTime == 0 )
                {
                    socketServer[FD].lastUseTime = osKernelSysTick();
                }

                if( osKernelSysTick() - socketServer[FD].lastUseTime > socketServer[FD].timeOut )
                {
                    if (socketServer[i].streamIn)
                    {
                        socketServer[i].streamIn->clean();
                    }

                    if (socketServer[i].streamOut)
                    {
                        socketServer[i].streamOut->clean();
                    }

                    close(i);

                    if (func2)
                    {
                        func2(2);
                    }

                    close(i);
                }
            }
            else
            {
                socketServer[FD].lastUseTime = osKernelSysTick();
            }

            if(port == 8802)
            {
                if (in->available() >= 21)
                {
                    in->pickData(buff, 21);

                    if (decodePack(buff, 21, &packIndex, &packCMD, &errorCode, &PackLen, data) >= 0)
                    {
                        if (in->available() >= PackLen + 21)
                        {
                            PackLen = in->popData(buff, PackLen + 21);

                            if (decodePack(buff, PackLen, &packIndex, &packCMD, &errorCode, &PackLen, data) == pdTRUE)
                            {
                                /*
                                解包成功
                                */
                                debugOut(0, (char *)"[\t%d] Decode pack ok: index->%lld,cmd->%d,packLen->%d,date:", osKernelSysTick(), packIndex, packCMD, PackLen);

                                for (int i = 0; i < PackLen; i++)
                                {
                                    debugOut(0, (char *)"0X%02X ", data[i]);
                                }

                                debugOut(0, (char *)"\r\n");

                                if( /* packIndexBak == packIndex */ 0 )
                                {
                                    PackLen = makePack(buff, packIndex, packCMD + 10000, 0, 0, NULL);
                                    out->pushData(buff, PackLen);
                                }
                                else
                                {
                                    switch (packCMD)
                                    {
                                        case 2001:
                                            if (1)
                                            {
                                                // Enum_QueryCarInfo
                                                float pos, speed;
                                                int posInt;
                                                uint8_t status;
                                                uint8_t batVol;
                                                status = 0;
                                                union
                                                {
                                                    uint8_t Hex[4];
                                                    int Data;
                                                } i32ToHex;
                                                union
                                                {
                                                    uint8_t Hex[2];
                                                    uint16_t Data;
                                                } u16ToHex;
                                                GetPosition(&pos);
                                                GetSpeed(&speed);
                                                i32ToHex.Data = (int)pos;
                                                u16ToHex.Data = (uint16_t)speed;
                                                status = GetMotionStatus();
                                                batVol = (uint8_t)(Battery.Voltage / 1000);

                                                for (int i = 0; i < 4; i++)
                                                {
                                                    data[i] = i32ToHex.Hex[3 - i];
                                                }

                                                data[4] = u16ToHex.Hex[1];
                                                data[5] = u16ToHex.Hex[0];
                                                data[6] = status;
                                                data[7] = batVol;
                                                PackLen = makePack(buff, packIndex, 12001, 0, 8, data);
                                                out->pushData(buff, PackLen);
                                            }

                                            break;

                                        case 2002:
                                            // Enum_SetMaxSpeed
                                            navData.cmd = Enum_SetMaxSpeed;
                                            navData.Data.speedTo = data[0] * 255 + data[1];

                                            if (xQueueSend(NavigationOperationQue, &navData, 1000) == pdPASS)
                                            {
                                                PackLen = makePack(buff, packIndex, 12002, 0, 0, NULL);
                                                out->pushData(buff, PackLen);
                                            }
                                            else
                                            {
                                                PackLen = makePack(buff, packIndex, 12002, 1, 0, NULL);
                                                out->pushData(buff, PackLen);
                                            }

                                            break;

                                        case 2003:
                                            if (1)
                                            {
                                                int rValue = 0;

                                                if( packIndex == packIndexBak )
                                                {
                                                    debugOut(0, "[\t%d] <TCP> {navigation} same index find index->%lld, ignore it\r\n", osKernelSysTick(), packIndex );
                                                    rValue = 0;
                                                }
                                                else
                                                {
                                                    packIndexBak = packIndex;
                                                    uint8_t trails = data[2];

                                                    for (int i = 0; i < trails; i++)
                                                    {
                                                        if (rValue)
                                                        {
                                                            break;
                                                        }

                                                        for (int j = 0; j < 4; j++)
                                                        {
                                                            i32ToHex.Hex[3 - j] = data[3 + j + i * 7];
                                                        }

                                                        navData.cmd = Enum_SendNavigation;
                                                        navData.Data.posTo = i32ToHex.Data;

                                                        if (xQueueSend(NavigationOperationQue, &navData, 100) != pdPASS)
                                                        {
                                                            if (!rValue)
                                                            {
                                                                rValue = 0;
                                                            }
                                                        }

                                                        if (data[7 + i * 7] + data[8 + i * 7] != 0)
                                                        {
                                                            navData.cmd = Enum_sendOperation;
                                                            navData.Data.speedTo = data[7 + i * 7] * 255 + data[8 + i * 7];
                                                            navData.Data.op = 1;
                                                            navData.Data.posTo = i32ToHex.Data;

                                                            if (xQueueSend(NavigationOperationQue, &navData, 100) == pdPASS)
                                                            {
                                                                if (!rValue)
                                                                {
                                                                    rValue = 0;
                                                                }
                                                            }
                                                        }

                                                        if (data[9 + i * 7])
                                                        {
                                                            navData.cmd = Enum_sendOperation;
                                                            navData.Data.posTo = i32ToHex.Data;
                                                            navData.Data.op = data[9 + i * 7] + 1;

                                                            if (xQueueSend(NavigationOperationQue, &navData, 100) == pdPASS)
                                                            {
                                                                if (!rValue)
                                                                {
                                                                    rValue = 0;
                                                                }
                                                            }
                                                        }
                                                    }
                                                }

                                                if (rValue)
                                                {
                                                    PackLen = makePack(buff, packIndex, 12003, 1, 0, 0);
                                                }
                                                else
                                                {
                                                    PackLen = makePack(buff, packIndex, 12003, 0, 0, 0);
                                                }

                                                out->pushData(buff, PackLen);
                                            }

                                            break;

                                        case 2004:
                                            if (1)
                                            {
                                                int rValue = 0;
                                                navData.cmd = Enum_PauseNavigation;
                                                navData.Data.op = 1;

                                                if (xQueueSend(NavigationOperationQue, &navData, 100) == pdPASS)
                                                {
                                                    rValue = 0;
                                                }
                                                else
                                                {
                                                    rValue = 1;
                                                }

                                                if (rValue)
                                                {
                                                    PackLen = makePack(buff, packIndex, 12004, 1, 0, 0);
                                                }
                                                else
                                                {
                                                    PackLen = makePack(buff, packIndex, 12004, 0, 0, 0);
                                                }

                                                out->pushData(buff, PackLen);
                                            }

                                            break;

                                        case 2005:
                                            if (1)
                                            {
                                                int rValue = 0;
                                                navData.cmd = Enum_PauseNavigation;
                                                navData.Data.op = 0;

                                                if (xQueueSend(NavigationOperationQue, &navData, 100) == pdPASS)
                                                {
                                                    rValue = 0;
                                                }
                                                else
                                                {
                                                    rValue = 1;
                                                }

                                                if (rValue)
                                                {
                                                    PackLen = makePack(buff, packIndex, 12005, 1, 0, 0);
                                                }
                                                else
                                                {
                                                    PackLen = makePack(buff, packIndex, 12005, 0, 0, 0);
                                                }

                                                out->pushData(buff, PackLen);
                                            }

                                            break;

                                        case 2006:
                                            if (1)
                                            {
                                                int rValue = 0;
                                                navData.cmd = Enum_CancelNavigation;
                                                navData.Data.op = 1;

                                                if (xQueueSend(NavigationOperationQue, &navData, 100) == pdPASS)
                                                {
                                                    rValue = 0;
                                                }
                                                else
                                                {
                                                    rValue = 1;
                                                }

                                                if (rValue)
                                                {
                                                    PackLen = makePack(buff, packIndex, 12006, 1, 0, 0);
                                                }
                                                else
                                                {
                                                    PackLen = makePack(buff, packIndex, 12006, 0, 0, 0);
                                                }

                                                out->pushData(buff, PackLen);
                                            }

                                            break;

                                        case 2007:
                                            if (1)
                                            {
                                                if (PackLen == 1)
                                                {
                                                    int rValue = 0;

                                                    if (!data[0])
                                                    {
                                                        navData.Data.op = 0;
                                                    }
                                                    else
                                                    {
                                                        navData.Data.op = 1;
                                                    }

                                                    navData.cmd = Enum_PushThing;

                                                    if (xQueueSend(NavigationOperationQue, &navData, 100) == pdPASS)
                                                    {
                                                        rValue = 0;
                                                    }
                                                    else
                                                    {
                                                        rValue = 1;
                                                    }

                                                    if (rValue)
                                                    {
                                                        PackLen = makePack(buff, packIndex, 12007, 1, 0, 0);
                                                    }
                                                    else
                                                    {
                                                        PackLen = makePack(buff, packIndex, 12007, 0, 0, 0);
                                                    }

                                                    out->pushData(buff, PackLen);
                                                }
                                                else
                                                {
                                                    PackLen = makePack(buff, packIndex, 12007, 1, 0, 0);
                                                    out->pushData(buff, PackLen);
                                                }
                                            }

                                            break;

                                        case 2008:
                                            if (1)
                                            {
                                                uint8_t isThingOnCar = isPackOnCar();
                                                PackLen = makePack(buff, packIndex, 12008, 0, 1, &isThingOnCar);
                                                out->pushData(buff, PackLen);
                                            }

                                            break;

                                        case 2010:
                                            if (1)
                                            {
                                                ClearMotorAlarm();
                                                PackLen = makePack(buff, packIndex, 12010, 0, 0, 0);
                                                out->pushData(buff, PackLen);
                                            }

                                            break;

                                        case 2011:
                                            if (1)
                                            {
                                                if (PackLen == 4)
                                                {
                                                    for (int i = 0; i < 4; i++)
                                                    {
                                                        i32ToHex.Hex[i] = data[3 - i];
                                                    }

                                                    navData.cmd = Enum_SetZeroPosition;
                                                    navData.Data.posTo = i32ToHex.Data;
                                                    xQueueSend(NavigationOperationQue, &navData, 100);
                                                    PackLen = makePack(buff, packIndex, 12011, 0, 0, 0);
                                                    out->pushData(buff, PackLen);
                                                }
                                            }

                                            break;

                                        case 2013:
                                            if (1)
                                            {
                                                if (PackLen == 1)
                                                {
                                                    if (!data[0])
                                                    {
                                                        navData.Data.op = 0;
                                                    }
                                                    else
                                                    {
                                                        navData.Data.op = 1;
                                                    }

                                                    navData.cmd = Enum_PullThing;

                                                    if (xQueueSend(NavigationOperationQue, &navData, 100) == pdPASS)
                                                    {
                                                        PackLen = makePack(buff, packIndex, 12013, 0, 0, 0);
                                                        out->pushData(buff, PackLen);
                                                    }
                                                    else
                                                    {
                                                        PackLen = makePack(buff, packIndex, 12013, 1, 0, 0);
                                                        out->pushData(buff, PackLen);
                                                    }
                                                }
                                                else
                                                {
                                                    PackLen = makePack(buff, packIndex, 12013, 1, 0, 0);
                                                    out->pushData(buff, PackLen);
                                                }
                                            }

                                            break;

                                        case 2016:
                                            if (data[0] == 1)
                                            {
                                                HAL_GPIO_WritePin(OUT_9_GPIO_Port, OUT_9_Pin, GPIO_PIN_SET);
                                            }
                                            else
                                            {
                                                HAL_GPIO_WritePin(OUT_9_GPIO_Port, OUT_9_Pin, GPIO_PIN_RESET);
                                            }

                                            PackLen = makePack(buff, packIndex, 12016, 0, 0, 0);
                                            out->pushData(buff, PackLen);
                                            break;

                                        case 2017:
                                            if (1)
                                            {
                                                uint8_t isChargeKeyOpen = HAL_GPIO_ReadPin(OUT_9_GPIO_Port, OUT_9_Pin);
                                                PackLen = makePack(buff, packIndex, 12017, 0, 1, &isChargeKeyOpen);
                                                out->pushData(buff, PackLen);
                                            }

                                            break;

                                        case 2018:
                                            /*
                                            navData.cmd = 5;
                                            navData.Data.op = 1;
                                            xQueueSend( NavigationOperationQue, &navData, 100 );
                                            */
                                            PackLen = makePack(buff, packIndex, 12018, 0, 0, 0);
                                            out->pushData(buff, PackLen);
                                            break;

                                        case 2019:
                                            navData.cmd = Enum_SetInOutSwitch;

                                            if (data[0] == 1)
                                            {
                                                navData.Data.op = 1;
                                            }
                                            else if (data[0] == 2)
                                            {
                                                navData.Data.op = 0;
                                            }
                                            else
                                            {
                                                PackLen = makePack(buff, packIndex, 12019, 1, 0, 0);
                                                out->pushData(buff, PackLen);
                                                break;
                                            }

                                            xQueueSend(NavigationOperationQue, &navData, 100);
                                            PackLen = makePack(buff, packIndex, 12019, 0, 0, 0);
                                            out->pushData(buff, PackLen);
                                            break;

                                        case 2020:
                                            navData.cmd = Enum_setHandSpeedMode;
                                            navData.Data.op = data[0];
                                            xQueueSend(NavigationOperationQue, &navData, 100);
                                            PackLen = makePack(buff, packIndex, 12020, 0, 0, 0);
                                            out->pushData(buff, PackLen);
                                            break;

                                        case 2021:
                                            if (PackLen == 4)
                                            {
                                                for (int i = 0; i < 4; i++)
                                                {
                                                    i32ToHex.Hex[i] = data[3 - i];
                                                }

                                                navData.cmd = Enum_SetHandSpeed;
                                                navData.Data.speedTo = i32ToHex.Data;
                                                xQueueSend(NavigationOperationQue, &navData, 10);
                                                PackLen = makePack(buff, packIndex, 12021, 0, 0, 0);
                                                out->pushData(buff, PackLen);
                                            }

                                            break;

                                        case 2022:
                                            if( 1 )
                                            {
                                                InOutSwitch in = getSwitchStatus();

                                                if( in == InOutSwitchIn )
                                                {
                                                    data[0] = 2;
                                                }
                                                else if( in == InOutSwitchOut )
                                                {
                                                    data[0] = 1;
                                                }
                                                else
                                                {
                                                    data[0] = 0;
                                                }

                                                PackLen = makePack( buff, packIndex, 12022, 0, 1, data );
                                                out->pushData(buff, PackLen );
                                            }

                                        default:
                                            break;
                                    }

                                    //packIndexBak = packIndex;
                                }
                            }
                        }
                    }
                    else
                    {
                        in->popData(buff, 1);
                    }
                }
            }

            break;

        case SOCK_CLOSE_WAIT:
            if (socketServer[i].streamIn)
            {
                socketServer[i].streamIn->clean();
            }

            if (socketServer[i].streamOut)
            {
                socketServer[i].streamOut->clean();
            }

            close(i);

            if (func2)
            {
                func2(1);
            }

            break;

        case SOCK_CLOSED:
            socket(i, Sn_MR_TCP, port, Sn_MR_ND);
            break;

        default:
            break;
    }
}

int getFreeSocketID()
{
    for (int i = 0; i < 4; i++)
    {
        if (!sockertID[i])
        {
            return i;
        }
    }

    return -1;
}
int getFreeSocketData()
{
    for (int i = 0; i < 4; i++)
    {
        if (!socketServer[i].use)
        {
            return i;
        }
    }

    return -1;
}
int createSocket(FifoClass * in, FifoClass * out, uint16_t port, fun_ptr onCreatep, fun_ptr onClosep, BaseType_t timeout)
{
    int socketDataid = getFreeSocketData();
    socketServer[socketDataid].socketID = getFreeSocketID();
    socketServer[socketDataid].streamIn = in;
    socketServer[socketDataid].streamOut = out;
    socketServer[socketDataid].netPara.port = port;
    socketServer[socketDataid].use = true;
    socketServer[socketDataid].close = false;
    socketServer[socketDataid].onConnect = onCreatep;
    socketServer[socketDataid].onClose = onClosep;
    sockertID[socketServer[socketDataid].socketID] = 1;
    socketServer[socketDataid].timeOut = timeout;
    return socketDataid;
}
/* ***************************************************************************************************/
// CLI Interface begin
static bool showBanner = true;
static int cliNetworkID = 0;
int cliSOcketConnect(int i)
{
    char buff[500];
    sprintf(buff, "Welcome to LS-RGV Command Line Interface IP:%d.%d.%d.%d\r\n%s", networkconfig.lip[0], networkconfig.lip[1], networkconfig.lip[2], networkconfig.lip[3], BUILDTIME);
    socketServer[cliNetworkID].streamOut->pushData((uint8_t *)buff, strlen(buff));
    socketServer[cliNetworkID].streamOut->pushData((uint8_t *)"\r\nLS-RGV $ ", sizeof("\r\nLS-RGV $ "));
}
void CLITask(void const * parment)
{
    BaseType_t xMoreDataToFollow;
    long lBytes;
    static char cInputString[100], cOutputString[1024];
    int CmdStringIndex = 0;
    static FifoClass stdinBuff;
    static FifoClass stdoutBuff;
    enum KEY_ACTION
    {
        KEY_NULL = 0,   /* NULL */
        CTRL_A = 1,     /* Ctrl+a */
        CTRL_B = 2,     /* Ctrl-b */
        CTRL_C = 3,     /* Ctrl-c */
        CTRL_D = 4,     /* Ctrl-d */
        CTRL_E = 5,     /* Ctrl-e */
        CTRL_F = 6,     /* Ctrl-f */
        CTRL_H = 8,     /* Ctrl-h */
        TAB = 9,        /* Tab */
        CTRL_K = 11,    /* Ctrl+k */
        CTRL_L = 12,    /* Ctrl+l */
        ENTER = 10,     /* Enter */
        CTRL_N = 14,    /* Ctrl-n */
        CTRL_P = 16,    /* Ctrl-p */
        CTRL_T = 20,    /* Ctrl-t */
        CTRL_U = 21,    /* Ctrl+u */
        CTRL_W = 23,    /* Ctrl+w */
        ESC = 27,       /* Escape */
        BACKSPACE = 127 /* Backspace */
    };
    cliNetworkID = createSocket(&stdinBuff, &stdoutBuff, 5002, cliSOcketConnect, NULL, portMAX_DELAY);
    createSocket(NULL, &debugFifoBuff, 5001, NULL, NULL, portMAX_DELAY);
    vRegisterCLICommands();
    stdinBuff.clean();
    union
    {
        char Hex[4];
        uint32_t Data;
    } u32ToHex;
    u32ToHex.Hex[0] = 'a';
    u32ToHex.Hex[1] = 'p';
    u32ToHex.Hex[2] = 'p';
    u32ToHex.Hex[3] = 'M';
    /*
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != u32ToHex.Data)
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, u32ToHex.Data);
    */
    CmdStringIndex = 0;

    if (*(uint32_t *)0x0800C000 != u32ToHex.Data)
    {
        while( HAL_FLASH_Unlock() != HAL_OK )
        {
            osDelay(1);
        }

        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t SectorError;
        EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
        EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
        EraseInitStruct.Sector = FLASH_SECTOR_3;
        EraseInitStruct.NbSectors = 1;

        while (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
        {
            osDelay(1);
        }

        while( HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, 0x0800C000, u32ToHex.Data ) != HAL_OK )
        {
            osDelay(1);
        }

        HAL_FLASH_Lock();
    }

    for (;;)
    {
        if (stdinBuff.available())
        {
            switch (stdinBuff.front())
            {
                case '\b':
                    if (CmdStringIndex > 0)
                    {
                        CmdStringIndex--;
                    }

                    break;

                case '\t':
                    break;

                default:
                    cInputString[CmdStringIndex] = stdinBuff.front();

                    if (CmdStringIndex < sizeof(cInputString) - 1)
                    {
                        CmdStringIndex++;
                    }

                    break;
            }

            stdinBuff.popData(NULL, 1);

            if (CmdStringIndex > 0)
            {
                if (cInputString[CmdStringIndex - 1] == '\n')
                {
                    //    lBytes = stdinBuff.popData( (uint8_t *)cInputString, stdinBuff.available() );
                    //  cInputString[lBytes] = 0;
                    if (cInputString[CmdStringIndex - 2] == '\r')
                    {
                        cInputString[CmdStringIndex - 2] = 0;
                    }

                    if (cInputString[CmdStringIndex - 1] == '\n')
                    {
                        cInputString[CmdStringIndex - 1] = 0;
                    }

                    do
                    {
                        xMoreDataToFollow = FreeRTOS_CLIProcessCommand((const char *)cInputString, cOutputString, (size_t)sizeof(cOutputString));
                        stdoutBuff.pushData((uint8_t *)cOutputString, strlen(cOutputString));
                    }
                    while (xMoreDataToFollow != pdFAIL);

                    memset(cInputString, 0, sizeof(cInputString));
                    CmdStringIndex = 0;
                    stdoutBuff.pushData((uint8_t *)"\r\nLS-RGV $ ", sizeof("\r\nLS-RGV $ "));
                }
            }
            else
            {
                osDelay(1);
            }
        }
        else
        {
            osDelay(1);
        }
    }
}
// CLI interfact end
/* *****************************************************************************************************************/
extern "C"
{
    int fputc(int c, FILE *f);
    int debugOut(int isISR, const char *fmt, ...);
    static int inHandlerMode (void);
}
static int inHandlerMode (void)
{
    return __get_IPSR() != 0;
}
int debugOut(int isISR, const char *fmt, ...)
{
    if (!inHandlerMode())
    {
        taskENTER_CRITICAL();
    }

    /*
    if ( !isISR )
    {
    taskENTER_CRITICAL();
    }
    */
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    /*
    if ( !isISR )
    taskEXIT_CRITICAL();
    */
    if (!inHandlerMode())
    {
        taskEXIT_CRITICAL();
    }

    return 0;
}
int fputc(int c, FILE * f)
{
#if DEBUGENABLE
    debugFifoBuff.pushData((uint8_t *)&c, 1);
#endif
}