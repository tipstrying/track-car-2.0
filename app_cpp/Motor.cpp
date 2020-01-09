#include "Motor.h"
#include "AGV_Parallel_Motion.h"

#include "canopensample.h"
#include "app.h"
#include "hardware.h"
#include "httpapi.h"
#include "battery.h"
#include "stdio.h"
#include "math.h"
#include "rtc.h"
#include "listRunTask.h"

#define MaxSpeed 2000

static float lastPosition;

#ifdef __cplusplus
extern "C"
{
void MotorTestTask(void const *parment);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
}
#endif

static int Encoder_Value;
static AGV_Parallel_Motion agv;
static float AGV_Pos;

AGV_Parallel_Motion *getAGVHandle()
{
    return &agv;
}

void getTS_CURVEPAR(float *k, float *j, float *a, float *v)
{
    agv.get_ts_curve(k, j, a, v);
    return;
}

void setTS_CURVEPAR(float k, float j, float a, float v)
{
    agv.set_ts_curve(k, j, a, v);
    return;
}

static struct
{
    int BootUp;
    bool CanDelay;
    bool CanRestDelay;
    bool EcodeDelay;
    bool SpeedDelay;
    bool alarm;
} MotionStatus;

static int BeltOperating = 0;
static int BeltOperatingTime = 0;
static bool BeltOperatingPause = false;
static double milages = 0;

int IsArriver()
{
    int status = agv.Motion_Status_Now;
    if (MotionStatus.BootUp == 2)
    {
        return -1;
    }
    else
    {
        if (MotionStatus.BootUp)
        {
            return 6;
        }
        else
        {
            switch (status)
            {
            case 0:
            case 5:
                return 0;
            case 1:
            case 2:
                return 1;
            case 4:
            case 8:
            case 9:
                return 2;
            case 3:
                return 3;
            case 6:
                return 4;
            case 7:
                return 5;
            default:
                return 3;
            }
        }
    }
}

float GetStop_Accuracy()
{
    return agv.Stop_Accuracy;
}
void GetRealTimePosition(float *x)
{
    *x = agv.AGV_Pos;
}

void GetSpeed(float *xSpeed)
{
    *xSpeed = agv.Request_Speed;
}
void SetMaxSpeed(float xSpeeed)
{
    //   agv.sSpeed_max = xSpeeed;
}
int SetSelfPosition(float X)
{
    if (MotionStatus.BootUp)
    {
        agv.AGV_Pos = X;
        AGV_Pos = agv.AGV_Pos;
        lastPosition = agv.AGV_Pos;
        MotionStatus.BootUp = 0;

        if (DebugCtrl.enableStartUp)
        {
            /*
            taskENTER_CRITICAL();
            {
                printf("[\t%d] Set Self Position Ok\r\n", osKernelSysTick());
            }
            taskEXIT_CRITICAL();
            */
            debugOut( 0, (char *)"[\t%d] Set Self Position Ok\r\n", osKernelSysTick());
        }
        if (MotionStatus.SpeedDelay)
        {
            if (DebugCtrl.enableStartUp)
            {
                /*
                taskENTER_CRITICAL();
                {
                    printf("[\t%d] Speed Up \r\n", osKernelSysTick());
                }
                taskEXIT_CRITICAL();
                */
                debugOut( 0, (char *)"[\t%d] Speed Up \r\n", osKernelSysTick());
            }
            MotionStatus.SpeedDelay = false;
        }
        return 0;
    }
    else
    {
        return 1;
    }
}

void GetNextPiont(float *X)
{
    *X = AGV_Pos;
}
void GetPosition( float * X )
{
    *X = agv.AGV_Pos;
}
void SetiEmergency(int S)
{
    if (S)
        agv.iEmergency = true;
    else
        agv.iEmergency = false;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    ;
}

bool CanTx(int ID, int iLength, char iArray[8])
{
    if (SendBuffToCan1((uint8_t *)&iArray[0], ID, iLength))
    {
        return false;
    }
    else
        return true;
}
bool CanRx(int *oID, int *oLength, char oArray[])
{
    extern QueueHandle_t can1Queue;
    can1DataStd can1Data;
    if (can1Queue)
    {
        if (xQueueReceive(can1Queue, &can1Data, 0) == pdTRUE)
        {
            *oID = can1Data.RxHeader.StdId;
            *oLength = can1Data.RxHeader.DLC;
            for (int i = 0; i < *oLength; i++)
            {
                oArray[i] = can1Data.Data[i];
            }
            if (DebugCtrl.enableCanRawData)
            {
                if (/* *oLength == 1 && oArray[0] == 0x7f */ 0)
                {
                    ;
                }
                else
                {
                    /*
                    taskENTER_CRITICAL();
                    {
                        printf("[\t%d] Can ID: %02X, DLC: %d Data: ", osKernelSysTick(), *oID, *oLength);
                        for (int i = 0; i < *oLength; i++)
                        {
                            printf("%02X ", oArray[i]);
                        }
                        printf("\r\n");
                    }
                    taskEXIT_CRITICAL();
                    */
                    debugOut( 0, (char *)"[\t%d] Can ID: %02X, DLC: %d Data: ", osKernelSysTick(), *oID, *oLength);
                    for (int i = 0; i < *oLength; i++)
                    {
                        debugOut(0, (char *)"%02X ", oArray[i]);
                    }
                    debugOut(0, (char *)"\r\n");
                }
            }
            return true;
        }
    }
    return false;
}
// static int canHeartBestCount[3] = {0, 0, 0};
void Rx_SDO_Commplate(int oID, int oIndex, char oSubindex, int oValue)
{
    switch (oID)
    {
    case 0x01:
        switch (oIndex)
        {
        case 0x6063:
            Encoder_Value = oValue;
            if (MotionStatus.EcodeDelay)
            {
            }
            MotionStatus.EcodeDelay = false;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void Rx_PDO_Commplate(int oID, int oIndex, char oSubindex, int oValue)
{
    static bool alarmOuted[2] = {false, false};
    switch (oID)
    {
    case 0x181:
        Encoder_Value = -oValue;
        if (MotionStatus.EcodeDelay)
        {
            if (DebugCtrl.enableStartUp)
            {
                /*
                taskENTER_CRITICAL();
                {
                    printf("[\t%d] Ecode 0 Up\r\n", osKernelSysTick());
                }
                taskEXIT_CRITICAL();
                */
                
            }
            MotionStatus.EcodeDelay = false;
        }
        break;

    default:
        break;
    }
}

void canHeartbeat(int oID, CANopenMaster::CANopenResponse::te_HeartBeat oStatus)
{
    switch (oID)
    {
    case 1:
        if (MotionStatus.CanDelay)
        {
            if (DebugCtrl.enableStartUp)
            {
                taskENTER_CRITICAL();
                {
                    printf("[\t%d] Motor 1 Up\r\n", osKernelSysTick());
                }
                taskEXIT_CRITICAL();
            }
        }
        MotionStatus.CanDelay = false;
        if (oStatus == 0x7f)
        {
            if (MotionStatus.CanRestDelay)
            {
                MotionStatus.CanRestDelay = false;
                MotionStatus.EcodeDelay = true;
                taskENTER_CRITICAL();
                {
                    printf("[\t%d] Motor 1 Rest OK\r\n", osKernelSysTick());
                }
                taskEXIT_CRITICAL();
            }
        }
        break;
    default:
        break;
    }
}

void cancelNavigate()
{
    agv.clearMotionStatusError();
}

double GetMilage(void)
{
    return milages;
}

static CANopenMaster::CANopenRequest CANopen_Tx;
static CANopenMaster::CANopenResponse CANopen_Rx;


RunTaskDef runTaskHeader;


void MotionTask(void const *parment)
{
    double milagesXBack = 0;

    static int Encode;
    static int request_speed;

    struct
    {
        int pollStep;
        int count;
        int TemperatureDelay;
    } canOpenStatus;

    //    if( CanManualQueue == 0 )
    //    {
    //        CanManualQueue = xQueueCreate( 5, sizeof( CanManualDef ) );
    //    }
    NavigationOperationStd navigationOperationData;

    MotionStatus.BootUp = 3;
    MotionStatus.EcodeDelay = true;
    MotionStatus.SpeedDelay = false;
    MotionStatus.CanDelay = false;
    MotionStatus.CanRestDelay = true;
    MotionStatus.alarm = false;
    // defatult enable EXTI, Navigation, Switch, Operation, StartUp log
    DebugCtrl.enableNavigation = 1;
    DebugCtrl.enableOperation = 1;
    DebugCtrl.enableStartUp = 1;
    DebugCtrl.enableSwitch = 1;
    DebugCtrl.enableXYExti = 0;
    DebugCtrl.enableMotion = 1;
    DebugCtrl.enableMotionDebug = 0;
    DebugCtrl.enableSecondLoaclizationTimeOut = 0;
    DebugCtrl.enableRealTimeEcode = 0;
    DebugCtrl.enableRealTimeSpeed = 0;
    DebugCtrl.runSensorCheck = 1;
    DebugCtrl.RFIDUpdate = 1;
    DebugCtrl.MotorTemperature = 1;
    DebugCtrl.enableCanRawData = 0;
    DebugCtrl.enableMotorAlarm = 1;
    // osDelay( 5000 );
    /*
    while( EXTIQue == 0 )
    {
        EXTIQue = xQueueCreate( 10, sizeof( ExtiQueDef ) );
        osDelay( 10 );
    }
    */
    setExti(1);

    bool IsupDown;
    Can1Init();
    //    InitMap();

    if (DebugCtrl.enableStartUp)
    {
        taskENTER_CRITICAL();
        {
            printf("Motor Start Up Ok\r\n");
        }
        taskEXIT_CRITICAL();
    }
    //agv.Stop_Accuracy = 1;
    agv.sArriveCtrlTime = 210;
    agv.sSpeed_min = 10;

    //    sAccBackUp = agv.sAcceleration;
    uint32_t PreviousWakeTime = osKernelSysTick();

    CANopen_Rx.Event_Rx_SDO_Complete = Rx_SDO_Commplate;
    CANopen_Rx.Event_Rx_PDO_Complete = Rx_PDO_Commplate;
    CANopen_Tx.Event_Tx_Work = CanTx;
    CANopen_Rx.Event_Rx_Work = CanRx;
    CANopen_Rx.Event_Rx_HeartBeat_Complete = canHeartbeat;

    agv.AGV_Pos = 0;

    /* power key */
    struct
    {
        uint32_t clock;
        bool flag;
        bool Poweroff;
        bool savePowerStatus;
    } PowerCount;
    PowerCount.Poweroff = false;
    PowerCount.savePowerStatus = false;
    PowerCount.flag = false;
    canOpenStatus.count = 0;
    canOpenStatus.pollStep = -1;
    canOpenStatus.TemperatureDelay = 0;

    uint32_t TickCount = 0;
    if (DebugCtrl.enableStartUp)
    {
        taskENTER_CRITICAL();
        {
            printf("[\t%d] Motion Start1\r\n", osKernelSysTick());
        }
        taskEXIT_CRITICAL();
    }
    MotionStatus.alarm = false;

    osDelay(5000);
    //MotionStatus.BootUp = 0;
    runTaskHeader.next = 0;

    InOutSwitch inOutTarget = getSwitchStatus();
    if( inOutTarget == InOutSwitchUnknow )
        inOutTarget = InOutSwitchIn;

    for (;;)
    {
        if (1)
        {
            if (NavigationOperationQue)
            {
                if (xQueueReceive(NavigationOperationQue, &navigationOperationData, 0) == pdPASS)
                {
                    switch (navigationOperationData.cmd)
                    {
                    case 3:
                        if (!MotionStatus.BootUp)
                        {
                            AGV_Pos= navigationOperationData.Data.posTo;
                        }
                        break;
                    case 4:
                        if( 1 )
                        {
                            RunTaskDef runTask;
                            /*

                            1: 设置速度
                            2: 伸摆杆
                            3：收摆杆
                            4：伸摆杆到位确认
                            5：收摆杆到位确认
                            6：清零

                             */
                            switch( navigationOperationData.Data.op )
                            {
                            case 1: // speed;
                                runTask.cmd = setSpeed;
                                runTask.data.fData = navigationOperationData.Data.speedTo;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd( &runTaskHeader, runTask );
                                break;
                            case 2:
                                runTask.cmd = 2;
                                runTask.data.uData = 0;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd( &runTaskHeader, runTask );
                                break;
                            case 3:
                                runTask.cmd = 2;
                                runTask.data.uData = 1;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd( &runTaskHeader, runTask );
                                break;
                            case 4:
                                runTask.cmd = 3;
                                runTask.data.uData = 0;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd( &runTaskHeader, runTask );
                                break;
                            case 5:
                                runTask.cmd = 3;
                                runTask.data.uData = 1;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd( &runTaskHeader, runTask );
                                break;
                            case 6:
                                runTask.cmd = 4;
                                runTask.data.uData = 0;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd( &runTaskHeader, runTask );
                                break;
                            default:
                                break;
                            }
                        }
                        break;
                    case 5:
                        break;

                    case 6:
                        SetSelfPosition(navigationOperationData.Data.posTo);
                        break;
                    case 10:
                        if (!MotionStatus.BootUp)
                        {
                            // BeltOperatingConfig(navigationOperationData.Data.data, navigationOperationData.Data.time );
                        }
                        break;
                    case 14:
                        cancelNavigate();
                        break;
                    default:
                        break;
                    }
                }
            }
        }
        if (getPowerKey() == on)
        {
            if (PowerCount.flag)
            {
                if (PreviousWakeTime - PowerCount.clock > 500)
                {
                    PowerCount.Poweroff = true;
                }
            }
            else
            {
                PowerCount.flag = true;
                PowerCount.clock = PreviousWakeTime;
            }
        }
        else
        {
            if (PowerCount.Poweroff)
            {
                setPowerKey(off);
            }
            PowerCount.flag = false;
        }

        if (getEmergencyKey() == on)
        {
            agv.iEmergency = false;
        }
        else
        {
            agv.iEmergency = false;
        }
#if 0
        if (CANopen_Rx.work())
        {
            static int enCodeAlarmCount = 0;
            if (abs(agv.EncoderValue - Encoder_Value) > 10000)
            {
                if (enCodeAlarmCount < 3)
                {
                    taskENTER_CRITICAL();
                    {
                        printf("[\t%d] Encode up too much\r\n", PreviousWakeTime);
                    }
                    taskEXIT_CRITICAL();
                }
                if (agv.EncoderValue == 0)
                {
                    agv.EncoderValue = Encoder_Value;
                }
                else
                {
                    enCodeAlarmCount++;
                }
            }
            else
            {
                agv.EncoderValue = Encoder_Value;
            }
        }
#else
        agv.EncoderValue += agv.Request_RPM * AGV_EncoderCPC * 2 / 60 / 1000;
        agv.DetectDynamics();
        if( MotionStatus.BootUp )
            MotionStatus.BootUp = 0;
//       HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR3, agv.AGV_Pos );

#endif


        if (MotionStatus.EcodeDelay)
        {
            ;
        }
        else
        {
            if (MotionStatus.BootUp > 0)
                MotionStatus.BootUp = 0;
        }

        agv.clock = (int)PreviousWakeTime;
        CANopen_Tx.clock_time = (int)PreviousWakeTime;
        CANopen_Rx.clock_time = (int)PreviousWakeTime;

        if (MotionStatus.BootUp)
        {
            agv.Motion_Status_Now = agv.Motion_work(agv.AGV_Pos);
            if (agv.iEmergency == true)
            {
                agv.Motion_Status_Now = AGV_Parallel_Motion::ms_Emergency;
            }
            AGV_Pos = agv.AGV_Pos;
            lastPosition = agv.AGV_Pos;
        }
        else
        {
            agv.Motion_Status_Now = agv.Motion_work(AGV_Pos);
            while( runTaskHeader.next )
            {
                if( agv.AGV_Pos > runTaskHeader.next->position )
                {
                    if( agv.AGV_Pos - runTaskHeader.next->position < 5 )
                    {
                        /*
                            1: 设置速度
                            2: 伸摆杆
                            3：收摆杆
                            4：伸摆杆到位确认
                            5：收摆杆到位确认
                            6：清零
                            */
                        switch( runTaskHeader.next->cmd )
                        {
                        case 1:
                            agv.sSpeed_max = runTaskHeader.next->data.fData;
                            break;
                        case 2:
                            inOutTarget = InOutSwitchOut;
                            break;
                        case 3:
                            inOutTarget = InOutSwitchIn;
                            break;
                        case 4:
                        case 5:
                        case 6:
                            break;
                        default:
                            break;
                        }
                        /*
                        taskENTER_CRITICAL();
                        {
                            printf( "[\t%d] run Task at %0.2f: cmd->%d, position->%0.2f, speed->%0.2f\r\n", PreviousWakeTime, agv.AGV_Pos, runTaskHeader.next->cmd, runTaskHeader.next->position, runTaskHeader.next->data.fData );
                        }
                        taskEXIT_CRITICAL();
                        */
                        debugOut( 0, (char *)"[\t%d] run Task at %0.2f: cmd->%d, position->%0.2f, speed->%0.2f\r\n", PreviousWakeTime, agv.AGV_Pos, runTaskHeader.next->cmd, runTaskHeader.next->position, runTaskHeader.next->data.fData );
                    }
                    else
                    {
                        /*
                        taskENTER_CRITICAL();
                        {
                            printf( "[\t%d] miss operation at %0.2f : cmd->%d, position->%0.2f, speed->%0.2f\r\n", PreviousWakeTime, agv.AGV_Pos, runTaskHeader.next->cmd, runTaskHeader.next->position, runTaskHeader.next->data.fData );
                        }
                        taskEXIT_CRITICAL();
                        */
                        debugOut( 0, (char *)"[\t%d] miss operation at %0.2f : cmd->%d, position->%0.2f, speed->%0.2f\r\n", PreviousWakeTime, agv.AGV_Pos, runTaskHeader.next->cmd, runTaskHeader.next->position, runTaskHeader.next->data.fData );

                    }
                    listDeleteItemByIndex( &runTaskHeader, 1 );
                }
                else
                    break;

            }
            setSwitch( inOutTarget );
            if (agv.iEmergency == true)
            {
                AGV_Pos = agv.AGV_Pos;
                agv.Motion_Status_Now = AGV_Parallel_Motion::ms_Emergency;
            }
        }

        if (MotionStatus.SpeedDelay)
        {
            request_speed = 0;
        }
        else
        {
            request_speed = -(int)(((double)agv.Request_RPM * 512 * 10000 * 9.3333333 ) / 1875);
        }
        if (DebugCtrl.enableRealTimeSpeed)
        {
            static float xSpeedBak, ySpeedBak;
            if (xSpeedBak != agv.Request_Speed)
            {
                xSpeedBak = agv.Request_Speed;
                /*
                taskENTER_CRITICAL();
                {
                    printf("[\t%d] Real-Time Speed: %0.2f mm/s\r\n", PreviousWakeTime, agv.Request_Speed);
                }
                taskEXIT_CRITICAL();
                */
                debugOut( 0, (char *)"[\t%d] Real-Time Speed: %0.2f mm/s\r\n", PreviousWakeTime, agv.Request_Speed);
            }
        }
        if (abs(agv.AGV_Pos - milagesXBack) > 1)
        {
            milages += abs(agv.AGV_Pos - milagesXBack);
            milagesXBack = agv.AGV_Pos;
            if (DebugCtrl.enableRealTimeEcode)
            {
                /*
                taskENTER_CRITICAL();
                {
                    printf("[\t%d] X - Ecode: %d\tPosition: %0.2f\r\n", PreviousWakeTime, agv.EncoderValue, agv.AGV_Pos);
                }
                taskEXIT_CRITICAL();
                */
                debugOut( 0, (char *)"[\t%d] X - Ecode: %d\tPosition: %0.2f\r\n", PreviousWakeTime, agv.EncoderValue, agv.AGV_Pos);
            }
        }

        static int alarmBack[2] = {0, 0};

        static int canCount = 0;
        if (canCount++ >= 1)
        {
            canCount = 0;
            if( MotionStatus.alarm )
            {
                if( canOpenStatus.pollStep < 11 )
                {
                    canOpenStatus.pollStep = 11;
                }
            }

            switch (canOpenStatus.pollStep)
            {
            case -1:
                if (CANopen_Tx.initialzation(1))
                {
                    canOpenStatus.pollStep = 1;
                    debugOut( 0, (char *)"[\t%d]init 1 success\r\n", osKernelSysTick());
                }
                osDelay(10);
                break;

            case 1:
                if (CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Start_Remote_Node))
                    canOpenStatus.pollStep = 5;
                osDelay(10);
                break;

            case 5:
                canOpenStatus.count = 0;
                CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Master2Slave_request_4Bit23, 0x60ff, 0, request_speed);
                break;

            case 9:
                CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Master2Slave_request_2Bit2b, 0x1017, 0, 0x03EB);
                canOpenStatus.pollStep = 12;
                break;

            case 11:
                if (CANopen_Tx.InitialisingWithOutAcc(2))
                    canOpenStatus.pollStep = 3;
                break;
            case 12:
                if (MotionStatus.CanRestDelay)
                {
                    CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Rest_Node);
                }
                else
                    canOpenStatus.pollStep = -2;
                break;
            case 13:
                if (MotionStatus.alarm)
                    CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Master2Slave_request_1Bit2f, 0x7006, 0, 0x01);
                canOpenStatus.pollStep++;
                break;
            case 14:
                if (MotionStatus.alarm)
                {
                    CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Master2Slave_request_2Bit2b, 0x6040, 0, 0x80);
                }
                if (MotionStatus.alarm)
                {
                    canOpenStatus.pollStep = 11;
                    break;
                }
                MotionStatus.CanDelay = false;

                MotionStatus.EcodeDelay = true;
                canOpenStatus.pollStep = 9;
                break;
            default:
                canOpenStatus.pollStep = 9;
                break;
            }
        }
        // todo :
        /* 读取上位机下发的RFID位置，当小车停在格口中间时使用RFID更新当前位置 */
        /* 绝对延时，绝对延时保证代码执行周期固定，即使程序运行时间不确定 此延时依赖FreeRTOS */
        osDelayUntil(&PreviousWakeTime, 2);
    }
}
