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
#include "BeltDriveController.h"

#define MaxSpeed 2000

extern QueueHandle_t SwitchBeltTaskQue;
extern int switchReach;
extern InOutSwitch target;
extern InOutSwitch targetLast;
static float lastPosition;

#ifdef __cplusplus
extern "C"
{
    void MotorTestTask(void const *parment);
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
    void listAddCallBack(RunTaskDef data);
    void listDelCallBack(RunTaskDef data);
    void ClearMotorAlarm();
}
#endif

static int Encoder_Value;
static int MotorStatusWord_PDO;
static int MotorModeWord_PDO;

static float Speed_Value_PDO;

static AGV_Parallel_Motion agv;

static BeltDriveController Belt_Ctrl;
static int BeltGetPack = 0;

static float AGV_Pos;
QueueHandle_t setZerpSemap = 0;
QueueHandle_t SwitchIN6Semap = 0;
QueueHandle_t SwitchIN7Semap = 0;

void listAddCallBack(RunTaskDef data)
{
    if (DebugCtrl.AddRunTask)
        debugOut(0, "[\t%d] Add Run Task(cmd->%d, position->%0.2f, speed->%0.2f) To Run-task list [ok]\r\n", osKernelSysTick(), data.cmd, data.position, data.data.fData);
}
void listDelCallBack(RunTaskDef data)
{
    if (DebugCtrl.DelRunTask)
        debugOut(0, "[\t%d] Delete Run Task(cmd->%d, position->%0.2f, speed->%0.2f) From Run-task list [ok]\r\n", osKernelSysTick(), data.cmd, data.position, data.data.fData);
}

AGV_Parallel_Motion *getAGVHandle()
{
    return &agv;
}

static struct
{
    bool CanDelay;
    bool CanRestDelay;
    bool EcodeDelay;
    bool alarm;
    BaseType_t lastAlarmTime;
    bool alarmCleanDisable;
    bool enable;
    bool speedMode;
    BaseType_t lastPDOTime;
    bool startUp;
    bool handSpeedMode;
    int handSpeed;
    BaseType_t handSpeedTime;
    bool bootUp;
    BaseType_t lastEncodeTime;
    short Current;

} MotionStatus;

int IsMotorAlarm()
{
    if (agv.iEmergencyByPause)
    {
        return MotionStatus.alarm;
    }
    else
    {
        if (MotionStatus.alarmCleanDisable)
        {
            return (MotionStatus.alarm || !MotionStatus.enable);
        }
        else
            return 0;
    }
}

void ClearMotorAlarm()
{
    MotionStatus.alarmCleanDisable = false;
    MotionStatus.lastAlarmTime = osKernelSysTick();
}
static int BeltOperating = 0;
static int BeltOperatingTime = 0;
static bool BeltOperatingPause = false;
static double milages = 0;

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
void GetMaxSpeed(float *xSpeed)
{
    *xSpeed = agv.sSpeed_max;
}
void GetMotorCurrent(short *current)
{
    if (current)
        *current = MotionStatus.Current;
}
int SetSelfPosition(float X)
{
    agv.AGV_Pos = X;
    AGV_Pos = agv.AGV_Pos;
    lastPosition = agv.AGV_Pos;

    if (DebugCtrl.enableStartUp)
    {
        debugOut(0, "[\t%d] Set Self Position:%0.2f [...]\r\n", osKernelSysTick(), X);
    }
    if (!MotionStatus.EcodeDelay)
    {
        if (DebugCtrl.enableStartUp)
        {
            debugOut(0, "[\t%d] Speed Up [ok]\r\n", osKernelSysTick());
        }
        return pdTRUE;
    }
    else
    {
        if (DebugCtrl.enableStartUp)
        {
            debugOut(0, "[\t%d] Speed Up [error]\r\n", osKernelSysTick());
        }
    }
    return pdFALSE;
}

void GetNextPiont(float *X)
{
    *X = AGV_Pos;
}
void GetPosition(float *X)
{
    *X = agv.AGV_Pos;
}
void setRunAcc(float acc)
{
    if (agv.Motion_Status_Now == AGV_Parallel_Motion::ms_Arrived)
        agv.sAcceleration = acc;
}
float getRunAcc()
{
    return agv.sAcceleration;
}

void SetiEmergency(int S)
{
    if (S)
        agv.iEmergencyBySoftware = true;
    else
        agv.iEmergencyBySoftware = false;
}
RunTaskDef runTaskHeader;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IN_5_Pin)
    {
        if (HAL_GPIO_ReadPin(IN_5_GPIO_Port, IN_5_Pin))
        {
            if (setZerpSemap)
            {
                debugOut(1, "[\t%d] GPIO ISR position:%0.2f\r\n", osKernelSysTick(), agv.AGV_Pos);
                BaseType_t nextTask;
                xSemaphoreGiveFromISR(setZerpSemap, &nextTask);
            }
        }
    }
    if (GPIO_Pin == IN_6_Pin)
    {
        if (SwitchIN6Semap)
        {
            if (HAL_GPIO_ReadPin(IN_6_GPIO_Port, IN_6_Pin))
            {
                debugOut(1, "[\t%d] GPIO ISR IN6 [UP] [ok]\r\n", osKernelSysTick());
                BaseType_t nextTask;
                //    xSemaphoreGiveFromISR( SwitchIN6Semap, &nextTask );
            }
            else
            {
                debugOut(1, "[\t%d] GPIO ISR IN6 [DOWN] [ok]\r\n", osKernelSysTick());
            }
        }
    }
    if (GPIO_Pin == IN_7_Pin)
    {
        if (SwitchIN7Semap)
        {
            if (HAL_GPIO_ReadPin(IN_7_GPIO_Port, IN_7_Pin))
            {
                debugOut(1, "[\t%d] GPIO ISR IN7 [UP] [ok]\r\n", osKernelSysTick());
                BaseType_t nextTask;
                //   xSemaphoreGiveFromISR( SwitchIN7Semap, &nextTask );
            }
            else
            {
                debugOut(1, "[\t%d] GPIO ISR IN7 [DOWN] [ok]\r\n", osKernelSysTick());
            }
        }
    }
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
                    debugOut(0, "[\t%d] Can ID: %02X, DLC: %d Data: ", osKernelSysTick(), *oID, *oLength);
                    for (int i = 0; i < *oLength; i++)
                    {
                        debugOut(0, "%02X ", oArray[i]);
                    }
                    debugOut(0, "\r\n");
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
    return;
}

void Rx_PDO_Commplate(int oID, char Array[8], int len)
{
    static bool alarmOuted[2] = {false, false};
    MotionStatus.lastPDOTime = osKernelSysTick();
    switch (oID)
    {
    case 0x181:
        if (1)
        {
            if (MotionStatus.startUp)
                MotionStatus.startUp = false;

            union {
                char Hex[4];
                int Data;
            } i32ToHex;
            for (int i = 0; i < 4; i++)
            {
                i32ToHex.Hex[i] = Array[i];
            }
            Encoder_Value = i32ToHex.Data;
            if (MotionStatus.EcodeDelay)
            {
                if (DebugCtrl.enableStartUp)
                {
                    debugOut(0, "[\t%d] Encode Up [ok]\r\n", osKernelSysTick());
                }
                MotionStatus.EcodeDelay = false;
                agv.EncoderValue = Encoder_Value;
                MotionStatus.lastEncodeTime = osKernelSysTick();

                agv.DetectDynamics();
                float posTmp = 0;
                double milsTmp = 0;
                if (readPosFromBKP(&posTmp, &milsTmp))
                {
                    milages = milsTmp;
                    SetSelfPosition(posTmp);
                }
                else
                    SetSelfPosition(0);
            }
            i32ToHex.Hex[0] = Array[4];
            i32ToHex.Hex[1] = Array[5];
            i32ToHex.Hex[2] = Array[6];
            i32ToHex.Hex[3] = Array[7];
            double speedTmp = i32ToHex.Data * 1875;
            speedTmp = speedTmp / 10000;
            speedTmp = speedTmp / 512;
            agv.FeedBack_RPM = speedTmp;
        }
        break;
    case 0x281:
    {
        if (1)
        {
            union {
                short i16Data;
                uint16_t u16Data;
                uint8_t Hex[2];
            } i16ToHex;
            i16ToHex.Hex[0] = Array[0];
            i16ToHex.Hex[1] = Array[1];

            MotionStatus.Current = i16ToHex.i16Data;
            uint16_t alarmCode = 0;
            i16ToHex.Hex[0] = Array[2];
            i16ToHex.Hex[1] = Array[3];
            alarmCode = i16ToHex.u16Data;

            if (alarmCode)
            {
                if (MotionStatus.alarm != true)
                {
                    MotionStatus.alarm = true;
                    if (osKernelSysTick() > MotionStatus.lastAlarmTime)
                    {
                        if (osKernelSysTick() - MotionStatus.lastAlarmTime < 10000)
                        {
                            MotionStatus.alarmCleanDisable = true;
                        }
                        else
                            MotionStatus.lastAlarmTime = MotionStatus.lastPDOTime;
                    }
                    else
                        MotionStatus.lastAlarmTime = MotionStatus.lastPDOTime;

                    debugOut(0, "[\t%d] Motor Alarm [first]:code->0x%X\r\n", osKernelSysTick(), alarmCode);
                }
                else
                {
                    MotionStatus.alarm = true;
                    static uint16_t alarmCodeBak;
                    if (alarmCodeBak != alarmCode)
                    {
                        debugOut(0, "[\t%d] Motor Alarm: code->0x%X\r\n", osKernelSysTick(), alarmCode);
                        alarmCodeBak = alarmCode;
                    }
                }
            }
            else
            {
                MotionStatus.alarm = false;
                MotionStatus.alarmCleanDisable = false;
            }
            i16ToHex.Hex[0] = Array[4];
            i16ToHex.Hex[1] = Array[5];
            MotorStatusWord_PDO = i16ToHex.u16Data;
            MotorModeWord_PDO = Array[6];
            if (MotorStatusWord_PDO & 0x04)
            {
                MotionStatus.enable = true;
            }
            else
            {
                MotionStatus.enable = false;
            }
            switch (MotorModeWord_PDO)
            {
            case 0:
                if (MotionStatus.speedMode != false)
                {
                    MotionStatus.speedMode = false;
                    debugOut(0, "[\t%d] Motor Disable\r\n", osKernelSysTick());
                }
                break;
            default:
            case 1:
                if (MotionStatus.speedMode != false)
                {
                    MotionStatus.speedMode = false;
                    debugOut(0, "[\t%d] Motor NOT AT SPEED MODE!!!\r\n", osKernelSysTick());
                }
                break;
            case 3:
                MotionStatus.speedMode = true;
                break;
            }
        }
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
                debugOut(0, "[\t%d] Motor 1 Up\r\n", osKernelSysTick());
            }
            MotionStatus.CanDelay = false;
        }

        if (oStatus == 0x7f)
        {
            if (MotionStatus.CanRestDelay)
            {
                //   MotionStatus.CanRestDelay = false;
                MotionStatus.EcodeDelay = true;
                debugOut(0, "[\t%d] Motor 1 Rest OK\r\n", osKernelSysTick());
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
extern "C"
{
    void modbusTask(void const *arg);
    void StartSwitchTask();
    int isPackOnCar();
}

int isPackOnCar()
{
    if (BeltOperating == 6 || BeltOperating == 7)
        return getThingSensorStatus(1);
    else
    {
        if (BeltGetPack)
            return 0;
        else
            return getThingSensorStatus(0);
    }
}
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
        int heartBeatDelay;
        int alarmBak[2];
    } canOpenStatus;

    //    if( CanManualQueue == 0 )
    //    {
    //        CanManualQueue = xQueueCreate( 5, sizeof( CanManualDef ) );
    //    }
    NavigationOperationStd navigationOperationData;

    MotionStatus.EcodeDelay = true;
    MotionStatus.CanDelay = true;
    MotionStatus.CanRestDelay = true;
    MotionStatus.alarm = false;
    MotionStatus.alarmCleanDisable = false;
    MotionStatus.startUp = true;
    MotionStatus.bootUp = true;

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
    DebugCtrl.AddRunTask = 1;
    DebugCtrl.DelRunTask = 1;

    Can1Init();

    agv.Stop_Accuracy = 2;
    agv.sArriveCtrlTime = 5000;
    agv.sSpeed_min = 10;
    agv.sSpeed_max = 300;
    agv.sDeceleration_distance = 2;
    agv.sAcceleration = 2000;

    uint32_t PreviousWakeTime = osKernelSysTick();

    CANopen_Rx.Event_Rx_SDO_Complete = Rx_SDO_Commplate;
    CANopen_Rx.Event_Rx_PDO_Complete = Rx_PDO_Commplate;
    CANopen_Tx.Event_Tx_Work = CanTx;
    CANopen_Rx.Event_Rx_Work = CanRx;
    CANopen_Rx.Event_Rx_HeartBeat_Complete = canHeartbeat;

    agv.AGV_Pos = 0;

    /* power key */

    canOpenStatus.count = 0;
    canOpenStatus.pollStep = -1;
    canOpenStatus.TemperatureDelay = 0;

    uint32_t TickCount = 0;
    if (DebugCtrl.enableStartUp)
    {
        debugOut(0, "[\t%d] Motion Start1\r\n", osKernelSysTick());
    }
    MotionStatus.alarm = false;
    setZerpSemap = xSemaphoreCreateBinary();
    while (!setZerpSemap)
    {
        setZerpSemap = xSemaphoreCreateBinary();
    }

    osDelay(5000);
    MotionStatus.CanDelay = false;

    runTaskHeader.next = 0;

    InOutSwitch inOutTarget = getSwitchStatus();
    InOutSwitch inOutTargetNow;
    if (inOutTarget == InOutSwitchUnknow)
        inOutTarget = InOutSwitchIn;

    for (;;)
    {
        agv.clock = (int)PreviousWakeTime;
        CANopen_Tx.clock_time = (int)PreviousWakeTime;
        CANopen_Rx.clock_time = (int)PreviousWakeTime;
        if (1) /// rec task work for server and add to run task
        {
            if (NavigationOperationQue)
            {
                if (xQueueReceive(NavigationOperationQue, &navigationOperationData, 0) == pdPASS)
                {
                    switch (navigationOperationData.cmd)
                    {
                    case Enum_SetZeroPosition:
                        agv.AGV_Pos = navigationOperationData.Data.posTo;
                        AGV_Pos = agv.AGV_Pos;
                        break;
                    case Enum_SetMaxSpeed:
                        agv.sSpeed_max = navigationOperationData.Data.speedTo;
                        debugOut(0, "[\t%d] Set Speed:%0.2f\r\n", PreviousWakeTime, agv.sSpeed_max);
                        break;
                    case Enum_SendNavigation:

                        if (/* navigationOperationData.Data.posTo < agv.AGV_Pos */ 1)
                        {
                            RunTaskDef runTask;
                            if (listGetItemByCMD(&runTaskHeader, 6, &runTask))
                            {
                                if (navigationOperationData.Data.posTo < runTask.position)
                                {
                                    if (runTask.position + navigationOperationData.Data.posTo < AGV_Pos)
                                    {
                                        // back car
                                        if (getSwitchStatus() == InOutSwitchIn)
                                        {
                                            AGV_Pos = runTask.position + navigationOperationData.Data.posTo;
                                        }
                                        else
                                        {
                                            inOutTarget = InOutSwitchIn;
                                        }
                                    }
                                    else
                                        AGV_Pos = runTask.position + navigationOperationData.Data.posTo;
                                }
                                else
                                {
                                    if (navigationOperationData.Data.posTo < AGV_Pos)
                                    {
                                        // back car
                                        if (getSwitchStatus() == InOutSwitchIn)
                                        {
                                            AGV_Pos = navigationOperationData.Data.posTo;
                                        }
                                        else
                                        {
                                            inOutTarget = InOutSwitchIn;
                                        }
                                    }
                                    else
                                        AGV_Pos = navigationOperationData.Data.posTo;
                                }
                            }
                            else
                            {
                                if (navigationOperationData.Data.posTo < AGV_Pos)
                                {
                                    if (getSwitchStatus() == InOutSwitchIn)
                                    {
                                        AGV_Pos = navigationOperationData.Data.posTo;
                                    }
                                    else
                                        inOutTarget = InOutSwitchIn;
                                }
                                else
                                    AGV_Pos = navigationOperationData.Data.posTo;
                            }
                        }
                        else
                            AGV_Pos = navigationOperationData.Data.posTo;
                        agv.isNewPosition = true;
                        if (agv.iEmergencyByCancel)
                            agv.iEmergencyByCancel = false;

                        debugOut(0, "[\t%d] Sest Position :%0.2f\r\n", PreviousWakeTime, AGV_Pos);
                        break;
                    case Enum_sendOperation:
                        if (1)
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
                            switch (navigationOperationData.Data.op)
                            {
                            case 1: // speed;
                                runTask.cmd = 1;
                                runTask.data.fData = navigationOperationData.Data.speedTo;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd(&runTaskHeader, runTask);
                                break;
                            case 2:
                                runTask.cmd = 2;
                                runTask.data.uData = 0;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd(&runTaskHeader, runTask);
                                break;
                            case 3:
                                runTask.cmd = 3;
                                runTask.data.uData = 1;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd(&runTaskHeader, runTask);
                                break;
                            case 4:
                                runTask.cmd = 4;
                                runTask.data.uData = 0;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd(&runTaskHeader, runTask);
                                break;
                            case 5:
                                runTask.cmd = 5;
                                runTask.data.uData = 1;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd(&runTaskHeader, runTask);
                                break;
                            case 6:
                                runTask.cmd = 6;
                                runTask.data.uData = 0;
                                runTask.position = navigationOperationData.Data.posTo;
                                listAdd(&runTaskHeader, runTask);
                                break;
                            default:
                                break;
                            }
                        }
                        break;
                    case Enum_SetInOutSwitch:
                        if (navigationOperationData.Data.op)
                            inOutTarget = InOutSwitchOut;
                        else
                            inOutTarget = InOutSwitchIn;
                        break;

                    case Enum_disableMotor:
                        if (navigationOperationData.Data.op)
                            canOpenStatus.pollStep = 2;
                        else
                            canOpenStatus.pollStep = 3;

                    case Enum_CancelNavigation:
                        agv.iEmergencyByCancel = true;
                        agv.iEmergencyBySoftware = false;
                        // agv.iEmergencyByPause = false;
                        deleteList(&runTaskHeader);
                        break;
                    case Enum_PauseNavigation:
                        if (navigationOperationData.Data.op)
                        {
                            /*
                            canOpenStatus.pollStep = 2;
                            navigationOperationData.cmd = 4;
                            xQueueSend( SwitchBeltTaskQue, &navigationOperationData, 100 );
                            */
                            debugOut(0, "[\t%d] <CMD> <Motion> {Pause} ok\r\n", PreviousWakeTime);
                            agv.iEmergencyByPause = true;
                        }
                        else
                        {
                            debugOut(0, "[\t%d] <INFO> <Motion> {Exit Pause} exit pause mode\r\n", PreviousWakeTime);

                            //                            canOpenStatus.pollStep = 3;
                            //                            navigationOperationData.cmd = 5;
                            //                            while( switchReach == -2 )
                            //                            {
                            //                                xQueueSend( SwitchBeltTaskQue, &navigationOperationData, 100 );
                            //                                osDelay(10);
                            //                            }
                            agv.iEmergencyByPause = false;
                        }
                        break;
                    case Enum_PullThing:
                        if (navigationOperationData.Data.op)
                            BeltOperating = 7;
                        else
                            BeltOperating = 6;
                        break;
                    case Enum_PushThing:
                        BeltOperatingTime = 2000;
                        if (navigationOperationData.Data.op)
                            BeltOperating = 2;
                        else
                            BeltOperating = 1;
                        break;
                    case Enum_setHandSpeedMode:
                        if (navigationOperationData.Data.op)
                        {
                            if (!MotionStatus.handSpeedMode)
                            {
                                MotionStatus.handSpeedMode = true;
                                MotionStatus.handSpeed = 0;
                            }
                            debugOut(0, "[\t%d] Switch to Hand Speed Mode OK\r\n", PreviousWakeTime);
                        }
                        else
                        {
                            if (MotionStatus.handSpeedMode)
                            {
                                MotionStatus.handSpeedMode = false;
                                AGV_Pos = agv.AGV_Pos;
                            }
                            debugOut(0, "[\t%d] Switch Auto Position Mode OK\r\n", PreviousWakeTime);
                        }
                        break;
                    case Enum_SetHandSpeed:
                        if (abs(MotionStatus.handSpeed - navigationOperationData.Data.speedTo) > 50)
                        {
                            MotionStatus.handSpeed = navigationOperationData.Data.speedTo;
                            debugOut(0, "[\t%d] Set Hand Speed Mode speed:%d\r\n", PreviousWakeTime, MotionStatus.handSpeed);
                        }
                        MotionStatus.handSpeedTime = PreviousWakeTime;
                        break;
                    case Enum_SetSleep:
                        if (navigationOperationData.Data.op)
                        {
                            canOpenStatus.pollStep = 2;
                            navigationOperationData.cmd = 4;
                            xQueueSend(SwitchBeltTaskQue, &navigationOperationData, 100);
                        }
                        else
                        {
                            canOpenStatus.pollStep = 3;
                            navigationOperationData.cmd = 5;
                            xQueueSend(SwitchBeltTaskQue, &navigationOperationData, 100);
                        }
                        break;
                    default:
                        break;
                    }
                }
            }
        }

        if (1)
        {
            static int thingSensorBak[3];
            Belt_Ctrl.info.clock = (int)PreviousWakeTime;
            Belt_Ctrl.info.read_input[0] = getThingSensor(1, 25) == on ? 1 : 0;
            Belt_Ctrl.info.read_input[1] = getThingSensor(2, 25) == on ? 1 : 0;
            Belt_Ctrl.info.read_input[2] = getThingSensor(3, 25) == on ? 1 : 0;
            for (int i = 0; i < 3; i++)
            {
                if (thingSensorBak[i] != Belt_Ctrl.info.read_input[i])
                {
                    thingSensorBak[i] = Belt_Ctrl.info.read_input[i];
                    taskENTER_CRITICAL();
                    {
                        printf("[\t%d] ThingSensor status charge. [%d]->%d\r\n", PreviousWakeTime, i, thingSensorBak[i]);
                    }
                    taskEXIT_CRITICAL();
                }
            }

            Belt_Ctrl.work();
            switch (BeltOperating)
            {
            case 0:
                if (Belt_Ctrl.info.motor_stop)
                {
                    beltCtrl(0, BeltFront, 20);
                    if (!Belt_Ctrl.info.read_input[0] && Belt_Ctrl.info.read_input[1] && !Belt_Ctrl.info.read_input[2])
                    {
                        BeltGetPack = 0;
                    }
                }
                else
                {
                    beltCtrl(1, Belt_Ctrl.info.motor_direction == true ? BeltFront : BeltRev, 10);
                }
                break;
            case 1:
                for (int i = 0; i < 2; i++)
                {
                    if (BeltOperatingTime > 0)
                        BeltOperatingTime--;
                }
                if (BeltOperatingTime > 0)
                {
                    beltCtrl(1, BeltFront, 10);
                }
                else
                {
                    beltCtrl(0, BeltFront, 10);
                    BeltOperating = 0;
                }
                break;
            case 2:
                for (int i = 0; i < 2; i++)
                {
                    if (BeltOperatingTime > 0)
                        BeltOperatingTime--;
                }
                if (BeltOperatingTime > 0)
                {
                    beltCtrl(1, BeltRev, 10);
                }
                else
                {
                    beltCtrl(0, BeltRev, 10);
                    BeltOperating = 0;
                }
                break;
            case 3:
                beltCtrl(0, BeltFront, 10);
                break;
            case 4:
                beltCtrl(1, BeltFront, 10);
                break;
            case 5:
                beltCtrl(1, BeltRev, 10);
                break;
            case 6:
                if (!Belt_Ctrl.info.read_input[0] && !Belt_Ctrl.info.read_input[1] && !Belt_Ctrl.info.read_input[2])
                {
                    BeltGetPack = 1;
                    beltCtrl(1, BeltFront, 10);
                }
                else
                    BeltOperating = 0;
                break;
            case 7:
                if (!Belt_Ctrl.info.read_input[0] && !Belt_Ctrl.info.read_input[1] && !Belt_Ctrl.info.read_input[2])
                {
                    BeltGetPack = 1;
                    beltCtrl(1, BeltRev, 10);
                }
                else
                    BeltOperating = 0;
                break;
            default:
                BeltOperating = 0;
                break;
            }
        }

        if (1) // update encode and run status
        {
#if 1
            if (CANopen_Rx.work())
            {
                if (abs(agv.EncoderValue - Encoder_Value) > 100000)
                {
                    debugOut(0, "[\t%d] <ERROR> <Motion> {ENCODE} Encode up too much last->%d, new->%d [error]\r\n", PreviousWakeTime, agv.EncoderValue, Encoder_Value);
                    agv.iEmergencyByError = true;
                }
                else
                    agv.EncoderValue = Encoder_Value;
                MotionStatus.lastEncodeTime = PreviousWakeTime;
                //agv.DetectDynamics();
                static float posBakForBKP;
                if (fabsf(posBakForBKP - agv.AGV_Pos) > 1)
                {
                    posBakForBKP = agv.AGV_Pos;
                    writePosToBKP(agv.AGV_Pos, milages);
                }
                static float posBakForLog;
                if (fabsf(posBakForLog - agv.AGV_Pos) > 100)
                {
                    posBakForLog = agv.AGV_Pos;
                    debugOut(0, "[\t%d] <INFO> <Motion> {Real-Time Position} Positon->%0.2f, encoder->%d\r\n", PreviousWakeTime, posBakForLog, agv.EncoderValue);
                }
            }
            if (!MotionStatus.startUp)
            {
                if (PreviousWakeTime > MotionStatus.lastEncodeTime)
                {
                    if (PreviousWakeTime - MotionStatus.lastEncodeTime > 100)
                    {
                        debugOut(0, "[\t%d] <ERROR> <Motion> {ENCODE} Encode up timeout!!!\r\n", PreviousWakeTime);
                        MotionStatus.lastEncodeTime = PreviousWakeTime;
                    }
                }
                else
                    MotionStatus.lastEncodeTime = PreviousWakeTime;
            }
#else
            if (MotionStatus.EcodeDelay)
                MotionStatus.EcodeDelay = false;
            agv.EncoderValue += agv.Request_RPM * AGV_EncoderCPC * 5 / 60 / 1000;
            // agv.DetectDynamics();
//       HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR3, agv.AGV_Pos );
#endif
        }

        if (1) // run task at position
        {
            while (runTaskHeader.next)
            {
                if (osKernelSysTick() - PreviousWakeTime > 2)
                    break;
                if (fabsf(agv.AGV_Pos - runTaskHeader.next->position) < 10)
                {
                    if (1)
                    {
                        /*
                            1: 设置速度
                            2: 伸摆杆
                            3：收摆杆
                            4：伸摆杆到位确认
                            5：收摆杆到位确认
                            6：清零
                            */
                        if (runTaskHeader.next->cmd != 6)
                            debugOut(0, "[\t%d] run Task at %0.2f real-time speed->%f: cmd->%d, position->%0.2f, speed->%0.2f\r\n", PreviousWakeTime, agv.AGV_Pos, agv.Request_RPM, runTaskHeader.next->cmd, runTaskHeader.next->position, runTaskHeader.next->data.fData);
                        switch (runTaskHeader.next->cmd)
                        {
                        case 1:
                            agv.sSpeed_max = runTaskHeader.next->data.fData;
                            listDeleteItemByIndex(&runTaskHeader, 1);
                            debugOut(0, "Set speed by Running-task :%0.2f\r\n", agv.sSpeed_max);
                            break;
                        case 2:
                            inOutTarget = InOutSwitchOut;
                            listDeleteItemByIndex(&runTaskHeader, 1);
                            break;
                        case 3:
                            inOutTarget = InOutSwitchIn;
                            listDeleteItemByIndex(&runTaskHeader, 1);
                            break;
                        case 4:
                            if ((getSwitchStatus() != InOutSwitchOut) || (switchReach != 1) || (targetLast != InOutSwitchOut))
                            {
                                if (!agv.iEmergencyBySoftware)
                                    debugOut(0, "[\t%d] target not reach [OUT] !!!!!!\r\n", PreviousWakeTime);
                                agv.iEmergencyBySoftware = true;
                            }
                            else
                            {
                                listDeleteItemByIndex(&runTaskHeader, 1);
                            }
                            inOutTargetNow = InOutSwitchOut;
                            break;
                        case 5:
                            if ((getSwitchStatus() != InOutSwitchIn) || (switchReach != 1) || (targetLast != InOutSwitchIn))
                            {
                                if (!agv.iEmergencyBySoftware)
                                    debugOut(0, "[\t%d] target not reach [IN] !!!!!!\r\n", PreviousWakeTime);
                                agv.iEmergencyBySoftware = true;
                            }
                            else
                                listDeleteItemByIndex(&runTaskHeader, 1);
                            inOutTargetNow = InOutSwitchIn;
                            break;
                        case 6:
                            if (0)
                            {
                                if (xSemaphoreTake(setZerpSemap, 0) == pdPASS)
                                {
                                    float posNow = agv.AGV_Pos;
                                    agv.AGV_Pos = 0;
                                    AGV_Pos = AGV_Pos - posNow;
                                    listDeleteItemByIndex(&runTaskHeader, 1);
                                }
                            }
                            break;
                        default:
                            break;
                        }
                    }
                    else
                    {
                        if ((runTaskHeader.next->cmd == 4) || (runTaskHeader.next->cmd == 5))
                            if (agv.iEmergencyBySoftware)
                                break;
                        debugOut(0, "[\t%d] miss operation at %0.2f : cmd->%d, position->%0.2f, speed->%0.2f\r\n", PreviousWakeTime, agv.AGV_Pos, runTaskHeader.next->cmd, runTaskHeader.next->position, runTaskHeader.next->data.fData);
                        listDeleteItemByIndex(&runTaskHeader, 1);
                    }
                    if (runTaskHeader.next->cmd == 6)
                        break;
                    if ((runTaskHeader.next->cmd == 4) || (runTaskHeader.next->cmd == 5))
                        if (agv.iEmergencyBySoftware)
                            break;
                }
                else
                {
                    if (agv.AGV_Pos > runTaskHeader.next->position)
                    {
                        if (runTaskHeader.next->cmd == 6)
                        {
                            if (agv.AGV_Pos - runTaskHeader.next->position > 100)
                            {
                                debugOut(0, "[\t%d] miss operation at %0.2f : cmd->%d, position->%0.2f, speed->%0.2f\r\n", PreviousWakeTime, agv.AGV_Pos, runTaskHeader.next->cmd, runTaskHeader.next->position, runTaskHeader.next->data.fData);
                                listDeleteItemByIndex(&runTaskHeader, 1);
                            }
                        }
                        else
                        {
                            /* when car stop by cmd 4,5 must igonre position */
                            if (runTaskHeader.next->cmd == 4 || runTaskHeader.next->cmd == 5)
                            {
                                if (agv.iEmergencyBySoftware)
                                    break;
                                else
                                    listDeleteItemByIndex(&runTaskHeader, 1);
                            }
                            else
                            {
                                debugOut(0, "[\t%d] miss operation at %0.2f : cmd->%d, position->%0.2f, speed->%0.2f\r\n", PreviousWakeTime, agv.AGV_Pos, runTaskHeader.next->cmd, runTaskHeader.next->position, runTaskHeader.next->data.fData);
                                listDeleteItemByIndex(&runTaskHeader, 1);
                            }
                        }
                    }
                    break;
                }
            }
        }
        if (1)
        {
            if (MotionStatus.enable && MotionStatus.speedMode)
            {
                if (agv.iEmergencyByMotorDisable)
                {
                    debugOut(0, "[\t%d] Set Speed ok\r\n", PreviousWakeTime);
                }
                agv.iEmergencyByMotorDisable = false;
            }
            else
            {
                if (!agv.iEmergencyByMotorDisable)
                {
                    debugOut(0, "[\t%d] Set Speed zero by motor disable or not speed mode!\r\n", PreviousWakeTime);
                }
                agv.iEmergencyByMotorDisable = true;
            }
            /* Let car run to switch status check position when switch status is error */
            if (agv.iEmergencyBySoftware && runTaskHeader.next)
                agv.Motion_Status_Now = agv.Motion_work(runTaskHeader.next->position);
            else
                agv.Motion_Status_Now = agv.Motion_work(AGV_Pos);
        }

        if (1)
        {
            if (xSemaphoreTake(setZerpSemap, 0) == pdPASS)
            {
                RunTaskDef zeroTask;
                if (listGetItemByCMD(&runTaskHeader, 6, &zeroTask))
                {
                    if (fabsf(zeroTask.position - agv.AGV_Pos) < 100)
                    {
                        debugOut(0, "[\t%d] run Task at %0.2f: cmd->%d, position->%0.2f, speed->%0.2f\r\n", PreviousWakeTime, agv.AGV_Pos, runTaskHeader.next->cmd, runTaskHeader.next->position, runTaskHeader.next->data.fData);
                        float posNow = agv.AGV_Pos;
                        float dis = posNow - zeroTask.position;
                        agv.AGV_Pos = 0;
                        AGV_Pos = AGV_Pos - zeroTask.position;
                        //   AGV_Pos = AGV_Pos + dis;
                        listDeleteItemByCMD(&runTaskHeader, 6);
                        BaseType_t timeBak = osKernelSysTick();
                        while (listGetItemByCMD(&runTaskHeader, 6, NULL))
                        {
                            listDeleteItemByCMD(&runTaskHeader, 6);
                            if (osKernelSysTick() >= timeBak)
                            {
                                if (osKernelSysTick() - timeBak > 100)
                                {
                                    agv.iEmergencyByError = true;
                                    break;
                                }
                            }
                            else
                            {
                                timeBak = osKernelSysTick();
                            }
                        }
                    }
                }
            }
        }

        if (1)
        {
            target = inOutTarget;
            if (agv.iEmergencyBySoftware)
            {
                if (inOutTargetNow == getSwitchStatus())
                    agv.iEmergencyBySoftware = false;
            }
        }

        if (MotionStatus.EcodeDelay)
        {
            request_speed = 0;
        }
        else
        {
            if (MotionStatus.handSpeedMode)
            {
                if (PreviousWakeTime >= MotionStatus.handSpeedTime)
                {
                    if (PreviousWakeTime - MotionStatus.handSpeedTime > 500)
                    {
                        if (abs(request_speed) > 0)
                            debugOut(0, "\t%d] <INFO> <Motion> {HandMode} stop by timeout\r\n", PreviousWakeTime);
                        request_speed = 0;
                    }
                    else
                    {
                        static float mm2RPM = 1.0 / (AGV_WheelDiameter * PI) * 60.0;
                        request_speed = (int)(((double)(MotionStatus.handSpeed * mm2RPM) * 512 * 10000 * 9.333333) / 1875);
                    }
                }
                else
                {
                    MotionStatus.handSpeedTime = PreviousWakeTime;
                    request_speed = 0;
                }
            }
            else
            {
                if (agv.iEmergencyByPause)
                {
                    request_speed = (int)(((double)agv.Request_RPM * 512 * 10000 * 9.333333) / 1875);
                    /*
                    if( request_speed == 0 )
                    {
                        if( switchReach != -2 )
                        {
                            canOpenStatus.pollStep = 2;
                            navigationOperationData.cmd = 4;
                            xQueueSend( SwitchBeltTaskQue, &navigationOperationData, 100 );
                        }
                    }
                    */
                }
                else
                {
                    if (agv.Request_RPM == 0)
                        request_speed = 0;
                    else
                        request_speed = (int)(((double)agv.Request_RPM * 512 * 10000 * 9.333333) / 1875);
                }
            }
        }

        if (DebugCtrl.enableRealTimeSpeed)
        {
            static float xSpeedBak, ySpeedBak;
            if (xSpeedBak != agv.Request_Speed)
            {
                xSpeedBak = agv.Request_Speed;
                debugOut(0, "[\t%d] Real-Time Speed: %0.2f mm/s\r\n", PreviousWakeTime, agv.Request_Speed);
            }
        }
        if (abs(agv.AGV_Pos - milagesXBack) > 1)
        {
            milages += abs(agv.AGV_Pos - milagesXBack);
            milagesXBack = agv.AGV_Pos;
            if (DebugCtrl.enableRealTimeEcode)
            {
                debugOut(0, "[\t%d] X - Ecode: %d\tPosition: %0.2f\r\n", PreviousWakeTime, agv.EncoderValue, agv.AGV_Pos);
            }
        }
        if (!MotionStatus.CanDelay)
        {
            if (canOpenStatus.count++ >= 1)
            {
                canOpenStatus.count = 0;
                if (MotionStatus.alarm)
                {
                    if (!MotionStatus.alarmCleanDisable)
                    {
                        if (canOpenStatus.pollStep < 4)
                        {
                            canOpenStatus.pollStep = 4;
                        }
                    }
                }
                if (MotionStatus.startUp)
                {
                    canOpenStatus.pollStep = 0;
                }
                else
                {
                    if (canOpenStatus.pollStep == 0)
                        canOpenStatus.pollStep = 1;
                }
                switch (canOpenStatus.pollStep)
                {
                case 0:
                    if (CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Start_Remote_Node))
                    {
                        canOpenStatus.pollStep++;
                        debugOut(0, "[\t%d] Start CanOpen Node ID:1 [ok]\r\n", osKernelSysTick());
                    }
                    // osDelay(10);
                    break;

                case 1:
                    /*
                    CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Master2Slave_request_4Bit23, 0x60ff, 0, request_speed);
                    if( MotionStatus.startUp )
                        MotionStatus.startUp = false;
                    */
                    if (1)
                    {

                        char rpdoData[8];
                        union {
                            int Data;
                            char Hex[4];
                        } i32ToHex;
                        union {
                            short Data;
                            char Hex[2];
                        } i16ToHex;
                        if (MotionStatus.handSpeedMode)
                        {
                            if (abs(request_speed) > 10)
                            {
                                i32ToHex.Data = request_speed;
                                i16ToHex.Data = 0x0f;
                            }
                            else
                            {
                                if (PreviousWakeTime >= MotionStatus.handSpeedTime)
                                {
                                    if (PreviousWakeTime - MotionStatus.handSpeedTime > 100)
                                    {
                                        i32ToHex.Data = 0;
                                        i16ToHex.Data = 0x06;
                                    }
                                    else
                                    {
                                        i32ToHex.Data = 0;
                                        i16ToHex.Data = 0x0f;
                                    }
                                }
                                else
                                    MotionStatus.handSpeedTime = PreviousWakeTime;
                            }
                        }
                        else
                        {
                            if (agv.iEmergencyByPause)
                            {
                                if (abs(request_speed) < 10)
                                {
                                    i32ToHex.Data = 0;
                                    i16ToHex.Data = 0x06;
                                }
                                else
                                {
                                    i32ToHex.Data = request_speed;
                                    i16ToHex.Data = 0xf;
                                }
                            }
                            else
                            {
                                if (agv.Motion_Status_Now == AGV_Parallel_Motion::ms_Arrived)
                                {
                                    i32ToHex.Data = 0;
                                    i16ToHex.Data = 0x06;
                                }
                                else
                                {
                                    i32ToHex.Data = request_speed;
                                    i16ToHex.Data = 0x0f;
                                }
                            }
                        }
                        for (int i = 0; i < 4; i++)
                            rpdoData[i] = i32ToHex.Hex[i];

                        rpdoData[4] = i16ToHex.Hex[0];
                        rpdoData[5] = i16ToHex.Hex[1];
                        rpdoData[6] = 3;
                        CanTx(0x141, 7, rpdoData);
                    }
                    break;
                case 2:
                    if (MotorModeWord_PDO != 0)
                        CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Master2Slave_request_2Bit2b, 0x6040, 0, 6);
                    break;
                case 3:
                    if (!MotionStatus.enable)
                    {
                        CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Master2Slave_request_2Bit2b, 0x6040, 0, 0xf);
                    }
                    else
                        canOpenStatus.pollStep = 1;
                    break;
                case 4:
                    if ((MotorStatusWord_PDO & 0x08) != 0)
                    {
                        CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Master2Slave_request_2Bit2b, 0x6040, 0, 0x86);
                    }
                    else
                        canOpenStatus.pollStep = 3;
                    break;
                case 5:
                    CANopen_Tx.write(1, CANopenMaster::CANopenRequest::Master2Slave_request_2Bit2b, 0x1017, 0, 1000);
                    canOpenStatus.heartBeatDelay = 0;
                    canOpenStatus.pollStep = 1;
                    break;
                case 6:

                default:
                    canOpenStatus.pollStep = -1;
                    break;
                }
            }
        }
        // todo :
        /* 绝对延时，绝对延时保证代码执行周期固定，即使程序运行时间不确定 此延时依赖FreeRTOS */
        osDelayUntil(&PreviousWakeTime, 2);
    }
}
