#include "MotorCan.h"
#include "stdio.h"
/*****************************************************************

初始化 Can1。
波特率 500k。

输入参数：
    无

返回值：
    0：  初始化未发现错误。
    -1： 滤波器初始化失败。
    -2： Can1 启动失败。
    -3： Can1 接收中断初始化失败。

*******************************************************************/
// static Can1DataDef can1Data;

QueueHandle_t can1Queue = 0;

int Can1Init()
{
    while( can1Queue == 0 )
    {
        can1Queue = xQueueCreate( 20, sizeof( can1DataStd ) );
    }
    // if( MotorQue == 0 )
    //   MotorQue = xQueueCreate( 10, sizeof( MotorCMD ) );
    CAN_FilterTypeDef  sFilterConfig;
    /*## Configure the CAN1 Filter ###########################################*/
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        return -1;
    }
    /** Start the CAN1 peripheral ###########################################*/
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        /* Start Error */
        return -2;
    }
    /** Activate CAN1 RX notification #######################################*/

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Notification Error */
        return -3;
    }
    return 0;
}
/********************************************************************************************
通过 Can1 发送数据，一次发送8字节。
输入参数
    @buff：  8字节数组
    @ID：    目标 ID
返回值：
    -1： 发送失败
    0：  发送成功
***********************************************************************************************/
int SendBuffToCan1( uint8_t *buff, int ID, int dlc)
{
    if( DebugCtrl.enableCanRawData )
    {
        taskENTER_CRITICAL();
        {
            printf( "[\t%d] Can ID: %02X, DLC: %d Data: ",osKernelSysTick(), ID, dlc );
            for( int i = 0; i < dlc; i++ )
            {
                printf( "%02X ", buff[i] );
            }
            printf( "\r\n" );
        }
        taskEXIT_CRITICAL();
    }
    CAN_TxHeaderTypeDef TxHeader1;
    uint32_t    TxMailbox;
    TxHeader1.StdId = ID;
    TxHeader1.IDE = CAN_ID_STD;
    TxHeader1.DLC = dlc;
    TxHeader1.RTR = CAN_RTR_DATA;
    TxHeader1.TransmitGlobalTime = DISABLE;
//    Can1Data.IsRecive = 0;
    if( HAL_CAN_AddTxMessage( &hcan1, &TxHeader1, buff, &TxMailbox ) != HAL_OK )
        return -1;
    else
        return 0;
}

/* Can1 消息接收中断 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BaseType_t wakeUp = pdFAIL;
    can1DataStd can1Data;
    if( HAL_CAN_GetRxFifoFillLevel( &hcan1, CAN_RX_FIFO0 ) > 0 )
    {
        if( HAL_CAN_GetRxMessage( &hcan1, CAN_RX_FIFO0, &can1Data.RxHeader, can1Data.Data ) == HAL_OK )
        {
            if( can1Queue )
            {
                xQueueSendFromISR( can1Queue, &can1Data, &wakeUp );
            }
        }
    }
}
/* Can1 邮箱满中断 */
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    BaseType_t wakeUp = pdFAIL;
    can1DataStd can1Data;
    if( HAL_CAN_GetRxFifoFillLevel( &hcan1, CAN_RX_FIFO0 ) > 0 )
    {
        if( HAL_CAN_GetRxMessage( &hcan1, CAN_RX_FIFO0, &can1Data.RxHeader, can1Data.Data ) == HAL_OK )
        {
            if( can1Queue )
            {
                xQueueSendFromISR( can1Queue, &can1Data, &wakeUp );
            }
        }
    }
}
