#include "CANopenSample.h"
/*
CAN - ID左电机 601 右电机 607
	ready to switch on 电机停止转动会松开刹车
	<-601  2B 40 60 00 06 00 00 00
	->851  60 40 60 00 00 00 00 00
	switch on 电机停止转动会保持刹车
		< -601 2B 40 60 00 07 00 00 00
		->581 60 40 60 00 00 00 00 00
		使能或者停止电机
		< -601 2B 40 60 00 0F 00 00 00
		->581 60 40 60 00 00 00 00 00
		设置为速度模式
		< -601 2F 60 60 00 03 00 00 00
		->581 60 60 60 00 03 00 00 00
		设置目标速度1RPS
		设定值 * 240 / 60 = RPM
		< -601 23 FF 60 00 F0 00 00 00
		->581 60 FF 60 00 00 00 00 00
		设置加速度0x0258 / 60 rps
		< -601 23 83 60 00 58 02 00 00
		->581 60 83 60 00 00 00 00 00 00
		设置减速度0x0258 / 60 rps
		< -601 23 84 60 00 58 02 00 00
		->581 60 84 60 00 00 00 00 00 00
		开始运动
		< -601 2B 40 60 00 0F 00 00 00
		->581 60 40 60 00 00 00 00 00
		改变目标速度10rps
		< -601 23 FF 60 00 FF 00 00 00
		->581 60 FF 60 00 00 00 00 00
		运动停止 会保持刹车
		< -601 2B 40 60 00 0F 01 00 00
		->581 60 40 60 00 00 00 00 00
		获取绝对值编码器32位
		< -601 40 0A 70 00 00 00 00 00
		->581 43 0A 70 00 01 02 03 04
		获取转速rps
		< -601 40 09 70 00 00 00 00 00
		-> 581 4B 09 70 00 01 02 00 00
    < -601 2B 17 10 00 EB 03 00 00 //修改心跳时间为1000ms

*/

namespace CANopenMaster
{
CANopenResponse::CANopenResponse()
{
    this->time_ctrl = this->clock_time;
}

CANopenResponse::~CANopenResponse()
{
}
bool CANopenResponse::work()
{

    bool rValue = false;
    if (this->CheckTimeOut())
    {
        //printf("[Debug] [CANopen Error]  ID:  %d/r/n", this->Node_ID);
    }
    int oID, oLength;
    char oArray[8];
    if (this->Event_Rx_Work)
    {
        while (this->Event_Rx_Work(&oID, &oLength, oArray))
        {
            if (oID == 0)
            {
                //这是slave 做为心跳包来使用的，收到它说明有总线上挂着另外一个master
                //error
                //printf("[ Debug ]  [CANopenError] there are same  CANopen-maser in this bus /r/n");
            }
            else
            {
                //非例外的状况
                if (!this->CheckException(oID, oLength))
                {
                    int Node_IDx = oID % 0x80;
                    int IDx = oID - Node_IDx;
                    switch (IDx)
                    {
                    case 0x0080://紧急事件
                        if (this->Event_Rx_Emergency_Complete)
                        {
                            short iError;
                            char iRegister;
                            int iError_Area;
                            if (this->CheckRead_regs(oArray,oLength, iError, iRegister, iError_Area,false))
                            {
                                this->Event_Rx_Emergency_Complete(Node_IDx,iError,iRegister,iError_Area);
                                rValue = false;
                            }
                        }
                        break;
                    case 0x0180://pdo1-tx
                        if( this->Event_Rx_PDO_Complete )
                        {
                            short oIndex;
                            char oSubindex = 0;
                            int oData;
                            char DataMap[] = { 4, 0, 0 };
                            if(this->CheckRead_regs_PDO(oArray, &oLength, &oIndex, &oData, DataMap))
                            {
                                this->Event_Rx_PDO_Complete(oID, oIndex, oSubindex, oData);
                                rValue = true;
                            }
                        }
                        break;
                    case 0x0200://pdo1-rx
                        break;
                    case 0x0280://pdo2-tx

                        if( this->Event_Rx_PDO_Complete )
                        {
                            short oIndex;
                            char oSubindex = 0;
                            int oData;
                            char DataMap[] = { 2, 2, 2 };
                            if(this->CheckRead_regs_PDO(oArray, &oLength, &oIndex, &oData, DataMap))
                                this->Event_Rx_PDO_Complete(oID, oIndex, oSubindex, oData);
                        }

                        break;
                    case 0x0300://pdo2-rx
                        break;
                    case 0x0380://pdo3-tx
                        break;
                    case 0x0400://pdo3-rx
                        break;
                    case 0x0480://pdo3-tx
                        break;
                    case 0x0500://pdo3-rx
                        break;
                    case 0x0580://sdo(发送的服务)
                        if (this->Event_Rx_SDO_Complete)
                        {
                            short oIndex;
                            char oSubindex;
                            int oData;
                            if(this->CheckRead_regs(oArray, &oLength, &oIndex, &oSubindex, &oData))
                            {
                                this->Event_Rx_SDO_Complete(Node_IDx, oIndex, oSubindex, oData);
                                rValue = false;
                            }
                        }
                        break;
                    case 0x0600:
                        break;
                    case 0x0700://nmt
                        if (this->Event_Rx_HeartBeat_Complete)
                        {
                            te_HeartBeat get_status;
                            if(this->CheckRead_regs(oArray,&oLength, &get_status))
                            {
                                this->Event_Rx_HeartBeat_Complete(Node_IDx, get_status);
                                rValue = false;
                            }
                        }
                        break;
                    }
                }
            }
        }
    }
    return rValue;
}

bool CANopenResponse::CheckRead_regs(char iArray[8], int *iLength, short *oIndex, char *oSubIndex, int *oData)
{

    CANopenResponse::tu_convert16 iTool16;
    CANopenResponse::tu_convert32 iTool32;

    //SDO(发送的服务)
    for (int i = 0; i < 2; i++)
    {
        iTool16.cValue[i] = iArray[i + 1];
    }
    short index = iTool16.wValue;
    char  subindex = iArray[3];
    for (int i = 0; i < 4; i++)
    {
        iTool32.cValue[i] = iArray[i + 4];
    }
    int iData = iTool32.iValue;
    *oIndex = index;
    *oSubIndex = subindex;
    *oData = iData;
    bool rValue = false;
    if( *iLength == 8 )
    {
        rValue = true;
        this->time_ctrl = this->clock_time;
    }
    return rValue;
}
bool CANopenResponse::CheckRead_regs_PDO(char iArray[8], int *iLength, short *oIndex, int *oData, char * DataMap)
{

    CANopenResponse::tu_convert16 iTool16;
    CANopenResponse::tu_convert32 iTool32;

    if( DataMap[0] == *iLength )
    {
        if( DataMap[0] == 4 )
        {
            for (int i = 0; i < 4; i++)
            {
                iTool32.cValue[i] = iArray[i];
            }
            *oData = iTool32.iValue;
            this->time_ctrl = this->clock_time;
            return true;
        }
    }
    else
    {
        /*
        if( DataMap[0] == 2 )
        {
            for( int i = 0; i < 2; i++ )
            {
                iTool16.cValue[i] = iArray[i];
            }
            *oIndex = iTool16.wValue;
            if( *iLength > DataMap[0] )
            {

            }
        }
        */
        for( int j = 0; j < 2; j++ )
        {
            iTool16.cValue[j] = iArray[j];
        }
        *oIndex = iTool16.wValue;
        for( int i = 0; i < 4; i++ )
        {
            iTool32.cValue[i] = iArray[i+2];
        }
        *oData = iTool32.iValue;
        return true;
    }
}
bool CANopenResponse::CheckRead_regs(  char iArray[8], int *iLength, te_HeartBeat *oStatus)
{
    bool rValue = false;
    if ( *iLength == 1 )
    {
        *oStatus = (te_HeartBeat)iArray[0];
        rValue = true;
        this->time_ctrl = this->clock_time;
    }
    return rValue;
}

bool CANopenResponse::CheckRead_regs(char iArray[8], int iLength, short oError, char  oRegiter, int oError_Area, bool iStop_All)
{
    CANopenResponse::tu_convert16 iTool16;
    CANopenResponse::tu_convert32 iTool32;
    for (int i = 0; i < 2; i++)
    {
        iTool16.cValue[i] = iArray[i + 1];
    }
    oError = iTool16.wValue;
    oRegiter = iArray[2];
    for (int i = 0; i < 5; i++)
    {
        iTool32.cValue[i] = iArray[i + 3];
    }
    oError_Area = iTool32.iValue;
    bool rValue = false;
    if (iLength == 8)
    {
        rValue = true;
        this->time_ctrl = this->clock_time;
    }
    return rValue;
}

bool CANopenResponse::CheckTimeOut()
{
    bool rValue = false;
    if (this->clock_time - this->time_ctrl > 500)
    {
        rValue = true;
    }
    return rValue;
}

bool CANopenResponse::CheckException(int iFc_ID, int iLength)
{
    bool rValue = false;
    switch (iFc_ID)
    {
    case 0x0000://NMT控制模块
        rValue = true;
        break;
    case 0x0080://SYNC同步
        rValue = true;
        break;
    case 0x0100://时间标记
        rValue = true;
        break;
    }
    return rValue;
}


CANopenRequest::CANopenRequest()
{
    this->polling_step = 0;
}

CANopenRequest::~CANopenRequest()
{
}

int CANopenRequest::RequesLength(char iIndex)
{
    int rValue = 0;
    switch (iIndex)
    {
    case 0x2f://M->S 1字节
        rValue = 1;
        break;
    case 0x2b://M->S 2字节
        rValue = 2;
        break;
    case 0x27://S->M 3字节
        rValue = 3;
        break;
    case 0x23://M->M 4字节
        rValue = 4;
        break;
    case 0x40://M->S 0字节
        rValue = 0;
        break;
    }
    return rValue;
}

void CANopenRequest::work()
{
//		static int lastTimeCtrl = this->clock_time;
//		if( this->write_buffer.Size() > 0 )
//		{
//			if( this->clock_time - lastTimeCtrl > 5 )
//			{
//				Write_Data_Type get_value =  this->write_buffer[0];
//				if( this->Event_Tx_Work )
//				{
//					this->Event_Tx_Work(get_value.ID,get_value.Length,get_value.Data);
//					this->write_buffer.Erase(0);
//				}
//				lastTimeCtrl = this->clock_time;
//			}
//		}
//		if (this->initial())
//			this->polling();
}

//NMT id = 0x700 + node_id
bool CANopenRequest::write(int iNode_ID, te_NMT_NodeCtrl iRequest)
{
    bool rValue = false;
    char write_array[8];
    write_array[0] = (char)iRequest;
    write_array[1] =  iNode_ID;
    if (this->Event_Tx_Work)
        rValue = this->Event_Tx_Work(0, 2, write_array);
    return rValue;
}

//SDO_Request id = 0x600 +node_id
bool CANopenRequest::write(int Node_ID, te_SDO_request_cmd icmd, short iIndex, char iIndex_sub, int  iData)
{
    char write_array[8];
    int write_length = 8;//this->RequesLength(icmd);
    write_array[0] = (char)icmd;
    CANopenRequest::tu_convert16 iTool16;
    iTool16.wValue = iIndex;
    for (int i = 0; i < 2; i++)
    {
        /*-----------------1 - 2 低位在前-------------------*/
        write_array[i + 1] = iTool16.cValue[i];
    }
    /*------------------------3----------------------------*/
    write_array[3] = iIndex_sub;
    CANopenRequest::tu_convert32 iTool32;
    iTool32.iValue = iData;
    for (int i = 0; i < 4; i++)
    {
        /*-----------------4 - 7低位在前-------------------*/
        write_array[i + 4] = iTool32.cValue[i];
    }
    bool rValue = false;
    if (this->Event_Tx_Work)
    {
        rValue = this->Event_Tx_Work(Node_ID + 0x0600, write_length, write_array);
    }
    return rValue;
}



bool CANopenRequest::write(int iNode_ID, short iData)
{
    char write_array[8];
    CANopenRequest::tu_convert16 iTool16;
    iTool16.wValue = iData;
    for (int i = 0; i < 2; i++)
    {
        /*-----------------1 - 2 低位在前-------------------*/
        write_array[i] = iTool16.cValue[i];
    }
    bool rValue = false;
    if (this->Event_Tx_Work)
    {
        rValue = this->Event_Tx_Work(iNode_ID + 0x0280, 2, write_array);
    }
    return rValue;
}

void CANopenRequest::polling(char iSize,...)
{
    va_list argptr;
    va_start( argptr, iSize );
    for( int i = 0; i < iSize; i ++ )
    {
        int Node_ID = va_arg( argptr, int );
        this->polling( Node_ID );
    }
    va_end( argptr );
}


void CANopenRequest::polling( int iNode_ID)
{
    switch( this->polling_step )
    {
    case 8:
        //读编码器
        if(this->write(iNode_ID, Master2Slave_request_0Bit40, 0x0a40, 0, 0))
            this->polling_step = 9;
        break;
    //读速度
    //this->write(iNode_ID, Master2Slave_request_0Bit40, 0x7009, 0, 0);
    //写速度	rps
    //RPM * 240 / 60 = RPS
    case 9:
        if(this->write(iNode_ID, Master2Slave_request_4Bit23, 0x60ff, 0, this->set_speed))
            this->polling_step = 10;
        break;
    }
}
bool CANopenRequest::initialzationPDO( int iNode_ID )
{
    static int last_Node_ID = 0;
    if( iNode_ID == 0 && iNode_ID > 127)
        return false;
    if( iNode_ID != last_Node_ID )
    {
        last_Node_ID = iNode_ID;
        this->polling_step = 0;
    }
    switch (this->polling_step)
    {
    case 0:
        if( write( iNode_ID, Master2Slave_request_4Bit23, 0x1800, 0x01, 0x80000180 + iNode_ID ) )
            polling_step++;
        break;
    case 1:
        if( write( iNode_ID, Master2Slave_request_1Bit2f, 0x1800, 0x02, 0xff ) )
            polling_step++;
        break;
    case 2:
        if( write( iNode_ID, Master2Slave_request_2Bit2b, 0x1800, 0x03, 0x64 ) )
            polling_step++;
        break;
    case 3:
        if( write( iNode_ID, Master2Slave_request_2Bit2b, 0x1800, 0x05, 0x0A ) )
            polling_step++;
        break;
    case 4:
        if( write( iNode_ID, Master2Slave_request_1Bit2f, 0x1A00, 0x00, 0x00 ) )
            polling_step++;
        break;
    case 5:
        if( write( iNode_ID, Master2Slave_request_4Bit23, 0x1A00, 0x01, 0x700A0020 ) )
            polling_step++;
        break;
    case 6:
        if( write( iNode_ID, Master2Slave_request_1Bit2f, 0x1A00, 0x00, 0x01 ) )
            polling_step++;
        break;
    case 7:
        if( write( iNode_ID,Master2Slave_request_4Bit23, 0x1800, 0x01, 0x180 + iNode_ID ) )
            polling_step++;
        break;
    case 8:
        if( write( iNode_ID, Master2Slave_request_4Bit23, 0x1801, 0x01, 0x80000280 + iNode_ID ) )
            polling_step++;
        break;
    case 9:
        if( write( iNode_ID, Master2Slave_request_1Bit2f, 0x1801, 0x02, 0xff ) )
            polling_step++;
        break;
    case 10:
        if( write( iNode_ID, Master2Slave_request_2Bit2b, 0x1801, 0x05, 0x14 ) )
            polling_step++;
        break;
    case 11:
        if( write( iNode_ID, Master2Slave_request_1Bit2f, 0x1A01, 0x00, 0x00 ) )
            polling_step++;
        break;
    case 12:
        if( write( iNode_ID, Master2Slave_request_4Bit23, 0x1A01, 0x01, 0x603f0010 ) )
            polling_step++;
        break;
    case 13:
        if( write( iNode_ID, Master2Slave_request_4Bit23, 0x1A01, 0x02, 0x60410010 ) )
            polling_step++;
        break;
    case 14:
        if( write( iNode_ID, Master2Slave_request_4Bit23, 0x1A01, 0x03, 0x700B0010 ) )
            polling_step++;
        break;
    case 15:
        if( write( iNode_ID, Master2Slave_request_1Bit2f, 0x1A01, 0x00, 0x03 ) )
            polling_step++;
        break;
    case 16:
        if( write( iNode_ID, Master2Slave_request_4Bit23, 0x1801, 0x01, 0x280 + iNode_ID ) )
            polling_step++;
        break;
    case 17:
        if( write( iNode_ID, Master2Slave_request_4Bit23, 0x1802, 0x01, 0x80000380 + iNode_ID ) )
            polling_step++;
        break;
    case 18:
        if( write( iNode_ID, Master2Slave_request_4Bit23, 0x1803, 0x01, 0x80000480 + iNode_ID ) )
            polling_step++;
        break;
    }
    return (this->polling_step > 18 );
}
bool CANopenRequest::initialzation( int iNode_ID )
{
    static int last_Node_ID = 0;
    if( iNode_ID == 0 && iNode_ID > 127)
        return false;
    if( iNode_ID != last_Node_ID )
    {
        last_Node_ID = iNode_ID;
        this->polling_step = 0;
    }
    switch (this->polling_step)
    {
    case 0:
        if (this->write(iNode_ID, Master2Slave_request_2Bit2b, 0x6040, 0, 0x00000006))
            this->polling_step++;
        break;
    case 1:
        if (this->write(iNode_ID, Master2Slave_request_1Bit2f, 0x6060, 0, 0x00000003))
            this->polling_step++;
        break;
    case 2:
        if( /* iNode_ID == 1 || iNode_ID == 2 */ 0 )
        {
            if (this->write(iNode_ID, Master2Slave_request_2Bit2b, 0x6040, 0, 0x0000006))
                this->polling_step++;
        }
        else
        {
            if (this->write(iNode_ID, Master2Slave_request_2Bit2b, 0x6040, 0, 0x000000f))
                this->polling_step++;
        }
        break;
    case 3:
        if (this->write(iNode_ID, Master2Slave_request_4Bit23, 0x60ff, 0, 0x00000000))
            this->polling_step++;
        break;
    }
    return (this->polling_step > 3);
}
bool CANopenRequest::InitialisingWithOutAcc( int iNode_ID )
{
    static int last_Node_ID = 0;
    if( iNode_ID == 0 && iNode_ID > 127)
        return false;
    if( iNode_ID != last_Node_ID )
    {
        last_Node_ID = iNode_ID;
        this->polling_step = 0;
    }
    switch (this->polling_step)
    {
    case 0:
        if (this->write(iNode_ID, Master2Slave_request_2Bit2b, 0x6040, 0, 0x00000006))
            this->polling_step++;
        break;
//    case 1:
//        if (this->write(iNode_ID, Master2Slave_request_2Bit2b, 0x6040, 0, 0x00000007))
//            this->polling_step++;
//        break;
//    case 2:
//        if (this->write(iNode_ID, Master2Slave_request_2Bit2b, 0x6040, 0, 0x0000010f))
//            this->polling_step++;
//        break;
    case 1:
        if (this->write(iNode_ID, Master2Slave_request_1Bit2f, 0x6060, 0, 0x00000003))
            this->polling_step++;
        break;
//    case 4:
//        if (this->write(iNode_ID, Master2Slave_request_4Bit23, 0x60ff, 0, 0x000000ff))
//            this->polling_step++;
//        break;
    case 2:
        if (this->write(iNode_ID, Master2Slave_request_2Bit2b, 0x6040, 0, 0x000000f))
            this->polling_step++;
        break;
    }
    return (this->polling_step > 2);
}
}
