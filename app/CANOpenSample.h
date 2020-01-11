#ifndef __CANOPENSAMPLE_H
#define __CANOPENSAMPLE_H
#include "stdarg.h"
#include "assert.h"
#include "stddef.h"

namespace CANopenMaster
{
	class CANopenResponse
	{
	public:
		typedef enum
		{
			Boot_Up = 0,
			Stopped = 4,
			Operational = 5,
			Pre_Operational = 127
		}te_HeartBeat;

		CANopenResponse();
		~CANopenResponse();

		bool work();
		bool(*Event_Rx_Work)(int *oID, int *oLength, char oArray[8]);
		void(*Event_Rx_SDO_Complete)(int oID, int oIndex, char oSubindex, int oValue);
        void(*Event_Rx_PDO_Complete)( int oID, char Array[8] );
		// *********I am still alive*******
		void (*Event_Rx_HeartBeat_Complete)(int oID, te_HeartBeat oStatus);
		void(*Event_Rx_Emergency_Complete)(int oID, short oError, char oRegiter, int oError_Area);
		int clock_time;
	private:	
		bool CheckRead_regs( char iArray[8], int *iLength, short *oIndex, char *oSubIndex, int *oData);
        bool CheckRead_regs_PDO(char iArray[8], int *iLength, short *oIndex, int *oData, int ID);
		bool CheckRead_regs( char iArray[8], int *iLength, te_HeartBeat *oStatus);
		bool CheckRead_regs( char iArray[8], int iLength, short oError,char  oRegiter,int oError_Area, bool iStop_All);
		int time_ctrl;
		bool CheckTimeOut();
		bool CheckException(int iFc_ID, int iLength);
		//预期回应的长度
//		int ResponseLength(int iIndex);
		typedef union
		{
			int iValue;
			short wValue[2];
			char cValue[4];
		}tu_convert32;//转换
		typedef union
		{
			short wValue;
			char cValue[2];
		}tu_convert16;//转换
	};

	class CANopenRequest
	{
	public:
		//节点保护
		typedef enum
		{
			Initialising = 0,  //初始化
			Disconnected = 1,//断开
			Connecting = 2,//已连接
			Preparing = 3, //准备
			Stopped = 4,//停止
			Operational = 5,//操作
			Pre_Operational = 127//预操作
		}te_NMT_request;
		//节点控制
		typedef enum
		{
			/*，进入Operational状态，运行完成后，节点心跳报文中的节点状态也变为Operational状态*/
			Start_Remote_Node = 1,
			/*发送stop remote node命令，进入Stopped状态，当然，心跳还是有的，只是节点不干活了*/
			Stop_Remote_Node = 2,
			Enter_Pre_Operational_State = 128,
			/*这个是reset node命令，用于让节点复位。
			复位之后，会首先进入Initializing状态（对于于心跳报文中的0x00），
			初始化完成后，进入Pre-Operational状态（对应心跳报文中的0x7F）。
			*/
			Rest_Node = 128,
			Rest_Communication = 130
		}te_NMT_NodeCtrl;
		//ID = 000, 数据为一个字节的的NMT状态 + Node-ID
		//ID  = 0x700 + Node-ID
		typedef enum
		{
			Master2Slave_request_0Bit40 = 0x40,
			Master2Slave_request_1Bit2f = 0x2f,
			Master2Slave_request_2Bit2b = 0x2b,
			Master2Slave_request_3Bit27 = 0x27,
			Master2Slave_request_4Bit23 = 0x23
		}te_SDO_request_cmd;

		CANopenRequest();
		~CANopenRequest();
		void work();
		//NMT id = 0 
		bool write(int iNode_ID, te_NMT_NodeCtrl iRequest);
		bool write(int iNode_ID, te_SDO_request_cmd icmd, short iIndex, char iSubindex, int  iData);		//SDO_Request id = 0x600 +node_id 
		bool write(int iNode_ID, short iData);		//PDO_Request id= 0x0280 + node_id
		//传出的发送函数
		bool(*Event_Tx_Work)(int iID, int iLength, char iArray[8]);
		int clock_time;
		int set_speed;		
		bool initialzation(int iNode_ID);
        bool initialzationPDO(int iNode_ID);
        bool InitialisingWithOutAcc( int iNode_ID );
        void polling(char iSize, ...);	
		void polling(int iNode_ID);
	private:
		int polling_step;
	  
		int RequesLength(char iIndex);
		//char RequestLength2Cmd(char iCmd);
//	  class Write_Data_Type
//		{
//			public:
//			Write_Data_Type();
//			~Write_Data_Type();
//			int ID;
//			int Length;
//			char Data[8];
//		};
//		Vector< Write_Data_Type > write_buffer;
		typedef union
		{
			int iValue;
			short wValue[2];
			char cValue[4];
		}tu_convert32;//转换
		typedef union
		{
			short wValue;
			char cValue[2];
		}tu_convert16;//转换
	};			
}
#ifdef __cplusplus 

#endif

#endif
