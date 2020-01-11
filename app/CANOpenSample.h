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
		//Ԥ�ڻ�Ӧ�ĳ���
//		int ResponseLength(int iIndex);
		typedef union
		{
			int iValue;
			short wValue[2];
			char cValue[4];
		}tu_convert32;//ת��
		typedef union
		{
			short wValue;
			char cValue[2];
		}tu_convert16;//ת��
	};

	class CANopenRequest
	{
	public:
		//�ڵ㱣��
		typedef enum
		{
			Initialising = 0,  //��ʼ��
			Disconnected = 1,//�Ͽ�
			Connecting = 2,//������
			Preparing = 3, //׼��
			Stopped = 4,//ֹͣ
			Operational = 5,//����
			Pre_Operational = 127//Ԥ����
		}te_NMT_request;
		//�ڵ����
		typedef enum
		{
			/*������Operational״̬��������ɺ󣬽ڵ����������еĽڵ�״̬Ҳ��ΪOperational״̬*/
			Start_Remote_Node = 1,
			/*����stop remote node�������Stopped״̬����Ȼ�����������еģ�ֻ�ǽڵ㲻�ɻ���*/
			Stop_Remote_Node = 2,
			Enter_Pre_Operational_State = 128,
			/*�����reset node��������ýڵ㸴λ��
			��λ֮�󣬻����Ƚ���Initializing״̬�����������������е�0x00����
			��ʼ����ɺ󣬽���Pre-Operational״̬����Ӧ���������е�0x7F����
			*/
			Rest_Node = 128,
			Rest_Communication = 130
		}te_NMT_NodeCtrl;
		//ID = 000, ����Ϊһ���ֽڵĵ�NMT״̬ + Node-ID
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
		//�����ķ��ͺ���
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
		}tu_convert32;//ת��
		typedef union
		{
			short wValue;
			char cValue[2];
		}tu_convert16;//ת��
	};			
}
#ifdef __cplusplus 

#endif

#endif
