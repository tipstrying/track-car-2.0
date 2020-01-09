#ifndef BELTDRIVECONTROLLER_H__
#define BELTDRIVECONTROLLER_H__
class BeltDriveController
{
public:
	typedef enum
	{
		Push_idle = 0,
		Push_front,
		Push_real,
		Push_none
	}te_control_step;
	class parameters
	{
	public:
		parameters()
		{}
		~parameters()
		{}

		bool read_input[3];
		bool motor_direction;
		bool motor_stop;
		int clock;
		int sArriveCtrlTime;//转动多久才停止
		bool  iEmergency; //急停

	};
	parameters info;
	BeltDriveController();
	~BeltDriveController();
	void work(void);
	te_control_step get_motion_step(void);
	bool  motion_ctrl(te_control_step iControlMode);

private:
	void  motion_ctrl(void);
	void   motor_ctrl(te_control_step iControlMode);
	te_control_step motion_control_step;
};


#endif
