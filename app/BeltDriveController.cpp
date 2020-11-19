#include "BeltDriveController.h"

BeltDriveController::BeltDriveController()
{
	this->info.clock = 0;
	this->info.motor_direction = false;
	for (int i = 0; i < 2; i++)
	{
		this->info.read_input[i] = false;
	}
	this->info.motor_stop = false;
	this->motion_control_step = Push_idle;
	this->info.motor_stop = false;
	this->info.iEmergency = false;
	this->info.sArriveCtrlTime = 10;
}

BeltDriveController::~BeltDriveController()
{

}

void BeltDriveController::work(void)
{
	this->motion_ctrl();
}

bool BeltDriveController::motion_ctrl(te_control_step iControlMode)
{
	bool rValue = false;
	if (iControlMode == Push_none)
		return rValue;
	if (this->motion_control_step == Push_idle || this->motion_control_step == Push_none)
	{
		rValue = true;
		this->motion_control_step = iControlMode;
	}
	return rValue;
}

BeltDriveController::te_control_step BeltDriveController::get_motion_step(void)
{
	return this->motion_control_step;
}

void BeltDriveController::motion_ctrl(void)
{
	static int last_time_ctrl = this->info.clock;
	switch (this->motion_control_step)
	{
	case Push_idle:
		if (!this->info.read_input[0] && this->info.read_input[1])
			this->motion_control_step = Push_none;
		else if (this->info.read_input[0] && !this->info.read_input[1])
			this->motion_control_step = Push_none;
		this->motor_ctrl(Push_idle);
		break;
	case Push_front:
		if (!this->info.read_input[0] && !this->info.read_input[1])
		{
			if (this->info.clock - last_time_ctrl > this->info.sArriveCtrlTime)
			{
				this->motor_ctrl(Push_idle);
				this->motion_control_step = Push_idle;
			}
		}
		else
		{
			this->motor_ctrl(Push_front);
			last_time_ctrl = this->info.clock;
		}
		break;
	case Push_real:
		if (!this->info.read_input[0] && !this->info.read_input[1])
		{
			if (this->info.clock - last_time_ctrl > this->info.sArriveCtrlTime)
			{
				this->motor_ctrl(Push_idle);
				this->motion_control_step = Push_idle;
			}
		}
		else
		{
			this->motor_ctrl(Push_real);
			last_time_ctrl = this->info.clock;
		}
		break;

	case Push_none:
		if (!this->info.read_input[0] && this->info.read_input[1])
		{
			last_time_ctrl = this->info.clock;
			this->motor_ctrl(Push_real);
		}
		else if (this->info.read_input[0] && !this->info.read_input[1])
		{
			last_time_ctrl = this->info.clock;
			this->motor_ctrl(Push_front);
		}
		else
		{
			if ((!this->info.read_input[0] && !this->info.read_input[1]))
			{
				//如果物体太小那就多转一段距离，在检测看看，这个距离只能用时间来表示500ms
				if (this->info.clock - last_time_ctrl > 100)
				{
					this->motor_ctrl(Push_idle);
					this->motion_control_step = Push_idle;
				}
			}
			else
			{
				//到位了就直接停下来
				this->motor_ctrl(Push_idle);
				this->motion_control_step = Push_idle;
			}
		}
		break;
	}
}

void BeltDriveController::motor_ctrl(te_control_step iControlMode)
{
	switch (iControlMode)
	{
	case Push_idle:
		this->info.motor_stop = true;
		break;
	case Push_front:
		this->info.motor_stop = false;
		this->info.motor_direction = true;
		break;
	case Push_real:
		this->info.motor_stop = false;
		this->info.motor_direction = false;
		break;
	default:
		this->info.motor_stop = true;
		break;
	}
	if(this->info.iEmergency)
		this->info.motor_stop = true;
}
