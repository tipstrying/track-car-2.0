#ifndef AGV_PARALLEL_MOTION_H__
#define AGV_PARALLEL_MOTION_H__

#define PI acos(-1.0)
#define AGV_WheelDiameter 60 //mm
#define AGV_EncoderCPC (10000 * 8.166667)

#include "hardware.h"

class AGV_Parallel_Motion
{
public:
    typedef enum
    {
        ms_Idle = 0, //空闲的
        ms_Straight = 1,
        ms_Error = 3,
        ms_Arrived = 5,
        ms_Cancel = 6,
        ms_Emergency = 7
    } Motion_Status;

    AGV_Parallel_Motion();
    ~AGV_Parallel_Motion();

    float AGV_Pos;
    float AGV_PosNext;

    bool isNewPosition;

    bool iEmergencyBySoftware; // 软件急停
    bool iEmergencyByPause;
    bool iEmergencyByCancel;
    bool iEmergencyByError;
    bool iEmergencyByMotorDisable;

    float Request_Speed;
    float Request_RPM;
    float FeedBack_Speed;
    float FeedBack_RPM;

    float sAcceleration;
    float sDcceleration;
    float sDeceleration_distance;
    float sSpeed_max;

    float sSpeed_min;    //最小线速度 mm/s
    float Stop_Accuracy; //停止精度
    int EncoderValue;    //输入32位编码器
    Motion_Status Motion_Status_Now;
    Motion_Status Motion_work(float iTarget); // 定时调用
    void DetectDynamics(void);                // 周期调用
    int clock;                                //时钟ms
    int sArriveCtrlTime;                      //停稳后要多长时间才认为到达目标点
    Motion_Status getMotionStatus();
    void clearMotionStatusError();

private:
    int sample_Time;
    Motion_Status Move_to(float iTarget);
    float Move(float iDistance);
    Motion_Status move_to_step;
    Motion_Status secondLocaliztionType;
};

#endif
