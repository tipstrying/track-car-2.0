#include "AGV_Parallel_Motion.h"
#include <math.h>
#include <stdio.h>
#include "hardware.h"
#include "app.h"

AGV_Parallel_Motion::AGV_Parallel_Motion()
{
    this->sample_Time = 0;
    this->sSpeed_min = 10.0;
    this->FeedBack_RPM = 0;
    this->FeedBack_Speed = 0;
    this->Request_RPM = 0;
    this->Request_Speed = 0;
    this->Stop_Accuracy = 1;
    this->sArriveCtrlTime = 300; // 20 * sample_Time  ms
    this->sAcceleration = 1500;
    this->sDeceleration_distance = 5;

    this->EncoderValue = 0;
    this->move_to_step = ms_Idle;
    iEmergencyByKey = false;
    iEmergencyBySoftware = false;

}

AGV_Parallel_Motion::~AGV_Parallel_Motion()
{
}

AGV_Parallel_Motion::Motion_Status AGV_Parallel_Motion::Motion_work(float iTarget)
{
    Motion_Status rValue = ms_Idle;
    this->DetectDynamics();
    if (this->iEmergencyByKey || this->iEmergencyBySoftware || this->iEmergencyByError || this->iEmergencyByPause || this->iEmergencyByCancel )
    {
        this->Request_RPM = 0;
        this->Request_Speed = 0;
    }
    else
    {
        rValue = this->Move_to(iTarget);
    }
    return rValue;
}

AGV_Parallel_Motion::Motion_Status AGV_Parallel_Motion::Move_to(float iTarget)
{
    const static float sPI = PI;
    static float mm2RPM = 1.0 / (AGV_WheelDiameter * sPI) * 60.0;
    Motion_Status rValue = ms_Idle;
    static int lastTimeCtrl = this->clock;
    float iDistance = fabsf( this->AGV_Pos - iTarget );
    static Motion_Status move_to_step_bak = move_to_step;
    switch( this->move_to_step )
    {

    default:
    case ms_Arrived:
    case ms_Emergency:
    case ms_Cancel:
        break;

    case ms_Idle:
        move_to_step_bak = move_to_step;
        if (iDistance < this->Stop_Accuracy)
        {
            if (this->clock - lastTimeCtrl > this->sArriveCtrlTime)
            {
                rValue = ms_Arrived;
            }
            this->Request_Speed = 0;
        }
        else
        {
            this->move_to_step = ms_Straight;
            return ms_Straight;
        }
        break;
    case ms_Straight:
        move_to_step_bak = move_to_step;

        if( fabsf(iTarget - this->AGV_Pos) > this->Stop_Accuracy * 0.5)
        {
            lastTimeCtrl = this->clock;
            if( this->AGV_Pos > iTarget )
            {
                this->Request_Speed = -this->Move(iDistance);
            }
            else
            {
                this->Request_Speed = this->Move(iDistance);
            }
        }
        else
        {
            lastTimeCtrl = this->clock;
            this->Request_Speed = 0.0f;

            if (this->clock - lastTimeCtrl > 10)
            {
                taskENTER_CRITICAL();
                {
                    printf("[\t%d] Motion Finish at %f mm\r\n", this->clock, this->AGV_Pos);
                }
                taskEXIT_CRITICAL();
                this->move_to_step = ms_Idle;
            }

        }
        rValue = ms_Straight;
        break;
    case ms_Error:
        rValue = ms_Error;
        break;

    }
    this->Request_RPM = this->Request_Speed * mm2RPM;
    return rValue;
}

float AGV_Parallel_Motion::Move(float iDistance)
{
    float SetSpeed = 0.000;
    static float speedNow = 0;
    static float speedOld = 0;
    float stopDistance;

    stopDistance = (speedNow * speedNow) / (2.0 * sAcceleration) + sDeceleration_distance;
    if (abs(iDistance) > stopDistance)
    {
        SetSpeed = sSpeed_max;
    }
    else
    {
        SetSpeed = sSpeed_min;
    }
    if (abs(iDistance) <= Stop_Accuracy /* * 0.5 */)
    {
        SetSpeed = sSpeed_min;
    }
    if (iDistance < 0)
        SetSpeed = -SetSpeed;

    float securityDelta_straight;
    if (speedNow > SetSpeed)
    {
        securityDelta_straight = sAcceleration * sample_Time / 1000.0;
    }
    else
    {
        securityDelta_straight = sAcceleration * sample_Time / 1000.0;
    }
    if (speedNow > SetSpeed)
    {
        if( stopDistance > iDistance )
        {
            if( iDistance > sDeceleration_distance )
                speedNow = sqrt( 2 * sAcceleration * (iDistance - sDeceleration_distance) );
            else
                speedNow = sSpeed_min;
        }
        else
        {
            if( speedNow > sSpeed_max )
            {
                speedNow -= securityDelta_straight;
            }
        }
    }
    else if (speedNow < SetSpeed)
        speedNow += securityDelta_straight;
    return speedNow;
}

void AGV_Parallel_Motion::DetectDynamics(void)
{
    taskENTER_CRITICAL();
    {
        const static float sPI = PI;
        static int lastEncoderValue;
        static int encoder_delta;

        static int lastTimeCtrl = this->clock;
        this->sample_Time = this->clock - lastTimeCtrl;
        lastTimeCtrl = this->clock;

        int temp_encoder_value = this->EncoderValue;
        encoder_delta = temp_encoder_value - lastEncoderValue;
        lastEncoderValue = temp_encoder_value;

        static float Encoder2mm = AGV_WheelDiameter * sPI / AGV_EncoderCPC;

        static float moveingDelta; // mm
        moveingDelta = encoder_delta * Encoder2mm;

        this->AGV_Pos += moveingDelta;
    }
    taskEXIT_CRITICAL();
}


AGV_Parallel_Motion::Motion_Status AGV_Parallel_Motion::getMotionStatus()
{
    return this->move_to_step;
}
void AGV_Parallel_Motion::clearMotionStatusError()
{

    if (this->move_to_step == AGV_Parallel_Motion::ms_Error)
    {
        this->move_to_step = ms_Idle;
    }
}