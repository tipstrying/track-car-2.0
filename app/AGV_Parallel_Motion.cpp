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
    this->ts_curve_k = 700;
    this->ts_curve_j_max = 1200;
    this->ts_curve_a_max = 500;
    this->ts_curve_v_max = 500;

    this->sAcceleration = 3000;
    this->sAccelerationLow = 1500;
    this->sSpeed_max = 500;
    this->sSpeed_mid = 150;
    this->sDeceleration_distance = 5;

    this->EncoderValue = 0;
    this->move_to_step = ms_Idle;
    iEmergency = false;

}

AGV_Parallel_Motion::~AGV_Parallel_Motion()
{
}

AGV_Parallel_Motion::Motion_Status AGV_Parallel_Motion::Motion_work(float iTarget)
{
    Motion_Status rValue = ms_Idle;
    this->DetectDynamics();
    if (this->iEmergency)
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
#if 1
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
#else
    if (this->move_to_step == ms_Idle)
    {
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
    }
    else if (this->move_to_step == ms_Straight)
    {
        move_to_step_bak = move_to_step;

        if( fabsf(iTarget - this->AGV_Pos) > this->Stop_Accuracy * 0.5)
        {
            lastTimeCtrl = this->clock;
            if( this->AGV_Pos > iTarget )
            {
                this->Request_Speed = this->Move(iDistance);
            }
            else
            {
                this->Request_Speed = -this->Move(iDistance);
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
    }
    else if (this->move_to_step == ms_Error)
    {
        rValue = ms_Error;
    }
#endif
    this->Request_RPM = this->Request_Speed * mm2RPM;
    return rValue;
}
void AGV_Parallel_Motion::set_ts_curve(float k, float j_max, float a_max, float v_max)
{
    this->ts_curve_k = k;
    this->ts_curve_j_max = j_max;
    this->ts_curve_a_max = a_max;
    this->ts_curve_v_max = v_max;
}
void AGV_Parallel_Motion::get_ts_curve(float *k, float *j_max, float *a_max, float *v_max)
{
    *k = this->ts_curve_k;
    *j_max = this->ts_curve_j_max;
    *a_max = this->ts_curve_a_max;
    *v_max = this->ts_curve_v_max;
}
float AGV_Parallel_Motion::ts_curve(unsigned char replan, unsigned int pMs, float Vstart, float k, float j_max, float a_max, float v_max, float s)
{
    static double lenS1;
    static double lenS2;
    static double stopS;
    static double Vend;
    static double v_sub;
    unsigned int t;
    static double Tstart;
    static double tempVstart;
    static unsigned int t1;
    static unsigned int t2;
    static unsigned int t3;
    float y;
    float a;
    float tt, tt1, tt2;
    float a_sub = a_max;
    if (replan == 1)
    {
        y = Vstart;
        a = 0;
        v_sub = 0;
        Tstart = pMs;
        tempVstart = Vstart;
        tt1 = j_max / k;
        tt2 = sqrt(2 * a_max / k);
        if (tt1 > tt2)
            tt = tt2;
        else
            tt = tt1;
        stopS = k * (tt * tt * tt * tt) / 24;
        Vend = k * (tt * tt * tt) / 6;
        lenS1 = (v_max + Vstart) * (v_max - Vstart) / 2 / a_max;
        lenS2 = (Vend + v_max) * (v_max - Vend) / 2 / a_max;
        if (stopS >= s)
        {
            stopS = s;
            tt = sqrt(24.0 * stopS / k);
            tt = sqrt(tt);
            Vend = k * (tt * tt * tt) / 6;
            t1 = 0;
            t2 = 0;
            t3 = 0;
            //            taskENTER_CRITICAL();
            //            {
            //                printf( "[\t%d] Speed:%f\r\n", pMs, Vend );
            //            }
            //            taskEXIT_CRITICAL();
        }
        if (s >= lenS1 + lenS2 + stopS)
        {
            t1 = 1000 * (v_max - Vstart) / a_max;
            t2 = 1000 * (s - lenS1 - lenS2 - stopS) / v_max;
            t3 = 1000 * (v_max - Vend) / a_max;
        }
        else if (s > stopS)
        {
            v_sub = sqrt(0.5 * (Vstart * Vstart + Vend * Vend + 2 * a_max * (s - stopS)));
            lenS1 = (v_sub + Vstart) * (v_sub - Vstart) / 2 / a_max;
            lenS2 = (Vend + v_sub) * (v_sub - Vend) / 2 / a_max;
            if (lenS1 <= 0)
            {
                t1 = 0;
                t2 = 0;
                t3 = 0;
            }
            else if (v_sub < Vend)
            {
                a_sub = 0.5 * sqrt(Vend * Vend - Vstart * Vstart) / (s - stopS);
                t1 = (Vend - v_sub) / a_sub;
                t2 = 0;
                t3 = 0;
            }
            else
            {
                t1 = 1000 * (v_sub - Vstart) / a_max;
                t2 = 0;
                t3 = 1000 * (v_sub - Vend) / a_max;
            }
        }
        else
        {
            t1 = 0;
            t2 = 0;
            t3 = 0;
        }
        //        taskENTER_CRITICAL();
        //        {
        //         printf("\r\n replan s=%f,t1=%d,t2=%d,t3=%d",stopS,t1,t2,t3);
        //         printf("replan t1=%d,t2=%d,t3=%d,stops=%f,vsub=%f,Vend=%f,v=%f,Vstart=%f",replan,t1,t2,t3,stopS,v_sub,Vend,y,Vstart);
        //        }
        //        taskEXIT_CRITICAL();
    }
    t = pMs - Tstart;
    if (t <= t1)
    {
        y = tempVstart + t * a_max * 0.001;
        // printf("\nseg1:replan=%uud,t=%u,t1=%d,t2=%d,t3=%d,stops=%f,vsub=%f,Vend=%f,v=%f,Vstart=%f",replan,t,t1,t2,t3,stopS,v_sub,Vend,y,Vstart);
    }
    else if (t <= (t1 + t2))
    {
        y = v_max;
        //printf("\nseg2:replan=%uud,t=%u,t1=%d,t2=%d,t3=%d,stops=%f,vsub=%f,Vend=%f,v=%f,Vstart=%f",replan,t,t1,t2,t3,stopS,v_sub,Vend,y,Vstart);
    }
    else if (t <= t1 + t2 + t3)
    {
        y = Vend + (t1 + t2 + t3 - t) * a_max * 0.001;
        //printf("\nseg3:replan=%uud,t=%u,t1=%d,t2=%d,t3=%d,stops=%f,vsub=%f,Vend=%f,v=%f,Vstart=%f",replan,t,t1,t2,t3,stopS,v_sub,Vend,y,Vstart);
    }
    else
    {
        float data = 24 * s / k;
        float x = sqrt(data);
        x = sqrt(x);
        y = k * x * x * x / 6;
        if (y > v_max)
            y = v_max;
        //printf("\nseg4:replan=%d,t=%d,t1=%d,t2=%d,t3=%d,stops=%f,vsub=%f,Vend=%f,v=%f,Vstart=%f",replan,t,t1,t2,t3,stopS,v_sub,Vend,y,Vstart);
    }
    if (y > v_max)
        y = v_max;
    return y;
}
float AGV_Parallel_Motion::Move(float iDistance)
{
#if 0
    static float speedNow,speedNowBak;
    static float distanceBak = iDistance;
    if( abs( iDistance ) > this->Stop_Accuracy * 0.5 )
    {
        speedNow = ts_curve( (unsigned char ) (this->isNewPosition == true ? 1 : 0 ),this->clock, abs(speedNowBak), this->ts_curve_k/* 1000 */, ts_curve_j_max /* 2000 */, this->ts_curve_a_max /* 1500 */, this->ts_curve_v_max /* 2500 */, abs(iDistance));
        if( speedNow < sSpeed_min )
            speedNow = sSpeed_min;
        if( distanceBak < iDistance )
        {
            if( DebugCtrl.enableMotionDebug )
            {
                taskENTER_CRITICAL();
                {
                    printf( "[\t%d] New Position Move To Get: iDistanceNew->%fmm iDistanceOld->%fmm replan->%s speedNw->%fmm/s speedOld->%fmm/s Position Now: x->%fmm y->%fmm Position Next x->%fmm y->%fmm\r\n", clock, iDistance, distanceBak, isNewPosition == true ? "true" : "false", speedNow, speedNowBak, AGV_Pose.x, AGV_Pose.y, AGV_PoseNext.x, AGV_PoseNext.y );
                }
                taskEXIT_CRITICAL();
            }
        }
        if( this ->isNewPosition )
        {
            if( DebugCtrl.enableMotionDebug )
            {
                taskENTER_CRITICAL();
                {
                    printf( "[\t%d] New Position Move To \"replan\":%s\t\"Distance\":%fmm speedNew->%fmm/s speedOld->%fmm/s Position Now: x->%fmm y->%fmm Position Next x->%fmm y->%fmm\r\n", clock, isNewPosition == true ? "true" : "false", iDistance, speedNow, speedNowBak, AGV_Pose.x, AGV_Pose.y, AGV_PoseNext.x, AGV_PoseNext.y );
                }
                taskEXIT_CRITICAL();
            }
            this->isNewPosition = false;
        }
        distanceBak = iDistance;
        speedNowBak = speedNow;
    }
    else
    {
        speedNow = 0;
    }
    if (iDistance < 0)
        return -speedNow;
    return speedNow;
#else
    float SetSpeed = 0.000;
    static float speedNow = 0;
    float stopDistance;

    if (speedNow > sSpeed_mid)
    {
        stopDistance = (speedNow * speedNow) / (2.0 * sAcceleration) + (sSpeed_mid * sSpeed_mid) / (2 * sAccelerationLow) + (((speedNow - sSpeed_mid) / sAcceleration) * sSpeed_mid) + sDeceleration_distance;
        if (abs(iDistance) > stopDistance)
        {
            SetSpeed = sSpeed_max;
        }
        else
        {
            SetSpeed = sSpeed_min;
        }
    }
    else
    {
        stopDistance = speedNow * speedNow / (2.0 * sAccelerationLow) + sDeceleration_distance;
        if (abs(iDistance) > stopDistance)
        {
            SetSpeed = sSpeed_max;
        }
        else
        {
            SetSpeed = sSpeed_min;
        }
    }
    if (abs(iDistance) <= Stop_Accuracy /* * 0.5 */)
    {
        SetSpeed = sSpeed_min;
    }
    if (iDistance < 0)
        SetSpeed = -SetSpeed;

    if (sSpeed_max == 0)
        SetSpeed = 0;

    float securityDelta_straight;

    if (speedNow > SetSpeed)
    {
        if (speedNow > sSpeed_mid)
        {
            securityDelta_straight = sAcceleration * sample_Time / 1000.0;
        }
        else
        {
            securityDelta_straight = sAccelerationLow * sample_Time / 1000.0;
        }
    }
    else
    {
        /*
        if( abs(iDistance) > (sSpeed_mid2 * sSpeed_mid2 / ( 2 * sAccelerationLow2 ) ))
        {
            securityDelta_straight = sAcceleration * sample_Time / 1000.0;
        }
        else
        {
            if( speedNow > sSpeed_mid2 )
            {
                securityDelta_straight = sAcceleration * sample_Time / 1000.0;
            }
            else
            {
                securityDelta_straight = sAccelerationLow2 * sample_Time / 1000.0;
            }
        }
        */
        securityDelta_straight = sAcceleration * sample_Time / 1000.0;
    }
    if (SetSpeed == 0)
    {

        if (abs(speedNow) <= sSpeed_min)
        {
            return 0;
        }
        else
        {
            if (speedNow > SetSpeed)
                speedNow -= securityDelta_straight;
            else if (speedNow < SetSpeed)
                speedNow += securityDelta_straight;
        }
    }
    else
    {
        if (speedNow > SetSpeed)
            speedNow -= securityDelta_straight;
        else if (speedNow < SetSpeed)
            speedNow += securityDelta_straight;
    }

    return speedNow;

#endif
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