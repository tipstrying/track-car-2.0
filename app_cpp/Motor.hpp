#ifndef __MOTOR_HPP
#define __MOTOR_HPP
#include "cmsis_os.h"
class MotorDriv {

public:
    MotorDriv();
private:
    int PosX;
    int PosY;
};
#endif