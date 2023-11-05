#ifndef __MOTOR_H__
#define __MOTOR_H__
#define _USE_MATH_DEFINES
#include <cmath>
class Motor
{
    public:
    Motor(int id = 1, double inital_position = 0, double Min_angle = 0, double Max_angle = 2*M_PI);
    int getID(){ return id; }
    double pos = 0; //radian
    double vel = 0; //rad/s
    double torque = 0; // NM
    double inital_pos;
    double measured_pos = 0;
    private:
    int id = 1;
    double Min_angle;
    double Max_angle;
    
    
    
};
#endif