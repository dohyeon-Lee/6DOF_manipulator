#include "Motor.h"

Motor::Motor(int id_, double inital_position_, double Min_angle_, double Max_angle_)
{
    id = id_;
    Min_angle = Min_angle_;
    Max_angle = Max_angle_;
    inital_pos = inital_position_;
    pos = inital_position_;
};