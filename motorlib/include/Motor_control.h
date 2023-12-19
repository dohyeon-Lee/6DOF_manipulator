#ifndef __GROUP_MOTOR_CONTROL_H__
#define __GROUP_MOTOR_CONTROL_H__

#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <unistd.h>
#include "dynamixel_sdk.h"
#include "Motor.h"  
#include "group_bulk_write.h"
#include <iostream>
using std::vector;

#define STDIN_FILENO 0
// Control table address
#define ADDR_MX_TORQUE_ENABLE           64 // MX540-W270 spec (AX-12A : 24)  Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           116 // MX540-W270 spec (AX-12A : 30)
#define ADDR_MX_PRESENT_POSITION        132 // MX540-W270 spec (AX-12A : 36)
#define ADDR_MX_GOAL_VELOCITY           104
#define ADDR_MX_PRESENT_VELOCITY        128      
#define ADDR_MX_GOAL_CURRENT            102
#define ADDR_MX_PRESENT_CURRENT         126

// Data Byte Length
#define LEN_MX_GOAL_POSITION            4
#define LEN_MX_PRESENT_POSITION         4
#define LEN_MX_GOAL_VELOCITY            4
#define LEN_MX_PRESENT_VELOCITY         4
#define LEN_MX_GOAL_CURRENT             2
#define LEN_MX_PRESENT_CURRENT          2

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define BAUDRATE                        1000000//115200
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0//-1048575                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095//1048575               // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     24                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define OPERATING_MODE                  11
#define VELOCITY_MODE                   1
#define POSITION_MODE                   3 // default
#define CURRENT_MODE                    0
#define EXTENDED_POSITION_MODE          4
class Motor_control
{
    public:
        Motor_control(vector<Motor> motor_);
        double fmap(double x, double in_min, double in_max, double out_min, double out_max);
        int angle(double angle_); // radian to control range
        double inv_angle(double data);
        int velocity(double rad_per_sec); // rad/s to control range
        int torque(double Nm);
        void torque_off(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler);
        void torque_on(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler);
        void setMode(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t mode);
        // void getPosition(dynamixel::GroupFastSyncRead groupFastSyncRead);
        void getPosition(dynamixel::GroupSyncRead groupSyncRead);
        void setPosition(dynamixel::GroupSyncWrite groupSyncWrite);
        void setVelocity(dynamixel::GroupSyncWrite groupSyncWrite);
        void setTorque(dynamixel::GroupSyncWrite groupSyncWrite);

        vector<Motor> motor;
        uint16_t data_length = LEN_MX_GOAL_CURRENT + LEN_MX_GOAL_VELOCITY + 4 + 4 + LEN_MX_GOAL_POSITION;
};

#endif