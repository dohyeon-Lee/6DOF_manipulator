#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include "dynamixel_sdk.h"                                 
#include <unistd.h> // usleep
#include <string>

#include "Motor.h"
#include "Motor_control.h"

#include <iostream>
#include <iomanip>
#include <cmath>

#include "Robotics.h"
#include <chrono>
#include <fstream>
#include <sstream>

using std::vector;
using namespace std;
// inital setting 
// 1. sudo chmod a+rw /dev/ttyUSB0
// 2. sudo gedit /sys/bus/usb-serial/devices/ttyUSB0/latency_timer and change 16 --> 1

int main()
{
  // setting start
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  uint16_t data_length = LEN_MX_GOAL_CURRENT + LEN_MX_GOAL_VELOCITY + 4 + 4 + LEN_MX_GOAL_POSITION;
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_CURRENT, data_length); //vel,pos,torque 
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);
  vector<Motor> motor;
  // setting finish

  // add motors
  motor.push_back(Motor(1));
  motor.push_back(Motor(2));
  motor.push_back(Motor(3));
  motor.push_back(Motor(4));
  motor.push_back(Motor(5));
  motor.push_back(Motor(6));
  Motor_control actuate_motor(motor);

 // position control (radian)
  actuate_motor.setMode(portHandler, packetHandler, POSITION_MODE);
  actuate_motor.motor[0].pos = 0;
  actuate_motor.motor[1].pos = 0;
  actuate_motor.motor[2].pos = 0;
  actuate_motor.motor[3].pos = 0;
  actuate_motor.motor[4].pos = 0;
  actuate_motor.motor[5].pos = 0;
  actuate_motor.setPosition(groupSyncWrite);

  usleep(5000000); // delay 5 seconds

  // velocity control (rad/s)
  actuate_motor.setMode(portHandler, packetHandler, VELOCITY_MODE);
  actuate_motor.motor[0].vel = 0;
  actuate_motor.motor[1].vel = 0;
  actuate_motor.motor[2].vel = 0;
  actuate_motor.motor[3].vel = 0;
  actuate_motor.motor[4].vel = 0;
  actuate_motor.motor[5].vel = 0;
  actuate_motor.setVelocity(groupSyncWrite);

  usleep(5000000); // delay 5 seconds

  // torque control (Nm)
  actuate_motor.setMode(portHandler, packetHandler, CURRENT_MODE);
  actuate_motor.motor[0].torque = 0;
  actuate_motor.motor[1].torque = 0;
  actuate_motor.motor[2].torque = 0;
  actuate_motor.motor[3].torque = 0;
  actuate_motor.motor[4].torque = 0;
  actuate_motor.motor[5].torque = 0;
  actuate_motor.setVelocity(groupSyncWrite);

  usleep(5000000); // delay 5 seconds

  // read encoder (radian)
  actuate_motor.getPosition(groupSyncRead);
  VectorXf now_thetalist(6);
  now_thetalist(0) = actuate_motor.motor[0].measured_pos;
  now_thetalist(1) = actuate_motor.motor[1].measured_pos;
  now_thetalist(2) = actuate_motor.motor[2].measured_pos;
  now_thetalist(3) = actuate_motor.motor[3].measured_pos;
  now_thetalist(4) = actuate_motor.motor[4].measured_pos;
  now_thetalist(5) = actuate_motor.motor[5].measured_pos;
  std::cout << now_thetalist << std::endl;

}