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

using std::vector;
double k4 = 4;
double k3 = 3;
double k2 = 1.0;
double hpbMKS[8][6] = {{0,0*k2,0,-10*k4,-50,0},{30,15*k2,10*k3,10*k4,50,0},{60,30*k2,20*k3,-10*k4,-50,0},{30,15*k2,10*k3,10*k4,50,0},{0,0*k2,0*k3,-10*k4,-50,0},{-30,15*k2,10*k3,10*k4,50,0},{-60,30*k2,20*k3,-10*k4,-50,0},{-30,15*k2,10*k3,10*k4,50,0}};

int main()
{
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  uint16_t data_length = LEN_MX_GOAL_CURRENT + LEN_MX_GOAL_VELOCITY + 4 + 4 + LEN_MX_GOAL_POSITION;
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_CURRENT, data_length); //vel,pos,torque 
  //dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_VELOCITY, LEN_MX_GOAL_VELOCITY);
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);
  vector<Motor> motor;

  //여기까지 건들필요없음

  motor.push_back(Motor(1));
  motor.push_back(Motor(2));
  motor.push_back(Motor(3));
  motor.push_back(Motor(4));
  motor.push_back(Motor(5));
  motor.push_back(Motor(6));

  Motor_control actuate_motor(motor);
 // position control 
  actuate_motor.setMode(portHandler, packetHandler, POSITION_MODE);

  double theta0;
  double theta1;
  double theta2;
  double theta3;
  double theta4;
  double theta5;

  double angle0;
  double angle1;
  double angle2;
  double angle3;
  double angle4;
  double angle5;

  double time = 1;
  double step = 100;

  actuate_motor.motor[0].pos = 0;
  actuate_motor.motor[1].pos = 0;
  actuate_motor.motor[2].pos = 0;
  actuate_motor.motor[3].pos = 0;
  actuate_motor.motor[4].pos = 0;
  actuate_motor.motor[5].pos = 0;
  actuate_motor.setPosition(groupSyncWrite);
  
  if (Nearzero(0.1) == false)
    printf("false");
  else
    printf("true");
  // //MKS
  // int count = 0;
  // while(1)
  // {
  //   count = count % 8;

  //   theta0 = hpbMKS[count][0];
  //   theta1 = hpbMKS[count][1];
  //   theta2 = hpbMKS[count][2];
  //   theta3 = hpbMKS[count][3];
  //   theta4 = hpbMKS[count][4];
  //   theta5 = hpbMKS[count][5];
    
  //   angle0 = -(actuate_motor.motor[0].pos - theta0 * (M_PI/180));
  //   angle1 = -(actuate_motor.motor[1].pos - theta1 * (M_PI/180));
  //   angle2 = -(actuate_motor.motor[2].pos - theta2 * (M_PI/180));
  //   angle3 = -(actuate_motor.motor[3].pos - theta3 * (M_PI/180));
  //   angle4 = -(actuate_motor.motor[4].pos - theta4 * (M_PI/180));
  //   angle5 = -(actuate_motor.motor[5].pos - theta5 * (M_PI/180));

  //   for(int i = 0; i < (int)step; i++)
  //   {
  //     actuate_motor.motor[0].pos += angle0/step;
  //     actuate_motor.motor[1].pos += angle1/step;
  //     actuate_motor.motor[2].pos += angle2/step;
  //     actuate_motor.motor[3].pos += angle3/step;
  //     actuate_motor.motor[4].pos += angle4/step;
  //     actuate_motor.motor[5].pos += angle5/step;
  //     actuate_motor.setPosition(groupSyncWrite);
  //     printf("%5.3lf %5.3lf %5.3lf %5.3lf %5.3lf %5.3lf \n", actuate_motor.motor[0].pos, actuate_motor.motor[1].pos, actuate_motor.motor[2].pos, actuate_motor.motor[3].pos, actuate_motor.motor[4].pos, actuate_motor.motor[5].pos);
  //     usleep(int(time*1000000.0/step));
  //   }
  //   count += 1;
  // }
  
  actuate_motor.torque_off(portHandler, packetHandler); //motor releiving code
}