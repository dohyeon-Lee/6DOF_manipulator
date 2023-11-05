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
using std::vector;

// inital setting 
// /sys/bus/usb-serial/devices/ttyUSB0/latency_timer 16 --> 1
// sudo chmod a+rw /dev/ttyUSB0

int main()
{
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  uint16_t data_length = LEN_MX_GOAL_CURRENT + LEN_MX_GOAL_VELOCITY + 4 + 4 + LEN_MX_GOAL_POSITION;
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_CURRENT, data_length); //vel,pos,torque 
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
  // dynamixel::GroupFastSyncRead groupFastSyncRead(portHandler, packetHandler, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
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

  usleep(5000000);

  MatrixXf M = getM();
  MatrixXf Blist = getBlist();

  VectorXf thetalist_start(6);
  VectorXf thetalist_end(6);
  thetalist_start << 0,0,0,0,0,0;
  thetalist_end << 0,-45,-20,0,45,0;
  thetalist_end *= (M_PI/180.);
  MatrixXf X_start = FKinBody(M, Blist, thetalist_start);
  // MatrixXf X_end = FKinBody(M, Blist, thetalist_end);
  MatrixXf X_end(4,4);
  X_end <<  1, 0, 0, 0.0,
            0, 1, 0, 0.2,
            0, 0, 1, 0.4,
            0, 0, 0, 1;
  std::cout << X_start << std::endl;
  MatrixXf R_start = X_start.block<3,3>(0,0);
  MatrixXf R_end = X_end.block<3,3>(0,0);
  MatrixXf P_start = X_start.block<3,1>(0,3);
  MatrixXf P_end = X_end.block<3,1>(0,3);

  double t = 0;
  double Duration = 0.06;
  VectorXf thetalist = thetalist_start;
  VectorXf thetalist_dot(6);
  thetalist_dot << 0,0,0,0,0,0;
  MatrixXf before_R_desired = R_start * MatrixExp3(MatrixLog3((R_start.transpose())*R_end)*0);
  MatrixXf before_P_desired = P_start + (P_end - P_start)*0;

  // control parameter
  double Kp = 0.8;
  double Ki = 0.01;

  // actuate
  int n = 100;
  VectorXf Iterm(6);
  Iterm << 0,0,0,0,0,0;
  actuate_motor.setMode(portHandler, packetHandler, VELOCITY_MODE);

  for(int i = 0; i < n; i++)
  {
    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    thetalist += thetalist_dot*Duration;
    t += Duration;
    VectorXf now_thetalist(6);
    //now_thetalist = thetalist;
    now_thetalist << 0,0,0,0,0,0;

    actuate_motor.getPosition(groupSyncRead);
    now_thetalist(0) = actuate_motor.motor[0].measured_pos;
    now_thetalist(1) = actuate_motor.motor[1].measured_pos;
    now_thetalist(2) = actuate_motor.motor[2].measured_pos;
    now_thetalist(3) = actuate_motor.motor[3].measured_pos;
    now_thetalist(4) = actuate_motor.motor[4].measured_pos;
    now_thetalist(5) = actuate_motor.motor[5].measured_pos;

    // actuate_motor.getPosition(groupSyncRead);
    // std::cout << actuate_motor.motor[0].measured_pos << ", "
    // << actuate_motor.motor[1].measured_pos << ", "
    // << actuate_motor.motor[2].measured_pos << ", "
    // << actuate_motor.motor[3].measured_pos << ", "
    // << actuate_motor.motor[4].measured_pos << ", "
    // << actuate_motor.motor[5].measured_pos << std::endl;

    MatrixXf X_sb = FKinBody(M,Blist,now_thetalist);
    MatrixXf R_sb = X_sb.block<3,3>(0,0);
    VectorXf P_sb = X_sb.block<3,1>(0,3);
    // trajectory s
    double s = (3/pow((Duration*n), 2))*pow(t,2) - (2/pow((Duration*n),3))*pow(t,3);
    MatrixXf X_desired = X_start * MatrixExp6(MatrixLog6(X_start.inverse()*X_end)*s);
    MatrixXf R_desired = X_desired.block<3,3>(0,0);
    VectorXf P_desired = P_start + (P_end - P_start)*s;
    
    MatrixXf R_desired_dot = (R_desired - before_R_desired)/Duration;
    MatrixXf Jb = JacobianBody(Blist, thetalist);
    
    VectorXf w_d = so3ToVec((R_desired.transpose())*R_desired_dot);
    VectorXf w_b = ((R_sb.transpose())*R_desired)*w_d;
    VectorXf P_desired_dot = (P_desired - before_P_desired)/Duration;
    VectorXf Xe(6);
    Xe << so3ToVec(MatrixLog3(((R_sb.transpose())*R_desired))), P_desired-P_sb;
    Iterm += Xe*Duration;
    VectorXf twist_decoupled(6);
    twist_decoupled << w_b, P_desired_dot;
    twist_decoupled += Kp*Xe + Ki*Iterm; 
    MatrixXf J_decoupled(6,6);
    J_decoupled << Jb.block<3,6>(0,0), R_sb*Jb.block<3,6>(3,0); 
    MatrixXf J_decoupled_pinv = J_decoupled.completeOrthogonalDecomposition().pseudoInverse();
    thetalist_dot = J_decoupled_pinv*twist_decoupled;

    before_P_desired = P_desired;
    before_R_desired = R_desired;

    // actuate position
    // actuate_motor.motor[0].pos = thetalist(0);
    // actuate_motor.motor[1].pos = thetalist(1);
    // actuate_motor.motor[2].pos = thetalist(2);
    // actuate_motor.motor[3].pos = thetalist(3);
    // actuate_motor.motor[4].pos = thetalist(4);
    // actuate_motor.motor[5].pos = thetalist(5);
    // actuate_motor.setPosition(groupSyncWrite);  
    actuate_motor.motor[0].vel = thetalist_dot(0);
    actuate_motor.motor[1].vel = thetalist_dot(1); // rad/s
    actuate_motor.motor[2].vel = thetalist_dot(2);
    actuate_motor.motor[3].vel = thetalist_dot(3);
    actuate_motor.motor[4].vel = thetalist_dot(4);
    actuate_motor.motor[5].vel = thetalist_dot(5);
    actuate_motor.setVelocity(groupSyncWrite);
    std::chrono::duration<double>sec = std::chrono::system_clock::now() - start;
    if (sec.count() < Duration)
      usleep(int((Duration - sec.count())*1000000));

    else
      std::cout << "to slow" << std::endl; 
    // std::cout << thetalist_dot << std::endl;
  }
  
  actuate_motor.motor[0].vel = 0;
  actuate_motor.motor[1].vel = 0; // rad/s
  actuate_motor.motor[2].vel = 0;
  actuate_motor.motor[3].vel = 0;
  actuate_motor.motor[4].vel = 0;
  actuate_motor.motor[5].vel = 0;

  actuate_motor.setVelocity(groupSyncWrite);
  std::cout << thetalist << std::endl;
  std::cout << before_R_desired ;

  // actuate_motor.setMode(portHandler, packetHandler, VELOCITY_MODE);
  // for(int i = 0; i < 3; i++)
  // {
  //   actuate_motor.motor[0].vel = 3.14; //lsrad/s
  //   actuate_motor.motor[1].vel = 3.14; // rad/s
  //   actuate_motor.motor[2].vel = 3.14;
  //   actuate_motor.motor[3].vel = 3.14;
  //   actuate_motor.motor[4].vel = 3.14;
  //   actuate_motor.motor[5].vel = 3.14;

  //   actuate_motor.setVelocity(groupSyncWrite);
  //   usleep(1000000);
  //   actuate_motor.motor[0].vel = -3.14;
  //   actuate_motor.motor[1].vel = -3.14; // rad/s
  //   actuate_motor.motor[2].vel = -3.14;
  //   actuate_motor.motor[3].vel = -3.14;
  //   actuate_motor.motor[4].vel = -3.14;
  //   actuate_motor.motor[5].vel = -3.14;
  //   actuate_motor.setVelocity(groupSyncWrite);
  //   usleep(1000000);
  // }

  // actuate_motor.torque_off(portHandler, packetHandler); //motor releiving code
}