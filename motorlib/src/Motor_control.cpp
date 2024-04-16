#include "Motor_control.h"

Motor_control::Motor_control(vector<Motor> motor_)
{
    motor = motor_; //shallow copy
};

double Motor_control::fmap(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min;
};

int Motor_control::angle(double angle_)
{
    return int(fmap(angle_, -M_PI, M_PI, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE));
};
double Motor_control::inv_angle(double data)
{
    return fmap(data, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE, -M_PI, M_PI);
}
int Motor_control::velocity(double rad_per_sec)
{
    double rpm = rad_per_sec * 9.5492968;
    int output = 0;
    if (int(rpm/0.229) > 127)
    {
        std::cout<<"To Fast"<<std::endl;
        output = 127;
    }
    else if (int(rpm/0.229) < -127)
    {
        std::cout<<"To Fast"<<std::endl;
        output = -127;
    }
    else
        output = int(rpm/0.229);
    return output;
};

int Motor_control::torque(double Nm_)
{
    double Nm = Nm_;
    if(Nm > 8.6)
        Nm = 8.6;
    else if(Nm < -8.6)
        Nm = -8.6;
    double current = fmap(Nm, -8.6, 8.6, -4.06, 4.06); // N-T curve (torque(Nm) -> current(A))
    
    // current limit : 2047 (0~2047), minimum control current : 2.69mA
    double minimum_current = 2.69 * 0.001;
    int output = int(current / minimum_current);
    
    if(output > 2047)
        output = 2047;
    else if(output < -2047)
        output = -2047;
    
    return output;
};

void Motor_control::torque_off(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    uint8_t dxl_error = 0;
    for(int i = 0; i < motor.size(); i++)
        packetHandler->write1ByteTxRx(portHandler, motor[i].getID(), ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
};


void Motor_control::torque_on(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    uint8_t dxl_error = 0;                         
    for(int i = 0; i < motor.size(); i++)
        packetHandler->write1ByteTxRx(portHandler, motor[i].getID(), ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

};

void Motor_control::setMode(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t mode)
{
    uint8_t dxl_error = 0; 
    torque_off(portHandler, packetHandler);
    for(int i = 0; i < motor.size(); i++)
    {
        packetHandler->write1ByteTxRx(portHandler, motor[i].getID(), OPERATING_MODE, mode, &dxl_error);
    }
    torque_on(portHandler, packetHandler);
};

// void Motor_control::getPosition(dynamixel::GroupFastSyncRead groupFastSyncRead)
// {
//     int dxl_comm_result = COMM_TX_FAIL;               // Communication result
//     bool dxl_addparam_result = false;                 // addParam result
//     bool dxl_getdata_result = false;                  // GetParam result

//     for(int i = 0; i < motor.size(); i++)
//     {
//         dxl_addparam_result = groupFastSyncRead.addParam(motor[i].getID());
//         if (dxl_addparam_result != true)
//         {
//             fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motor[i].getID());
//         }
//     }
//     dxl_comm_result = groupFastSyncRead.txRxPacket();
//     if (dxl_comm_result != COMM_SUCCESS)
//         printf("fail");

//     for(int i = 0; i < motor.size(); i++)
//         motor[i].measured_pos = inv_angle(groupFastSyncRead.getData(motor[i].getID(), ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION));
// }

void Motor_control::getPosition(dynamixel::GroupSyncRead groupSyncRead)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    bool dxl_getdata_result = false;                  // GetParam result

    for(int i = 0; i < motor.size(); i++)
    {
        dxl_addparam_result = groupSyncRead.addParam(motor[i].getID());
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motor[i].getID());
        }
    }
    dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        printf("fail");

    for(int i = 0; i < motor.size(); i++)
        motor[i].measured_pos = inv_angle(groupSyncRead.getData(motor[i].getID(), ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION));
}


void Motor_control::setPosition(dynamixel::GroupSyncWrite groupSyncWrite) //points->point
{  
                        
    for(int i = 0; i < motor.size(); i++)
    {
        uint32_t input_position = angle(motor[i].pos);
        int input_velocity = 0;
        int input_Torque = 0;

        uint8_t data[data_length];
        data[0] = input_Torque & 0xFF;
        data[1] = (input_Torque >> 8) & 0xFF;
        data[2] = input_velocity & 0xFF;
        data[3] = (input_velocity >> 8) & 0xFF;
        data[4] = (input_velocity >> 16) & 0xFF;
        data[5] = (input_velocity >> 24) & 0xFF;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 0;
        data[10] = 0;
        data[11] = 0;
        data[12] = 0;
        data[13] = 0;
        data[14] = input_position & 0xFF;
        data[15] = (input_position >> 8) & 0xFF;
        data[16] = (input_position >> 16) & 0xFF;
        data[17] = (input_position >> 24) & 0xFF;
        groupSyncWrite.addParam(motor[i].getID(), data);
    }
    groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
};
void Motor_control::setTorque(dynamixel::GroupSyncWrite groupSyncWrite) //points->point
{                      
    for(int i = 0; i < motor.size(); i++)
    {
        uint32_t input_position = 0;
        int input_velocity = 0;
        int input_Torque = torque(motor[i].torque);

        uint8_t data[data_length];
        data[0] = input_Torque & 0xFF;
        data[1] = (input_Torque >> 8) & 0xFF;
        data[2] = input_velocity & 0xFF;
        data[3] = (input_velocity >> 8) & 0xFF;
        data[4] = (input_velocity >> 16) & 0xFF;
        data[5] = (input_velocity >> 24) & 0xFF;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 0;
        data[10] = 0;
        data[11] = 0;
        data[12] = 0;
        data[13] = 0;
        data[14] = input_position & 0xFF;
        data[15] = (input_position >> 8) & 0xFF;
        data[16] = (input_position >> 16) & 0xFF;
        data[17] = (input_position >> 24) & 0xFF;
        groupSyncWrite.addParam(motor[i].getID(), data);
    }
    groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
};
void Motor_control::setVelocity(dynamixel::GroupSyncWrite groupSyncWrite) //points->point
{                      
    for(int i = 0; i < motor.size(); i++)
    {
        uint32_t input_position = 0;
        int input_velocity = velocity(motor[i].vel);
        int input_Torque = 0;

        uint8_t data[data_length];
        data[0] = input_Torque & 0xFF;
        data[1] = (input_Torque >> 8) & 0xFF;
        data[2] = input_velocity & 0xFF;
        data[3] = (input_velocity >> 8) & 0xFF;
        data[4] = (input_velocity >> 16) & 0xFF;
        data[5] = (input_velocity >> 24) & 0xFF;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 0;
        data[10] = 0;
        data[11] = 0;
        data[12] = 0;
        data[13] = 0;
        data[14] = input_position & 0xFF;
        data[15] = (input_position >> 8) & 0xFF;
        data[16] = (input_position >> 16) & 0xFF;
        data[17] = (input_position >> 24) & 0xFF;
        groupSyncWrite.addParam(motor[i].getID(), data);
    }
    groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
};
