#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H
#include <../../SRL2/dynamixel_sdk/dynamixel_sdk.h>
class rt485;
// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_GOAL_VELOCITY         104
#define ADDR_PRO_GOAL_CURRENT        102

#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_PRESENT_VELOCITY        128
#define ADDR_PRO_PRESENT_CURRENT        126

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -150000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#define ESC_ASCII_VALUE                 0x1b



class Dynamixel
{
public:
    Dynamixel();
    bool init(int dynamixelid , rt485*m_485Comm);
    void enableDynamixel(uint8_t flag);
    void setPos(float pos);
    void setpos(int pos);
    void setvel(int vel);
    void settor(int tor);

    void getPos();
    void getpos(uint32_t dxl_present_position);
    void getZeroPos();

    void openGripper(int current);
    void closeGripper(int currunt);
    void stopGripper();
    int DXL_ID;
    rt485  *m_dComm = NULL;   
    float initpos = 0;  // left  limb
//    float initpos[4]= {178.59,179.21,92.988,150.645};  // left  limb
//        float initpos[4]= {178.86,199.78,156.88,0};  // right limb
    float posr;
    float velr;
    float torr;
};

#endif // DYNAMIXEL_H
