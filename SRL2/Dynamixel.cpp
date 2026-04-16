#include "Dynamixel.h"
#include <../../SRL2/dynamixel_sdk/dynamixel_sdk.h>
#include "math.h"
#include "rt485.h"
#include "iostream"
Dynamixel::Dynamixel()
{

}
bool Dynamixel::init(int dynamixelid,rt485* m_485Comm)
{
    DXL_ID = dynamixelid;
    m_dComm = m_485Comm;
    return true;
}

void Dynamixel::enableDynamixel(uint8_t flag)
{
    rt_printf("DXL_ID:%d\n",DXL_ID);
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
//    // Enable Dynamixel Torque
//    dxl_comm_result = packetHandler->write1ByteTxOnly(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, flag);
    dxl_comm_result = m_dComm->packetHandler->write1ByteTxRx(m_dComm->portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, flag, &dxl_error);

}

void Dynamixel::setPos(float pos)
{
   int dxl_comm_result = COMM_TX_FAIL;             // Communication result
   int p =int((pos + initpos)*4096/360);
//   std::cout <<"pos:"<<p<<std::endl;
   dxl_comm_result = m_dComm->packetHandler->write4ByteTxOnly(m_dComm->portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, p);
//   rt_printf("dxl_comm_result:%d\n",dxl_comm_result);
}

void Dynamixel::setpos(int pos)  //  0-4096
{
   int dxl_comm_result = COMM_TX_FAIL;             // Communication result
   dxl_comm_result = m_dComm->packetHandler->write4ByteTxOnly(m_dComm->portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, pos);
}

void Dynamixel::setvel(int vel)  // -130-130
{
   int dxl_comm_result = COMM_TX_FAIL;             // Communication result
   dxl_comm_result = m_dComm->packetHandler->write4ByteTxOnly(m_dComm->portHandler, DXL_ID, ADDR_PRO_GOAL_VELOCITY, vel);
}

void Dynamixel::settor(int tor)  //-648 -648
{
   int dxl_comm_result = COMM_TX_FAIL;             // Communication result
   dxl_comm_result = m_dComm->packetHandler->write4ByteTxOnly(m_dComm->portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, tor);
   rt_printf("tor\n");
}

void Dynamixel::getPos()
{
   float p;
   uint8_t dxl_error = 0;
   uint32_t dxl_present_position;
   // Read present position
   int a=m_dComm->packetHandler->read4ByteTxRx(m_dComm->portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, &dxl_present_position,&dxl_error);

   p = float(dxl_present_position*360.0/4096) - initpos;

    rt_printf("a:%d\n",a);
    rt_printf("p:%f\n",p);
    posr = p ;
}


void Dynamixel::getpos(uint32_t dxl_present_position)
{
   uint8_t dxl_error = 0;
  int a=m_dComm->packetHandler->read4ByteRx(m_dComm->portHandler, DXL_ID, &dxl_present_position,&dxl_error);
  rt_printf("a:%d\n",a);
  rt_printf("dxl_present_position:%d\n",dxl_present_position);
}

void Dynamixel::getZeroPos()
{
   float p;
   p=posr ;
   rt_printf("Dynamixel ID[%d] init pos:%0.3f\n",DXL_ID,p);
}

/**************************new wrist****************************/
void Dynamixel::openGripper(int current)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//    int openCurrent = -6;
    dxl_comm_result = m_dComm->packetHandler->write2ByteTxOnly(m_dComm->portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, current);

}

void Dynamixel::closeGripper(int currunt)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//    int closeCurrent = 180;
    dxl_comm_result = m_dComm->packetHandler->write2ByteTxOnly(m_dComm->portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, currunt);

}

void Dynamixel::stopGripper()
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    int zeroCurrent = 0;
    dxl_comm_result = m_dComm->packetHandler->write2ByteTxOnly(m_dComm->portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, zeroCurrent);

}


