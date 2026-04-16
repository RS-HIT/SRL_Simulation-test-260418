#ifndef RT485_H
#define RT485_H
#include<vector>
#include <../../SRL2/dynamixel_sdk/dynamixel_sdk.h>
#include "Dynamixel.h"
// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel
#define BAUDRATE                        4000000
//#define BAUDRATE                        115200
//#define DEVICENAME                     "/dev/ttyUSB0"      // Check which port is being used on your controller

class RT_TASK;                                                          // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
class Dynamixel;

class rt485
{
public:
    rt485();
    std::vector<Dynamixel*>    dynamixels;
    bool init(const char *DEVICENAME);
    void adddynamixel(int id);
    dynamixel::PortHandler  *portHandler = NULL;
    dynamixel::PacketHandler  *packetHandler = NULL;

    static  void rxTask(void * arg);
    void rx();
    RT_TASK*  rx_task_485;
    void close();
    float   m_TStep = 0.05;
    int   m_dynamixel_num = 4;
};

#endif // RT485_H
