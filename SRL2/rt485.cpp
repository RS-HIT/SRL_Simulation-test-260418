#include "rt485.h"
#include "math.h"
#include "iostream"
#include <unistd.h>


#include </usr/xenomai/include/trank/native/task.h>
#include </usr/xenomai/include/trank/native/timer.h>
#include <sys/socket.h>
#include </usr/xenomai/include/trank/rtdk.h>
#include </usr/xenomai/include/trank/native/sem.h>
#include </usr/xenomai/include/trank/native/mutex.h>

rt485::rt485()
{

}

void rt485::rxTask(void * arg)
{
    rt485* pthis = (rt485*)arg;
    unsigned long ov;

    float rtPeroid = pthis->m_TStep*1000000000;
    rt_task_set_periodic(NULL, TM_NOW, rtPeroid);

    int count = 0,count1=0;
    while(true)
    {
        pthis->rx();
        rt_task_wait_period(&ov);

    }
}


void rt485::rx()
{
//     rt_printf("this is 485 thread\n");
//     for(int i=0;i<m_dynamixel_num;i++)
//     {
         dynamixels[2]->getPos();

//           dynamixels[2]->setvel(120);
//          dynamixels[2]->setpos(0);
         rt_printf("Dynamixel ID[%d] init pos:%0.3f\n",2, dynamixels[2]->posr);

//     }
}

bool rt485::init(const char *DEVICENAME)
{

    int err;

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindow
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);          // Present position

    // Open port
    if (portHandler->openPort())
    {
      printf("Succeeded to open the port!\n");
    }
    else
    {
      printf("Failed to open the port!\n");
      printf("Press any key to terminate...\n");
//      getch();
      return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
      printf("Succeeded to change the baudrate!\n");
    }
    else
    {
      printf("Failed to change the baudrate!\n");
      printf("Press any key to terminate...\n");
//      getch();
      return 0;
    }


//    rx_task_485 = new RT_TASK();
//    err = rt_task_create(rx_task_485, "485rx", 0, 59, 0);
//    if (err) {
//        printf("receivetest: Failed to create rt task, code %d\n",
//               errno);
//        return err;
//    }
//    rt_task_start(rx_task_485,rt485::rxTask, this);
//    if (err) {
//        printf("receivetest: Failed to start rt task, code %d\n",
//               errno);
//        return errno;
//    }
    return err;
}

void rt485::close()
{
    rt_task_delete(rx_task_485);

}


void rt485::adddynamixel(int id)
{
    Dynamixel* dynamixel_servor = new Dynamixel(); //动态分配一个mitMotor对象，这里调用的是默认构造函数。
    dynamixel_servor->init(id,this);
    dynamixels.push_back(dynamixel_servor);          //push_back() 在Vector最后添加一个元素
}
