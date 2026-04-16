#include "sensor.h"
#include <unistd.h>
#include </usr/xenomai/include/trank/native/task.h>
#include </usr/xenomai/include/trank/native/timer.h>
#include </usr/xenomai/include/trank/rtdk.h>
#include </usr/xenomai/include/trank/native/sem.h>
#include </usr/xenomai/include/trank/native/mutex.h>
#include <rtdm/testing.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <rtdm/ipc.h>
#include <malloc.h>
#include <sys/time.h>
#include <ctime>
sensor Sensor;
void sensor::unpack_reply(int len,uint ID,unsigned char DATA[8])
{

    float ru_sensor_angle_init[6] = {-173, -195, 88,213,0,0};
    if(len ==len)
    {
     uint id = ID -0x180;
     double angle = DATA[1]<< 8 | DATA[0];
     angle = angle / 100.0;

     if(id == 7 || id == 8 ) Sensor.pos= angle + ru_sensor_angle_init[id- 7];
     else if(id == 9 || id == 10 ) Sensor.pos = -angle + ru_sensor_angle_init[id - 7];

    }
}
