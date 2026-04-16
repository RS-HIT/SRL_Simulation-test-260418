#include "rtcan.h"
#include <libpcan.h>
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
#include "math.h"
#include "vector"
#include "mitmotor.h"
#include "magneticencoder.h"
#include <cstring>

rtCan::rtCan()
{
    can_handle = NULL;
}
void rtCan::canrxTask(void * arg)
{
    rtCan* pthis = (rtCan*)arg;
    //    unsigned long ov;

    //    float rtPeroid = pthis->m_TStep*1000000000;
    //    rt_task_set_periodic(NULL, TM_NOW, rtPeroid);

    int count = 0, count1=0;
    while(true)
    {
        pthis->canrx();
        //        rt_task_wait_period(&ov);

    }
}

void rtCan::canrx()
{

    //    for(int i=0;i<4;i++)
    //    {
    //          canTx(i+7,0,NULL);
    //    }

    TPCANMsg  canrx_msg;
    CAN_Read(can_handle, &canrx_msg);
    uint len = canrx_msg.LEN;
    //    rt_printf("len:%d\n",len);
    //     rt_printf("enter the can receive\n");
    //    rt_printf("len:%d,id_sensor:%d,lh_sensor_angle:%0.3f\n",len,id,lh_sensor_angle[1]);
    if(len == 6)
    {
        ccc1++;
        //        rt_printf("ccc1:%d\n",ccc1);
        uint id = canrx_msg.DATA[0];
        for(int i=0;i<motors.size();i++)
        {
            if(id == motors[i]->id)
            {
                motors[i]->unpack_reply(canrx_msg.DATA);
                break;
            }
        }

    }
    else if(len == 8)
    {

        //        float ru_sensor_angle_init[6] = {-80.420, -193.92, 88.22,0,0,0};
//        float sensor_angle_init[6] = {-80.420, -193.92, -269.391+180,-209.05,0,0};
        //        float rl_sensor_angle_init[6] = {-168.650, -17.580, 0,0,0,0};
        //        float sensor_angle_init[6] = {0, 0, 0,0,0,0};
        uint id = canrx_msg.ID - 0x180;
        if(id == 7)
        {
            ccc27++;
            if(ccc27%1==0)
            {
                rt_printf("ccc27:%d\n",ccc27/1);
            }
        }
        if(id == 8)
        {
            ccc28++;
            if(ccc28%1==0)
            {
                rt_printf("ccc28:%d\n",ccc28/1);
            }
        }
        if(id == 9)
        {
            ccc29++;
            if(ccc29%1==0)
            {
                rt_printf("ccc29:%d\n",ccc29/1);
            }
        }

        for(int i=0;i<encoders.size();i++)
        {
            if(id == encoders[i]->id)
            {

                encoders[i]->unpack_reply(canrx_msg.DATA,sensor_angle_init[id-7],id);
                break;

            }

        }
    }


    else if(len == 4)
    {

//        float sensor_angle_init[6] = {280.635, 40.276, 151.04,0,0,0};
        uint id = canrx_msg.ID - 0x580;
//        if((canrx_msg.ID==0x587))
//        {
//            ccc27++;
//            if(ccc27%1==0)
//            {
//                rt_printf("ccc27:%d\n",ccc27/1);
//            }

//        }
//        if((canrx_msg.ID==0x588))
//        {
//            ccc28++;
//            if(ccc28%1==0)
//            {
//                rt_printf("ccc28:%d\n",ccc28/1);
//            }

//        }
//        if((canrx_msg.ID==0x589))
//        {
//            ccc29++;
//            if(ccc29%1==0)
//            {
//                rt_printf("ccc29:%d\n",ccc29/1);
//            }

//        }

        for(int i=0;i<encoders.size();i++)
        {
            if(id == encoders[i]->id)
            {

                encoders[i]->unpack_reply1(canrx_msg.DATA,sensor_angle_init[id-7],id);
                break;

            }

        }


    }
    else
    {
        ccc3++;
        rt_printf("len=%d,ccc3:%d\n",len,ccc3);
    }


}


void rtCan::addmotor(int canid)
{
    mitMotor* motor = new mitMotor(); //动态分配一个mitMotor对象，这里调用的是默认构造函数。
    motor->init(canid,this);
//    rt_printf("5\n");
    motors.push_back(motor);          //push_back() 在Vector最后添加一个元素
}

void rtCan::addencoder(int canid)
{
    magneticEncoder* encoder = new magneticEncoder();
    encoder->init(canid,this);
//    rt_printf("6\n");
    encoders.push_back(encoder);
}

void rtCan::init(int port,float sensor_init[6])
{
    mlockall(MCL_CURRENT | MCL_FUTURE);

    can_handle = CAN_Open(HW_PCI, port);//HW_PCIE_FD

    CAN_Init(can_handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);

    for(int i = 0; i < 6; i ++) sensor_angle_init[i] = sensor_init[i];
//    memcpy(sensor_angle_init,sensor_init,sizeof(sensor_init));
//    sensor_angle_init = sensor_init;
    printf("CAN Status = %i\n", CAN_Status(can_handle));

}

int rtCan::startCanRec(const char* taskname)
{
    int i, err;
    canrx_task = new RT_TASK();
    err = rt_task_create(canrx_task, taskname, 0, 55, 0);
    if (err) {
        printf("receivetest: Failed to create canreceive rt task , code %d\n",
               errno);
        return err;
    }
    rt_task_start(canrx_task, rtCan::canrxTask, this);
    if (err) {
        printf("receivetest: Failed to start rt task, code %d\n",
               errno);
        return errno;
    }
    return err;
}

void rtCan::close()
{
    rt_task_delete(canrx_task);
    CAN_Close(can_handle);
}

void rtCan::canTx(int id, int len, BYTE* data)
{
    //    rt_printf("enter the can   send\n");
    TPCANMsg            cantx_msg;
    cantx_msg.MSGTYPE = MSGTYPE_STANDARD;
    cantx_msg.LEN = len;
    cantx_msg.ID = id;
    for(int i=0;i<len;i++)
    {
        cantx_msg.DATA[i] = data[i];
    }
    CAN_Write(can_handle, &cantx_msg);
}

//void rtCan::canTx(int id, int len, BYTE* data)
//{
////    rt_printf("enter the can   send\n");
//    TPCANMsg            cantx_msg;
//    cantx_msg.MSGTYPE = MSGTYPE_STANDARD;
//    cantx_msg.LEN = 8;
//    cantx_msg.ID = id;
//    for(int i=0;i<len;i++)
//    {
//        cantx_msg.DATA[i] = data[i];
//    }
//    CAN_Write(can_handle, &cantx_msg);
//}




