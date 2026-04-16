#include "magneticencoder.h"
#include "math.h"
#include "rtcan.h"
#include "iostream"
#include "mitmotor.h"

magneticEncoder::magneticEncoder()
{
            //可有内容，可无内容
}

bool magneticEncoder::init(int canid, rtCan* canComm)
{
    id = canid;
    m_pComm = canComm;
    return true;
}


void magneticEncoder::unpack_reply(unsigned char data[8],float init_angle,int id)
{

    double angle = data[1]<< 8 | data[0];
    angle = angle / 100.0;


    pos = angle + init_angle;

    if(pos>180)
    {
        pos-=360;
    }
    else if(pos<-180)
    {
        pos+=360;
    }

    if(id ==9||id ==10)
    {
        pos = -pos;
    }
    else
    {
        pos = pos;
    }



    //    printf("angle:%0.3f\n",angle);
//    if(abs(pos-pos_last)>180 && pos>180)
//    {
//        if(id ==9||id ==10)
//        {
//            pos = -pos+360;
//        }
//        else
//        {
//            pos = pos-360;
//        }
//    }
//    else if(abs(pos-pos_last)>180 && pos<180)
//    {
//        if(id ==9||id ==10)
//        {
//            pos = -pos-360;
//        }
//        else
//        {
//            pos = pos+360;
//        }
//    }
//    else
//    {
//        if(id ==9||id ==10)
//        {
//            pos = -pos;
//        }
//        else
//        {
//            pos = pos;
//        }
//    }

//    pos_last = pos;

}

void magneticEncoder::unpack_reply1(unsigned char data[4],float init_angle,int id)
{
    unsigned int angle_int;
    double angle;

    angle_int = data[2];
    angle_int = angle_int<<8|data[3];
    angle = angle_int*360/16384.0;



//    magnet_pos = angle - init_angle;
//    magnet_pos = 2*PI-magnet_pos*PI/180;

     pos = angle - init_angle;

    if(pos>180)
    {
        pos-=360;
    }
    else if(pos<-180)
    {
        pos+=360;
    }



    if(id ==7 || id ==8)
    {
        pos = -pos;
//        rt_printf("angel:%f\n",pos);
    }
    else
    {
        pos = pos;
//           rt_printf("angel:%f\n",pos);
    }

}

void magneticEncoder::unpack_reply2(unsigned char data[8],float init_angle)
{
    double angle = data[1]<< 8 | data[0];
    angle = angle / 100.0;
    pos = -angle + init_angle;
}


void magneticEncoder::getPos(float &p)
{
   p = pos;
}

void magneticEncoder::setZeroPos()
{
   pos_init = pos*3.1416/180;
}
