#include "ftsensor.h"
#include </usr/xenomai/include/trank/native/task.h>
#include </usr/xenomai/include/trank/native/timer.h>
#include </usr/xenomai/include/trank/rtdk.h>
#include </usr/xenomai/include/trank/native/sem.h>
#include </usr/xenomai/include/trank/native/mutex.h>
#include <sys/mman.h>

using namespace std;

ftSensor::ftSensor()
{

}

//ftsensorIP = "192.168.199.101"
bool ftSensor::init(const char* ftsensorIP)
{
    ft2 = Eigen::VectorXd::Zero(6);
    ft3 = Eigen::VectorXd::Zero(6);
    ft_bias2 = Eigen::VectorXd::Zero(6);
    ft_bias3 = Eigen::VectorXd::Zero(6);
    int err;

    /* Open the socket. */
    socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketHandle == -1)
    {
        exit(1);
        return false;
    }

    he = gethostbyname(ftsensorIP);
    memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);

    err = connect(socketHandle, (struct sockaddr*)&addr, sizeof(addr));
    if (err == -1)
    {
        exit(2);
        return false;
    }
    return true;

}

unsigned short ftSensor::crcByte(unsigned short crc, unsigned char ch) // Lookup table
{
    static const unsigned short ccitt_crc16_table[256] =
    {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
    };
    return ccitt_crc16_table[((crc >> 8) ^ ch) & 0xff] ^ (crc << 8);
}

unsigned short ftSensor::crcBuf(const void * buff, unsigned long len)
{
    unsigned long i;
    unsigned short crc = 0x1234;
    const char * buf = (const char *)buff;

    for(i = 0; i < len; i++)
    {
        crc = crcByte(crc, buf[i]);
        //printf("buf[%d]:%d\n",i,buf[i]);
    }
    //printf("crc:%d\n",crc);
    return crc;
}



void ftSensor::sendCmd(int cmd)
{
    //Command with parameter


    cmd_para.length = 10;
    cmd_para.sequence = 0;
    cmd_para.command = 1;
    cmd_para.parameters = 0;
    cmd_para.crc = crcBuf(&cmd_para,cmd_para.length-2);

    *(unsigned short*)&request1[0] = htons(cmd_para.length);
    *(unsigned char*)&request1[2] = cmd_para.sequence;
    *(unsigned char*)&request1[3] = cmd_para.command;
    *(unsigned long*)&request1[4] = htonl(cmd_para.parameters);
    *(unsigned short*)&request1[8] = htons(cmd_para.crc);

    //Command without parameter


    cmd_noPara.length = 6;
    cmd_noPara.sequence = 0;
    cmd_noPara.command = 2;
    cmd_noPara.crc = crcBuf(&cmd_noPara,cmd_noPara.length-2);

    *(unsigned short*)&request2[0] = htons(cmd_noPara.length);
    *(unsigned char*)&request2[2] = cmd_noPara.sequence;
    *(unsigned char*)&request2[3] = cmd_noPara.command;
    *(unsigned short*)&request2[4] = htons(cmd_noPara.crc);

    //printf("Set request\n");
    int sendReturn;
    if(cmd == 1)
    {
        //printf("IN\n");
        sendReturn = send(socketHandle, request1, 10, 0);
        //printf("%d\n", sendReturn);
    }
    else if(cmd == 2)
    {
        send(socketHandle, request2, 6, 0);
    }
    //printf("Request has been sent\n");
}


//void ftSensor::sensorTask(void* arg)
//{
//    ftSensor* pthis = (ftSensor* )arg;

//    while (true)
//    {
//       pthis->forceControl();
//    }

//}

void ftSensor::ftBias(int id)
{
    int i,n;
    unsigned char response[90];
    signed long data[6];
    char* AXES[] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };
    int aa, id_count;
    double bb;

    int rcbytes = recv(socketHandle, response, 90, 0);

    if(rcbytes == 90)
    {
        num_of_cycle = num_of_cycle + 1;
        if(id == 3)
        {
            id_count =42;
            for (i = 0; i < 6; i++)
            {
                data[i] = ntohl(*(signed long*)&response[id_count + i * 4]);
                aa = (int)data[i];
                bb = aa*1.0;
                ft_bias2(i) = bb/1000000;
            }
        }else{
            id_count =66;
            for (i = 0; i < 6; i++)
            {
                data[i] = ntohl(*(signed long*)&response[id_count + i * 4]);
                aa = (int)data[i];
                bb = aa*1.0;
                ft_bias3(i) = bb/1000000;
            }
        }
        //printf("FTSenor Bias!\n");
    }
    else
    {
        printf("Recv wrong:%d\n",rcbytes);
    }
}

void ftSensor::forceControl(int id)
{        
    int i,n;
    unsigned char response[90];
    signed long data[6];
    //double ft[6];
    char* AXES[] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };
    int aa, id_count;
    double bb;

//    double deadBand[6] = {0.5, 0.5, 0.5, 0.05, 0.05, 0.05};
//    double ftexp[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//    //double delta[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//    double k[6] = {0.000008, 0.000008, 0.000008, 0.002, 0.002, 0.002};
//    double b[6] = {0.000002, 0.000002, 0.000002, 0.0005, 0.0005, 0.0005};
//    double kf = 0.000008;
//    double bf = 0.000002;
//    double kt = 8;
//    double bt = 2;

    int rcbytes = recv(socketHandle, response, 90, 0);

    if(rcbytes == 90)
    {
        num_of_cycle = num_of_cycle + 1;
//        if(num_of_cycle % 100 == 0)
//        {
//            printf("num_of_cycle:%d\n",num_of_cycle);
//        }
        //printf("num_of_cycle:%d\n",num_of_cycle);
        if(id == 1)
        {
            id_count =42;
            for (i = 0; i < 6; i++)
            {
                data[i] = ntohl(*(signed long*)&response[id_count + i * 4]);
                aa = (int)data[i];
                bb = aa*1.0;
    //            ft[i] = bb/1000000;
                ft2(i) = bb/1000000;
                ft2(i) = ft2(i) - ft_bias2(i);
                //printf("%s: %ld, %d, %f, %f\t\n", AXES[i], data[i], aa, bb, ft[i]);
                //printf("%s: %f\n", AXES[i], ft[i]);
            }
        }

        if(id ==2)
        {
            id_count =66;
            for (i = 0; i < 6; i++)
            {
                data[i] = ntohl(*(signed long*)&response[id_count + i * 4]);
                aa = (int)data[i];
                bb = aa*1.0;
    //            ft[i] = bb/1000000;
                ft3(i) = bb/1000000;
                ft3(i) = ft3(i) - ft_bias3(i);
                //printf("%s: %ld, %d, %f, %f\t\n", AXES[i], data[i], aa, bb, ft[i]);
               // printf("%s: %f\n", AXES[i], ft[i]);
            }
        }



        //****************force control*****************
//        for (n = 0; n < 6; n++)
//        {
//            fte[n] = ft[n] - ftexp[n];
//            if(fte[n] > deadBand[n]){
//                fte[n] = fte[n] - deadBand[n];
//            }
//            else if (fte[n] < -deadBand[n]) {
//                fte[n] = fte[n] - deadBand[n];
//            }
//            else {
//                fte[n] = 0.0;
//            }
//            delta[n] = k[n] * fte[n] + b[n] * (fte[n] - ftlast[n]);
//            ftlast[n] = fte[n];
//        }
    }
    else
    {
        printf("Recv wrong:%d\n",rcbytes);
    }



}

void ftSensor::start()
{
    sendCmd(1);
    printf("Start ft reading\n");

//    int err;
//    mlockall(MCL_CURRENT | MCL_FUTURE);

//    forceControl_task = new RT_TASK();
//    err = rt_task_create(forceControl_task, "forceControlTask", 0, 99, 0);
//    if (err) {
//        printf("receivetest: Failed to create ftSensor rt task, code %d\n",
//               errno);
//    }
//    rt_task_start(forceControl_task, ftSensor::sensorTask, this);
//    if (err) {
//        printf("receivetest: Failed to start ftSensor rt task, code %d\n",
//               errno);
//    }

}

void ftSensor::stop()
{
    sendCmd(2);
    //rt_task_delete(forceControl_task);
    printf("Stop ft reading\n");
}
