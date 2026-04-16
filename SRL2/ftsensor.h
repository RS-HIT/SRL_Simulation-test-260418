#ifndef FTSENSOR_H
#define FTSENSOR_H
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <Eigen/Eigen>


class RT_TASK;

class ftSensor
{
public:
    ftSensor();
    bool init(const char* ftsensorIP);
    void start();
    void stop();
    void forceControl(int id);
    void ftBias(int id);
    int socketHandle;
    struct hostent* he;
    struct sockaddr_in addr;
    struct command_with_para
    {
        unsigned short length;
        unsigned char sequence;
        unsigned char command;
        int parameters;
        unsigned short crc;
    }cmd_para;

    struct command_without_para
    {
        unsigned short length;
        unsigned char sequence;
        unsigned char command;
        unsigned short crc;
    }cmd_noPara;
    unsigned char request1[10];
    unsigned char request2[6];
    int judge;
    //static void* readThread(void*);

    static void         sensorTask(void * arg);
    RT_TASK*    forceControl_task;

    int num_of_cycle = 0;
    Eigen::VectorXd ft2;
    Eigen::VectorXd ft3;
    Eigen::VectorXd ft_bias2;
    Eigen::VectorXd ft_bias3;
//    double ft[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//    double fte[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//    double ftlast[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//    double delta[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

private:

    int PORT = 49152;
    unsigned short crcByte(unsigned short crc, unsigned char ch);
    unsigned short crcBuf(const void * buff, unsigned long len);
    //struct command_with_para;
    //struct command_without_para;
    void sendCmd(int cmd);


    int threadRead;
};

#endif // FTSENSOR_H
