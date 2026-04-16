#include<stdio.h>
#include<sys/types.h>
#include<stdlib.h>
#include<string>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>
#include "Hololens.h"
using namespace std;

class CPython
{
private:
    int socket_fd;
    struct sockaddr_in addr;
    Hololens HoloFun;
public:
	CPython(const char* ip = "127.0.0.1", int port = 8888);
	string callPyFun(string, string, int);
	string callPyFun(string, string, string);
    void EndCPython();

    //将T，Q由字符串转化为数组
    string StrFirst(string strist,int num);
    VectorXd StrToVct(string Strlist,int num);
};
