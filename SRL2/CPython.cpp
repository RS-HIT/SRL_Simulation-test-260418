#include "CPython.h"
CPython::CPython(const char* ip, int port )
{
    socket_fd = socket(AF_INET, SOCK_STREAM,0);
        if(socket_fd == -1)
        {
            cout<<"socket 创建失败："<<endl;
            exit(-1);
        }
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);
    int res = connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr));
        if(res == -1)
        {
            cout<<"bind 链接失败："<<endl;
//            exit(-1);
        }
        cout<<"bind 链接成功："<<endl;
}

string CPython::callPyFun(string funName,string dataType, int data)
{

    string SendInf;
    SendInf = funName +"//" + dataType +"//" + to_string(data);

    int cllb=send(this->socket_fd, SendInf.c_str(), SendInf.length(), MSG_NOSIGNAL);
    if(cllb<0) {cout<<"send error"<<endl;return "NULL";}

    char recv_buf[1024];
    int ns1 = recv(this->socket_fd, recv_buf, sizeof(recv_buf), MSG_NOSIGNAL);
    if(ns1<0) {cout<<"recv error"<<endl;return "NULL";}
    recv_buf[ns1] = '\0';
    string InfGet;
    InfGet = recv_buf;
    return InfGet;


}
string CPython::callPyFun(string funName, string dataType, string data)
{

	string SendInf;
	SendInf = funName + "//" + dataType + "//" + data;
    int cllb=send(this->socket_fd, SendInf.c_str(), SendInf.length(), MSG_NOSIGNAL);
    if(cllb<0) {cout<<"send error"<<endl;return "NULL";}

    char recv_buf[1024];
    int ns1 = recv(this->socket_fd, recv_buf, sizeof(recv_buf), MSG_NOSIGNAL);
    if(ns1<0) {cout<<"recv error"<<endl;return "NULL";}
	recv_buf[ns1] = '\0';
	string InfGet;
	InfGet = recv_buf;
	return InfGet;
}
void CPython::EndCPython()
{
    string SendInf;
    SendInf = "EndPython//ClosePy//0";
    send(this->socket_fd, SendInf.c_str(), SendInf.length(), 0);
}
VectorXd CPython::StrToVct(string Strlist,int num)//num为数组数据个数
{
    VectorXd StrToV(num);
    if (Strlist =="[]"||Strlist =="")
    {
        return StrToV;
    }
    try{
        string tfstr=Strlist.substr(1,Strlist.length()-2);
        //printf("tfstr:%s\n",tfstr.c_str());
        StringList res = HoloFun.splitstr(tfstr, ' ');
        //printf("res0:%s\n",res[0].c_str());
        //字符串转浮点数组
        for(int i = 0; i < int(res.size()); i++)
        {
            //std::cout << res[i] << endl;
            StrToV[i]=atof(res[i].c_str());
            if(i==(num-1))
            {
                break;
            }
        }
    }
    catch(...){printf("error2");}
    return StrToV;
}
string CPython::StrFirst(string strist,int num)//num为前一个还是后一个0，1
{
    StringList StrlistR(2);
    try{
        StrlistR = HoloFun.splitstr(strist, ',');
        //printf("StrlistR:%s\n",StrlistR[0].c_str());
    }
    catch(...){printf("CPython::StrFirst Error\n");}
    return StrlistR[num];
}



