#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include "robot.h"
#include <pthread.h>

using namespace std;
using namespace robot;
typedef std::vector<std::string>  StringList;

enum HololensCmd_TYPE
{

    ID=22,
    BQ=32,
    TL=42,
    TR=52,
    ZR=62,
    DD=72,
    HJ=82,
    LK=92,

    CTH=13,
    INI=23,
    BD1=33,
    BD2=43,
    CTC=53,

    EyeG=14,
    CubT=24,
    WDZC=34,
    XGZY=44,
    TDGJ=54,
    CLMS=64,
    BLMS=74,
    FZHJ=84,
    HJRW=94,
    HJJS=104,


    reset=15,
    Pose1=25,
    PBack=35,

    GoOnce=16,
    NothingDo=9999,

    tool1=101,tool2=102,tool3=103,tool4=104,tool5=105,tool6=106,tool7=107,tool8=108
};

class Hololens
{
  public:
    robotArm *HOrobotArm[4];
    T265Camera *HOT265;
    HololensCmd_TYPE HololensCmd;

    struct T265HoloUse
    {
        Eigen::Vector3f  p_Old;
        Eigen::Vector3f pry_Old;
        Quaternionf q_Old;
        Eigen::Vector3f  p_now;
        Eigen::Vector3f pry_now;
        Quaternionf q_now;
    }H_T265;


    int SERV_PORT=12340;
    int sock_fd;
    int recv_num;
    char recv_buf[1024];
    int len;
    string msg;
    struct sockaddr_in addr_serv;
    struct sockaddr_in addr_client;

    //运算所需矩阵定义
    Matrix4d BaseToPointINI;
    Matrix4d EndToMark;
    Matrix4d HoloToMark;
    Matrix4d BaseToMark;

    Matrix4d BaseToHolo;
    Matrix4d BaseToT265;
    Matrix4d T265NowToT265Old;
    Matrix4d BaseNowToHolo;
    Matrix4d T265Pose;
    Matrix4d T265MXOld;
    Matrix4d T265MXNow;

    //Hololens指定目标位置矩阵
    Matrix4d HoloToPoint;
    Matrix4d BaseToPoint;
    Matrix4d BaseToPointOld;

    //用于发送机械臂的位置及四元数
    Eigen::Vector3d BaseToPoint_f;
    Quaterniond BaseToPoint_q;
    VectorXd RobotTargetJointP;

    //信息flag定义
    int flagBD=0;
    bool flagBDd=false;
    bool MoveCheck=false;

    //运算&&转换
    StringList splitstr(const std::string& str, char tag);
    Matrix4d TFtoMX(string tfinf);
    Matrix4d TQMX(Eigen::Vector3d t_f,Quaterniond t_q);
    bool MXTQ(Matrix4d MX44,Eigen::Vector3d  &p,Quaterniond &q);

    //初始化&&消息接受处理
    Hololens();
    ~Hololens();
    void init(robotArm &Box1,robotArm &Box2,robotArm &Box3,robotArm &Box4,T265Camera &mT265);
    void msg_recv();
    void msg_fun(string msg);
    void msg_send(string msg);

    //指令策略、判断
    bool CheckMoveOrNot(Matrix4d TFOld,Matrix4d TFNow);
    int SimpleRMLMove(int i,float vel,float acc,VectorXd tp);

    //发送指令
    void HoloCmd();
    void SetHoloCmdType(int CmdType);
    void SRLModelShow(VectorXd tp);

};



