#include "Hololens.h"

//线程开启
static void* startMsgRecv(void* arg)
{
    Hololens* pHere = (Hololens*)arg;
    pHere->msg_recv();
    return 0;
}
static void* startMsgCmd(void* arg)
{
    Hololens* pHere = (Hololens*)arg;
    pHere->HoloCmd();
    return 0;
}


//用字符分割字符串
StringList Hololens::splitstr(const std::string& str, char tag)
{
    StringList  li;
    std::string subStr;

    //遍历字符串，同时将i位置的字符放入到子串中，当遇到tag（需要切割的字符时）完成一次切割
    //遍历结束之后即可得到切割后的字符串数组
    for(size_t i = 0; i < str.length(); i++)
    {
        if(tag == str[i]) //完成一次切割
        {
            if(!subStr.empty())
            {
                li.push_back(subStr);
                subStr.clear();
            }
        }
        else //将i位置的字符放入子串
        {
            subStr.push_back(str[i]);
        }
    }

    if(!subStr.empty()) //剩余的子串作为最后的子字符串
    {
        li.push_back(subStr);
    }

    return li;
}
//unity Tf&Q to MX
Matrix4d Hololens::TFtoMX(string tfinf)
{
    Matrix4d m44;
    Eigen::Vector3d t_f;
    Eigen::Vector4d t_Q;
    //字符处理
    int pos=tfinf.find(")(");
    VectorXd StrToV(7);

    try
    {
        while (pos != -1)
        {
           tfinf.replace(pos,string(")(").length(),",");
           pos=tfinf.find(")(");
        }
        string tfstr=tfinf.substr(1,tfinf.length()-2);
        StringList res = splitstr(tfstr, ',');

        //字符串转浮点数组
        for(int i = 0; i < int(res.size()); i++)
        {
            //std::cout << res[i] << endl;
            StrToV[i]=atof(res[i].c_str());
            if(i==6)
            {
                break;
            }
        }

    }catch (...) {
        printf("Found Error In Hololens TFtoMX");
    }

    //生成t_f,q
    for(int j = 0;j<7;j++)
    {
        if(j<3)
        {
            t_f[j]=StrToV[j];
        }
        else
        {
            t_Q[j-3]=StrToV[j];
        }
    }
    t_f[2]=-t_f[2];
    t_Q[0]=-t_Q[0];
    t_Q[1]=-t_Q[1];

    Quaterniond t_q(t_Q);
    //t_f,q转化为齐次矩阵
    m44=TQMX(t_f,t_q);

    return m44;
}
//Common t_f & t_q to MX
Matrix4d Hololens::TQMX(Eigen::Vector3d t_f,Quaterniond t_q)
{
    Matrix4d MX44;
    Eigen::Matrix3d R4;
    R4=t_q.toRotationMatrix();
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            MX44(i,j)=R4(i,j);
        }
        MX44(i,3)=t_f[i];
        MX44(3,i)=0;
    }
    MX44(3,3)=1;

    return MX44;
}
//Common  MX to t_f & t_q
bool Hololens::MXTQ(Matrix4d MX44,Eigen::Vector3d &p,Quaterniond &q)
{
    bool flag_MXTQ=true;
    Eigen::Matrix3d R4;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R4(i,j)=MX44(i,j);
        }
        p[i]=MX44(i,3);
    }
    Quaterniond qGet(R4);
    q=qGet;
    return flag_MXTQ;
}

bool Hololens::CheckMoveOrNot(Matrix4d TFOld,Matrix4d TFNow)
{
    bool MoveOrNot=false;
    Eigen::Vector3d p_old;
    Eigen::Vector3d p_now;
    Eigen::Quaterniond q_old;
    Eigen::Quaterniond q_now;
    MXTQ(TFOld,p_old,q_old);
    MXTQ(TFNow,p_now,q_now);
    Eigen::Vector3d RXYZ_old=q_old.matrix().eulerAngles(0,1,2);
    Eigen::Vector3d RXYZ_now=q_now.matrix().eulerAngles(0,1,2);
    for(int i=0;i<3;i++)
    {
        if(fabs(p_old[i]-p_now[i])>0.001)
        {
            MoveOrNot=true;
        }
        if(fabs(RXYZ_old[i]-RXYZ_now[i])>1)
        {
            MoveOrNot=true;
        }
    }
    return MoveOrNot;

}

int Hololens::SimpleRMLMove(int i,float vel,float acc,VectorXd tp)
{
    if(HOrobotArm[i]->getMotionState() == SM_STATE::STOPPED)
    {
        HOrobotArm[i]->clearRmlPoints();
        HOrobotArm[i]->addRmlPoint(MOTION_TYPE::JOINT_RML,tp,vel,acc);
        HOrobotArm[i]->beginMotion(MOTION_TYPE::JOINT_RML);
        return 1;
    }
    else
    {
        return 0;
    }
}
//----------------逆运动学求解---写在了dynamics中。-------------------------

Hololens::Hololens()
{

}

Hololens::~Hololens()
{
    printf("Hololens Socket Closed \n");
    close(sock_fd);
}

void Hololens::init(robotArm &Box1,robotArm &Box2,robotArm &Box3,robotArm &Box4,T265Camera &mT265)
{
    HOrobotArm[0]=&Box1;
    HOrobotArm[1]=&Box2;
    HOrobotArm[2]=&Box3;
    HOrobotArm[3]=&Box4;
    HOT265=&mT265;
    BaseToPointINI=TQMX(Eigen::Vector3d(0,0,0),Quaterniond(1,0,0,0));
    BaseToPointOld=BaseToPointINI;

    //EndToMark=TQMX(Eigen::Vector3d(-0.085,0,0),Quaterniond(1,0,0,0));
    //EndToMark=TQMX(Eigen::Vector3d(-0.2,0.023,0),Quaterniond(1,0,0,0));
    EndToMark=TQMX(Eigen::Vector3d(-0.34,0.023,0),Quaterniond(1,0,0,0));
    BaseToT265=TQMX(Eigen::Vector3d(0.051,-0.097,0),Quaterniond(0.5,0.5,-0.5,-0.5));

    //Using Zero Pose
    Math::Vector3d pos;
    Math::Matrix3d qri;

//    VectorXd jp0(HOrobotArm->m_Dofs);
//    jp0 << 0,0,0;
//    HOrobotArm->m_dynamics.forwardKinematics(jp0,pos,rpy);
    HOrobotArm[3]->m_dynamics.getEndPose(pos,qri);
    Eigen::Quaterniond QBD(qri);
    //BaseToMark=TQMX(Eigen::Vector3d(pos[0]-0.06-0.065,pos[1]+0.01,pos[2]+0.04),Quaterniond(1,0,0,0));
    //BaseToMark=TQMX(Eigen::Vector3d(0,-0.074+0.01,0.093),Quaterniond(1,0,0,0));
    BaseToMark=TQMX(pos,QBD)*EndToMark;

    //--------
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
      perror("socket");
      exit(1);
    }

    /* 将套接字和IP、端口绑定 */
    memset(&addr_serv, 0, sizeof(struct sockaddr_in));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_port = htons(SERV_PORT);
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
    len = sizeof(addr_serv);

    /* 绑定socket */
    if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
    {
      perror("bind error:");
      exit(1);
    }
    //开启消息等待线程
    pthread_t tids[2];
    int HoloThread1 = pthread_create(&tids[0], NULL, startMsgRecv, this);
    if (HoloThread1 == 0)
    {
       printf("Hololens Msg_recv Thread Started\n");
    }
    int HoloThread2 = pthread_create(&tids[0], NULL, startMsgCmd, this);
    if (HoloThread2 == 0)
    {
       printf("Hololens startMsgCmd Thread Started\n");
    }

//    RobotTargetJointP=Eigen::Vector3d(0,60,-50);
//    SRLModelShow(RobotTargetJointP);

}

void Hololens::msg_recv()
{
    printf("servermsg wait:\n");
    while(1)
    {
      recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);

      if(recv_num < 0)
      {
        perror("recvfrom error:");
        continue;
      }

      recv_buf[recv_num] = '\0';
      //printf("server receive %d bytes: %s\n", recv_num, recv_buf);
      msg=recv_buf;
      if(msg=="close_socket")
      {
        close(sock_fd);
      }
      else
      {
        msg_fun(msg);
        //HoloCmd();
      }

    }
}
void Hololens::msg_send(string msg)
{
    int sendBytes;
    sendBytes = sendto(sock_fd, msg.c_str(), msg.length(), 0,(struct sockaddr *)&addr_client, len);
    if(sendBytes<0)
    {
       printf("Send Failed: %s \n",msg.c_str());
    }
}

void Hololens::msg_fun(string msg)
{
    if(msg.substr(0,5)=="reset")
    {
        printf("reset: %s\n", msg.c_str());
        HololensCmd=reset;
    }
    else if(msg.substr(0,5)=="Pose1")
    {
        HololensCmd=Pose1;
        printf("Robot Going To : %s\n", msg.c_str());
    }
    else if(msg.substr(0,5)=="PBack")
    {
        HololensCmd=PBack;
        printf("Robot Going To : %s\n", msg.c_str());
    }
    else if(msg.substr(0,3)=="INI")
    {
        HololensCmd=INI;
    }
    else if(msg.substr(0,4)=="tool")
    {
        printf("tool: %s\n", msg.c_str());
    }
    else if(msg.substr(0,3)=="ID:")
    {
        printf("ID: %s\n", msg.c_str());
        HololensCmd=ID;
    }
    else if(msg.substr(0,4)=="WDZC")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=WDZC;
    }
    else if(msg.substr(0,4)=="XGZY")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=XGZY;
    }
    else if(msg.substr(0,2)=="ZR")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=ZR;
    }
    else if(msg.substr(0,2)=="DD")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=DD;
    }
    else if(msg.substr(0,4)=="TDGJ")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=TDGJ;
    }
    else if(msg.substr(0,4)=="CLMS")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=CLMS;
    }
    else if(msg.substr(0,4)=="BLMS")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=BLMS;
    }
    else if(msg.substr(0,4)=="FZHJ")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=FZHJ;
    }
    else if(msg.substr(0,4)=="HJRW")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=HJRW;
    }
    else if(msg.substr(0,2)=="LK")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=LK;
    }
    else if(msg.substr(0,4)=="HJJS")
    {
        printf("CMD Now: %s\n", msg.c_str());
        HololensCmd=HJJS;
    }

    else
    {
        if(msg.substr(0,4)=="BD1:")
        {
            printf("BD One Now %s\n", msg.c_str());
            flagBD=1;
            HololensCmd=BD1;
        }
        if(msg.substr(0,4)=="BD2:")
        {
            printf("BD Two Now %s\n", msg.c_str());
            flagBD=2;
            HololensCmd=BD2;
        }
        else if(msg.substr(0,3)=="BQ:")
        {
            printf("BQ: %s\n", msg.c_str());
            HololensCmd=BQ;
        }
        else if(msg.substr(0,5)=="EyeG:")
        {
            printf("Eye Control %s\n", msg.c_str());
            HololensCmd=EyeG;
        }
        else if(msg.substr(0,4)=="CTH:")
        {
            printf("Come To Hand: %s\n", msg.c_str());
            HololensCmd=CTH;
        }
        else if(msg.substr(0,4)=="CTC:")
        {
            printf("Catch The Cable: %s\n", msg.c_str());
            HololensCmd=CTC;
        }
        else if(msg.substr(0,3)=="TF:")
        {
            printf("TF: %s\n", msg.c_str());
        }
        else if(msg.substr(0,5)=="CubT:")
        {
            printf("HoloTesting: %s\n", msg.c_str());
            HololensCmd=CubT;
        }
        else if(msg.substr(0,3)=="HJ:")
        {
//            printf("HoloHJ: %s\n", msg.c_str());
            HololensCmd=HJ;
        }
    }

}

void Hololens::SetHoloCmdType(int CmdType)
{
    HololensCmd=HololensCmd_TYPE(CmdType);
//    printf("SetHololensCmd %d \n",HololensCmd);
}
void Hololens::SRLModelShow(VectorXd tp)
{
    string MsgFroV;
    MsgFroV="SRLFK"+to_string(tp[0])+","+to_string(tp[1])+","+to_string(tp[2])+","+to_string(tp[3])+","+to_string(tp[4])+","+to_string(tp[5]);
    printf("SendingMSG:%s \n",MsgFroV.c_str());
    msg_send(MsgFroV);

}

void Hololens::HoloCmd()
{
    float gSpeed=0.2f;
    RobotTargetJointP=Eigen::VectorXd(6);
    RobotTargetJointP<<0,0,0,0,0,0;

    while(true)
    {
        //sleep(0.01);
        sleep(0.005);
//        if(HololensCmd==FZHJ)
//        {
//            //HOrobotArm[3]->m_485Comm.dynamixels[2]->enableDynamixel(1);
//            printf("辅助焊接开始");
//        }
        if(HololensCmd==HJ){
            float grPos=atof(msg.substr(3).c_str());
           // cout<<"HJPose:"<<grPos<<endl;
            //VectorXd tp0(6);
            //tp0 << HOrobotArm[3]->m_Desire.posd[0], HOrobotArm[3]->m_Desire.posd[1], HOrobotArm[3]->m_Desire.posd[2], HOrobotArm[3]->m_Desire.posd[3],  HOrobotArm[3]->m_Desire.posd[4], grPos * PI / 180.0f;
            HOrobotArm[3]->m_Desire.posd[5]=grPos* PI / 180.0f;
            //初始状态爪子水平
            HOrobotArm[3]->m_dynamics.setJointPositions(HOrobotArm[3]->m_Desire.posd);
            HOrobotArm[3]->m_485Comm.dynamixels[2]->setPos(grPos);
            HololensCmd=NothingDo;
        }


        //printf("HololensCmd Now %d \n",HololensCmd);
        //HOT265->getPose(H_T265.p_now,H_T265.pry_now,H_T265.q_now);
        //printf("T265PoseNow: %f %f %f \n",H_T265.pry_now[0],H_T265.pry_now[1],H_T265.pry_now[2]);

//        if(HololensCmd==reset)
//        {
//            if(HOrobotArm->getMotionState() == SM_STATE::STOPPED)
//            {
//                RobotTargetJointP<<0,0,0,0,0,0;
//                SRLModelShow(RobotTargetJointP);

//                printf("gohome\n");
//                SimpleRMLMove(2.0f*gSpeed,4.0f*gSpeed,RobotTargetJointP);
//                BaseToPointOld=BaseToPointINI;
//                HololensCmd=GoOnce;
//            }
//        }
//        if(HololensCmd==Pose1)
//        {
//            RobotTargetJointP<<0,60.0f * PI / 180.0f,-90.0f* PI / 180.0f,0,0,0;
//            SRLModelShow(RobotTargetJointP);

//            printf("goPose1\n");
//            SimpleRMLMove(2.0f*gSpeed,4.0f*gSpeed,RobotTargetJointP);
//            HololensCmd=GoOnce;
//
//        if(HololensCmd==PBack)
//        {
//            printf("goBackPose\n");
//            if(HOrobotArm->getMotionState() == SM_STATE::STOPPED)
//            {
//                float vel = 2.0f*gSpeed;
//                float acc = 4.0f*gSpeed;
//                VectorXd jp0(HOrobotArm->m_Dofs),jp1(HOrobotArm->m_Dofs);
//                VectorXd tp0(HOrobotArm->m_Dofs),tp1(HOrobotArm->m_Dofs);
//                Math::Vector3d pos,rpy;
//                HOrobotArm->m_dynamics.getEndPose(pos,rpy);

//                tp0 <<  pos[0],pos[1],pos[2]+0.05f,0,0,0;
//                jp1 << 0,60.0f * PI / 180.0f,-90.0f* PI / 180.0f,0.0 * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
//                HOrobotArm->clearRmlPoints();
//                HOrobotArm->addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
//                HOrobotArm->addRmlPoint(MOTION_TYPE::JOINT_RML,jp1,vel,acc);
//                HOrobotArm->beginMotion(MOTION_TYPE::JOINT_RML);

//                RobotTargetJointP<<0,60.0f * PI / 180.0f,-90.0f* PI / 180.0f,0,0,0;
//                SRLModelShow(RobotTargetJointP);
//            }

//            HololensCmd=GoOnce;
//        }

//        if(flagBD == 1)
//        {
//    //        Math::Vector3d posBD;
//    //        Math::Matrix3d oriBD;
//    //        Matrix4d MXBaseToEnd;
//    //        HOrobotArm->m_dynamics.getEndPose(posBD,oriBD);
//    //        Eigen::Quaterniond QBD(oriBD);
//    //        MXBaseToEnd=TQMX(posBD,QBD);
//    //        BaseToMark=MXBaseToEnd*EndToMark;

//            HoloToMark=TFtoMX(msg.substr(4));
//            BaseToHolo=BaseToMark*(HoloToMark.inverse());

//            HOT265->getPose(H_T265.p_Old,H_T265.pry_Old,H_T265.q_Old);
//            T265MXOld=TQMX(H_T265.p_Old.cast<double>(),H_T265.q_Old.cast<double>());
//            printf("BD Robot && Hololens .......First Step \n");
//            flagBD=0;
//            flagBDd=true;
//        }
//        else if(flagBDd == true)
//        {
//            HOT265->getPose(H_T265.p_now,H_T265.pry_now,H_T265.q_now);
//            //printf("T265PoseNow: %f %f %f \n",H_T265.p_now[0],H_T265.p_now[1],H_T265.p_now[2]);
//            T265MXNow=TQMX(H_T265.p_now.cast<double>(),H_T265.q_now.cast<double>());
//            T265Pose=(T265MXOld.inverse())*T265MXNow;
//            BaseNowToHolo=BaseToT265*(T265Pose.inverse())*(BaseToT265.inverse())*BaseToHolo;
//            if(HololensCmd==CTH || HololensCmd==CubT ||HololensCmd==EyeG||HololensCmd==CTC)
//            {
//                HoloToPoint=TFtoMX(msg.substr(HololensCmd%10+1));
//                //--------暂时不用相机数据
//                //BaseToPoint=BaseNowToHolo*HoloToPoint;
//                BaseToPoint=BaseToHolo*HoloToPoint;
//                MXTQ(BaseToPoint,BaseToPoint_f,BaseToPoint_q);
//                //MoveCheck=CheckMoveOrNot(BaseToPointOld,BaseToPoint);
//                MoveCheck=true;

//                if((HOrobotArm->getMotionState() == SM_STATE::STOPPED)&& MoveCheck==true)
//                {
//                    BaseToPointOld=BaseToPoint;
//                    float vel = 2.0f*gSpeed;
//                    float acc = 4.0f*gSpeed;
//                    VectorXd BaseToPoint_f_send(6);

//                    BaseToPoint_f_send<<BaseToPoint_f[0]-0.04f,BaseToPoint_f[1]-0.03f,BaseToPoint_f[2],0,0,0;

//                    //进行运动控制
//                    HOrobotArm->clearRmlPoints();
//                    //HOrobotArm->addRmlPoint(MOTION_TYPE::CARTESIAN_RML_TRANSLATION,Eigen::Vector3d(BaseToPoint_f_send[0]-0.02,BaseToPoint_f_send[1],BaseToPoint_f_send[2]+0.02),vel,acc);
//                    HOrobotArm->addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,BaseToPoint_f_send,vel,acc);
//                    HOrobotArm->beginMotion(MOTION_TYPE::JOINT_RML);

//                    //发送模型显示数据,暂时不跟踪旋转
//                    Math::Vector3d posXX,rpyXX;
//                    HOrobotArm->m_dynamics.getEndPose(posXX,rpyXX);
//                    HOrobotArm->m_dynamics.HoloInverseKinematics(BaseToPoint_f,rpyXX,RobotTargetJointP);
//                    SRLModelShow(RobotTargetJointP);

//                    printf("Robot Moving To Point:%f,%f,%f \n",BaseToPoint_f_send[0],BaseToPoint_f_send[1],BaseToPoint_f_send[2]);
//                    //先不做动基座反馈,只走一次
//                    HololensCmd=GoOnce;
//                }
//            }


//        }

    }

}




