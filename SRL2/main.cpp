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

#include "robot.h"
#include <unistd.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include "Dynamixel.h"
#include "CPython.h"
#include <iostream>
#include <fstream>
#include <string>
#include  "thread"
#include <ftsensor.h>
#include "mujocosim.h"
#include <cmath>
// #include <QDebug>
#define max 100
using namespace std;
using namespace  robot;
T265Camera m_t265;
mujocosim m_mujoco;
enum task_state
{
    fzby = 0,
    gdzc = 1,
    jxzy = 2,
    xgzy = 3,
    hjrw = 4,
    xlzp = 5
};

enum task_state_support
{
    none = 0,
    support = 1,
    force = 2,
    back = 3,
    gblance = 4

};

RT_TASK Top_data_Task;
bool Quitflag = false;
int m_task_state= 0;
//char *a[4]={"辅助搬运场景","焊接任务场景","精细作业场景","悬挂作业场景"};
//char *b[3]={"双臂模式","右臂模式","左臂模式"};
const char *a[6]={"load","suppot","hole","sky","welding","rope"};
const char *b[3]={"limb_double","limb_right","limb_left"};

ftSensor m_ftsensor;
robotArm arms[4];
rtCan  canPorts[4];
rt485  m_485Ports[2];
dynamics rbdls[4];
vrepsim  vrep;
Hololens Holo;
CPython phone;
float gSpeed = 0.5f;//0.1f;
const int graph_curves = 6;
float curvedata[graph_curves];

int ids=0;
char cmd;
int count1 =0;
int count2 =0;
float m_time = 0.0;
int state_support = task_state_support::none;
Eigen::Vector3d d435i_p;
Eigen::Matrix3d   d435i_t;
Eigen::Vector3d d435i_p22;
Eigen::Matrix3d   d435i_t22;
Eigen::Vector3d Hand_End_P,Hand_End_P1,Hand_End_V,Hand_End_V1;
float  Hand_End_V_xy,Hand_End_V1_xy;
Eigen::Vector3d Limb_End_P, Limb_End_P1;
Eigen::Vector3d  Hand_End_P_Last,Hand_End_P1_Last;
Eigen::Vector3d  Hand_End_V_Last,Hand_End_V1_Last;
//Hand_End_P_Last = VectorXd::Zero(3);
//,Hand_End_P1_last;
//Hand_End_P_Last << 0.0, 0.0, 0.0;


Eigen::Vector3d Tar_p;
Eigen::Matrix3d Tar_t;
int flagForReset=0;
int flag_f = 0;
struct
{
    vector<double> t;
    vector<double> force_r;  //jointToMotor：jpos_d转换得
    vector<double> force_l;

    vector<double> hand_x_r;
    vector<double> hand_y_r;
    vector<double> hand_z_r;

    vector<double> hand_x_l;
    vector<double> hand_y_l;
    vector<double> hand_z_l;

    vector<double> limb_x_r;
    vector<double> limb_y_r;
    vector<double> limb_z_r;

    vector<double> limb_x_l;
    vector<double> limb_y_l;
    vector<double> limb_z_l;

    vector<double> hand_vx_r;
    vector<double> hand_vy_r;
    vector<double> hand_vz_r;

    vector<double> hand_vx_l;
    vector<double> hand_vy_l;
    vector<double> hand_vz_l;

    vector<double> vxy_mean_l;
    vector<double> vxy_mean_r;

    vector<double> vz_mean_l;
    vector<double> vz_mean_r;


    vector<double> hand_vxy200_l;
    vector<double> hand_vxy200_r;

    vector<double> hand_vz200_l;
    vector<double> hand_vz200_r;

    double hand_vxy_mean_l;
    double hand_vxy_mean_r;
    double hand_vz_mean_l;
    double hand_vz_mean_r;
    //    VectorXd hand_v200;
} m_support;

struct
{
    Eigen::Vector3f    p;
    Eigen::Vector3f    p0;
    Eigen::Vector3f    p0_init;
    Eigen::Vector3f    pry;
    Eigen::Quaternionf q;

    Eigen::Quaternionf q_last;
    Eigen::Vector3f    v;
    Eigen::Vector3f  a;
    Eigen::Matrix3d mat;
    Eigen::Matrix3d mat0;
}cam[5];

struct
{
    double theta;
    Eigen::Vector3d axs;
    Eigen::Vector3d euler;
    Eigen::Matrix3d mat;
    Eigen::Quaternionf q;
    Eigen::AngleAxisd rotation_vector;
}joint2_r,joint1_r,joint2_l,joint1_l;

double sum_vector( vector<double> vector)
{
    double sum=0.0;
    int len = vector.size();
    for(int i=0; i<len; i++)
    {
        sum = sum + vector[i];
    }
    return sum;
}

static void toEulerAngle(Quaternionf q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}


Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    return R;
}

Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    cout << "Quaterniond2Euler result is:" <<endl;
    cout << "x = "<< euler[2] << endl ;
    cout << "y = "<< euler[1] << endl ;
    cout << "z = "<< euler[0] << endl << endl;
    return euler;
}

Matrix4d PRtoT(Eigen::Vector3d P,Eigen::Matrix3d R)
{
    Matrix4d T;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            T(i,j)=R(i,j);
        }
        T(i,3)=P[i];
        T(3,i)=0;
    }
    T(3,3)=1;

    return T;
}

bool TtoPR(Matrix4d T,Eigen::Vector3d &p, Eigen::Matrix3d &r)
{
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            r(i,j)=T(i,j);
        }
        p[i]=T(i,3);
    }
    return true;
}


void transpose(Eigen::Matrix3d& a, Eigen::Matrix3d& b)
{
    for (int i = 0; i < 3;i++)
    {
        for (int j = 0; j < 3; j++)
        {
            b(j,i) = a(i,j);
        }
    }
    return;
}

// Function:求矩阵的逆
// para:A-矩阵A
// return：矩阵A的逆矩阵
Eigen::Matrix4d inverse(Eigen::Matrix4d A)
{
    double E_Matrix[4][4];
    double mik;
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            if(i == j)
                E_Matrix[i][j] = 1.00;
            else
                E_Matrix[i][j] = 0.00;
        }
    }
    double CalcuMatrix[4][8];
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            CalcuMatrix[i][j] = A(i,j);
        }
        for(int k = 4; k < 8; k++)
        {
            CalcuMatrix[i][k] = E_Matrix[i][k-4];
        }
    }

    for(int i = 1; i <= 4-1; i++)
    {
        for(int j = i+1; j <= 4; j++)
        {
            mik = CalcuMatrix[j-1][i-1]/CalcuMatrix[i-1][i-1];
            for(int k = i+1;k <= 8; k++)
            {
                CalcuMatrix[j-1][k-1] -= mik*CalcuMatrix[i-1][k-1];
            }
        }
    }
    for(int i=1;i<=4;i++)
    {
        double temp = CalcuMatrix[i-1][i-1];
        for(int j=1;j<=8;j++)
        {
            CalcuMatrix[i-1][j-1] = CalcuMatrix[i-1][j-1]/temp;
        }
    }
    for(int k=4-1;k>=1;k--)
    {
        for(int i=k;i>=1;i--)
        {
            mik = CalcuMatrix[i-1][k];
            for(int j=k+1;j<=8;j++)
            {
                CalcuMatrix[i-1][j-1] -= mik*CalcuMatrix[k][j-1];
            }
        }
    }
    double InverseMatrix[4][4];
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            InverseMatrix[i][j] = CalcuMatrix[i][j+4];
        }
    }

    Eigen::Matrix4d Result;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            if(fabs(InverseMatrix[i][j]) < 0.0000001)
                InverseMatrix[i][j] = 0.00;
            Result(i,j) = InverseMatrix[i][j];
        }
    }
    return Result;
}



//void TestM()
//{
//    string Getdata;
//    int data =2;
//    Getdata = phone.callPyFun("GetD435QRcode", "int", data);
//    d435i_p = phone.StrToVct(phone.StrFirst(Getdata,0),3);
//    VectorXd dataQ=phone.StrToVct(phone.StrFirst(Getdata,1),4);
//    d435i_t = Quaternion2RotationMatrix(dataQ[0],dataQ[1],dataQ[2],dataQ[3]);

//    //    std::cout << 'dataT:'<<dataT<<endl;
//    //    std::cout << 'dataQ:'<<dataQ<<endl;

//}

bool TestMB()
{
    string Getdata;
    int data =2;
    for(int ica=0;ica<3;ica++){
        Getdata = phone.callPyFun("GetD435QRcode", "int", data);
        if(Getdata=="NULL") {
            cout<<"something wrong! try again!"<<endl;
            usleep(50000);
        }
        else{break;}
    }
    if(Getdata=="NULL") {return false;}

    d435i_p = phone.StrToVct(phone.StrFirst(Getdata,0),3);
    VectorXd dataQ=phone.StrToVct(phone.StrFirst(Getdata,1),4);
    d435i_t = Quaternion2RotationMatrix(dataQ[0],dataQ[1],dataQ[2],dataQ[3]);

    return true;
    //    std::cout << 'dataT:'<<dataT<<endl;
    //    std::cout << 'dataQ:'<<dataQ<<endl;

}

void TestM22()
{
    string Getdata;
    int data =22;
    Getdata = phone.callPyFun("GetD435QRcode", "int", data);
    d435i_p22 = phone.StrToVct(phone.StrFirst(Getdata,0),3);
    VectorXd dataQ=phone.StrToVct(phone.StrFirst(Getdata,1),4);
    d435i_t22 = Quaternion2RotationMatrix(dataQ[0],dataQ[1],dataQ[2],dataQ[3]);

    //    std::cout << 'dataT:'<<dataT<<endl;
    //    std::cout << 'dataQ:'<<dataQ<<endl;

}

void calTarToBase(int ids,Eigen::Vector3d pp,Eigen::Matrix3d tt)
{
    Eigen::Vector3d d435i_cp;
    Eigen::Matrix3d d435i_ct;


    Eigen::Matrix4d  TarToD435;
    Eigen::Matrix4d  D435ToOrigin;
    Eigen::Matrix4d  OriginToBase;

    Eigen::Matrix4d  TarToBase;

    Math::Vector3d DToBase_p;
    Math::Matrix3d DToBase_t_1;
    Math::Matrix3d DToBase_t;

    Math::Vector3d     EToBase_p;
    Math::Matrix3d     EToBase_t;

    arms[ids].m_dynamics.getEndPose(EToBase_p,EToBase_t);

    cout<<"EToBase_p:"<<EToBase_p<<endl;

    //caculate T1:TarToD435
    TarToD435 = PRtoT(pp,tt);
    cout<<"TarToD435:"<<TarToD435<<endl;

    //caculate T2:D435ToOrigin
    D435ToOrigin << 0,0,1,0,
            -1,0,0,0,
            0,-1,0,0,
            0, 0,0,1;
    //     cout<<"D435ToOrigin:"<<D435ToOrigin<<endl;


    //caculate T3:OriginToBase
    arms[ids].m_dynamics.getD435iPose(DToBase_p,DToBase_t_1);
    //    cout<<"DToBase_t_before:"<<DToBase_t_1<<endl;

    transpose(DToBase_t_1,DToBase_t);
    //     cout<<"DToBase_t_after:"<<DToBase_t<<endl;

    OriginToBase = PRtoT(DToBase_p,DToBase_t);

    cout<<"OriginToBase:"<<OriginToBase<<endl;

    //caculate T4:TargetToBase
    if(pp[0]!=0 && pp[1]!=0 && pp[2]!=0)
    {
        TarToBase = OriginToBase * D435ToOrigin *TarToD435;
        TtoPR(TarToBase,Tar_p,Tar_t);

        cout<<"Tar_p:"<<Tar_p<<endl;
    }
    else
    {
        Tar_p =  EToBase_p;
        Tar_t = EToBase_t;
    }

}

string to_String(int n)
{
    int m = n;
    char s[max];
    char ss[max];
    int i=0,j=0;
    if (n < 0)// 处理负数
    {
        m = 0 - m;
        j = 1;
        ss[0] = '-';
    }
    while (m>0)
    {
        s[i++] = m % 10 + '0';
        m /= 10;
    }
    s[i] = '\0';
    i = i - 1;
    while (i >= 0)
    {
        ss[j++] = s[i--];
    }
    ss[j] = '\0';
    return ss;
}

void gohome(int ids)
{
    //     arms[ids] = arms[ids];
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd jp1(arms[ids].m_Dofs+1),jp0(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs),tp6(arms[ids].m_Dofs);
        if(ids==0)
            if(ids==0)
            {
                jp1 << 35.0f * PI / 180.0f, 93.0f * PI / 180.0f, -175.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
                jp0 << 0.0f * PI / 180.0f, 60.0f * PI / 180.0f, -150.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
                arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);

                tp0<<pos[0],pos[1],pos[2],0,0,0;

                tp1<<pos[0],pos[1],pos[2]+0.1,0,0,0;
                tp2<<pos[0],pos[1],pos[2]-0.1,0,0,0;

                tp3<<pos[0]+0.1,pos[1],pos[2],0,0,0;
                tp4<<pos[0]-0.1,pos[1],pos[2],0,0,0;

                tp3<<pos[0],pos[1]-0.1,pos[2],0,0,0;
                tp4<<pos[0],pos[1]+0.1,pos[2],0,0,0;


            }

        if(ids==3)
        {
            jp1 <<  -35.0f * PI / 180.0f, 93.0f * PI / 180.0f, -175.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            jp0 << 0.0f * PI / 180.0f, 60.0f * PI / 180.0f, -150.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);

            tp0<<pos[0],pos[1],pos[2],0,0,0;

            tp1<<pos[0],pos[1],pos[2]+0.1,0,0,0;
            tp2<<pos[0],pos[1],pos[2]-0.1,0,0,0;

            tp3<<pos[0]+0.1,pos[1],pos[2],0,0,0;
            tp4<<pos[0]-0.1,pos[1],pos[2],0,0,0;

            tp5<<pos[0],pos[1]-0.1,pos[2],0,0,0;
            tp6<<pos[0],pos[1]+0.1,pos[2],0,0,0;
        }



        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp1,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}

void peg_initpos(int ids)
{
    //     arms[ids] = arms[ids];
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;

        //        float vel = 5.0f*gSpeed;
        //        float acc = 10.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd tp0(arms[ids].m_Dofs+1),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);

        tp0 << 0.0f * PI / 180.0f, 60.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 60.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp0,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}

void initpos(int ids)
{
    //     arms[ids] = arms[ids];
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd jp1(arms[ids].m_Dofs+1),jp0(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs),tp6(arms[ids].m_Dofs);
        if(ids==0)
            if(ids==0)
            {
                jp1 << 0.0f * PI / 180.0f, 60.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 60.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
                jp0 << 0.0f * PI / 180.0f, 60.0f * PI / 180.0f, -150.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
                arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);

                tp0<<pos[0],pos[1],pos[2],0,0,0;

                tp1<<pos[0],pos[1],pos[2]+0.1,0,0,0;
                tp2<<pos[0],pos[1],pos[2]-0.1,0,0,0;

                tp3<<pos[0]+0.1,pos[1],pos[2],0,0,0;
                tp4<<pos[0]-0.1,pos[1],pos[2],0,0,0;

                tp3<<pos[0],pos[1]-0.1,pos[2],0,0,0;
                tp4<<pos[0],pos[1]+0.1,pos[2],0,0,0;


            }

        if(ids==3)
        {
            jp1 << 0.0f * PI / 180.0f, 60.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 60.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            jp0 << 0.0f * PI / 180.0f, 60.0f * PI / 180.0f, -150.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);

            tp0<<pos[0],pos[1],pos[2],0,0,0;

            tp1<<pos[0],pos[1],pos[2]+0.1,0,0,0;
            tp2<<pos[0],pos[1],pos[2]-0.1,0,0,0;

            tp3<<pos[0]+0.1,pos[1],pos[2],0,0,0;
            tp4<<pos[0]-0.1,pos[1],pos[2],0,0,0;

            tp5<<pos[0],pos[1]-0.1,pos[2],0,0,0;
            tp6<<pos[0],pos[1]+0.1,pos[2],0,0,0;
        }



        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp1,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}


void actionconfig(int ids)
{
    //     arms[ids] = arms[ids];
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd jp1(arms[ids].m_Dofs+1),jp0(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs),tp6(arms[ids].m_Dofs);
        if(ids==0)
        {
            jp1 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            jp0 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);
            tp0<<pos[0],pos[1],pos[2],0,0,0;

            tp1 << pos[0],pos[1],pos[2],-45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp2 << pos[0],pos[1],pos[2],45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp3 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,-30.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp4 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,30.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp5 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,-45.0f * PI / 180.0f;
            tp6 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,45.0f * PI / 180.0f;
        }

        if(ids==3)
        {
            jp1 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            jp0 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);

            tp0<<pos[0],pos[1],pos[2],0,0,0;


            tp1 << pos[0],pos[1],pos[2],-45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp2 << pos[0],pos[1],pos[2],45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp3 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,-30.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp4 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,30.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp5 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,-45.0f * PI / 180.0f;
            tp6 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,45.0f * PI / 180.0f;
        }



        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp1,vel,acc);

        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);

        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp4,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);

        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp5,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp6,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);

        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}


void action1(int ids)
{
    //     arms[ids] = arms[ids];
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd jp1(arms[ids].m_Dofs+1),jp0(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs),tp6(arms[ids].m_Dofs);
        if(ids==0)
        {
            jp1 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            jp0 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);

            tp0<<pos[0],pos[1],pos[2],0,0,0;

            tp1<<pos[0],pos[1],pos[2],0,0,0;
            tp2<<pos[0],pos[1],pos[2]-0.1,0,0,0;

            tp3<<pos[0]+0.1,pos[1],pos[2],0,0,0;
            tp4<<pos[0]-0.1,pos[1],pos[2],0,0,0;

            tp5<<pos[0],pos[1]+0.1,pos[2],0,0,0;
            tp6<<pos[0],pos[1]-0.1,pos[2],0,0,0;
        }

        if(ids==3)
        {
            jp1 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            jp0 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);

            tp0<<pos[0],pos[1],pos[2],0.0,0.0f * PI / 180.0f,0;

            tp1<<pos[0],pos[1],pos[2],0,0,0;
            tp2<<pos[0],pos[1],pos[2]-0.1,0,0,0;

            tp3<<pos[0]+0.1,pos[1],pos[2],0,0,0;
            tp4<<pos[0]-0.1,pos[1],pos[2],0,0,0;

            tp5<<pos[0],pos[1]+0.1,pos[2],0,0,0;
            tp6<<pos[0],pos[1]-0.1,pos[2],0,0,0;
        }



        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp1,vel,acc);

        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/4,acc/4);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel/4,acc/4);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);

        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp4,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);

        if(ids==0)
        {

            arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp5,vel,acc);
            arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        }
        if(ids==3)
        {

            arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp6,vel,acc);
            arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        }
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}

void action3(int ids)
{
    //     arms[ids] = arms[ids];
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed/1.5;
        float acc = 2.0f*gSpeed/1.5;
        Math::Vector3d pos,rpy;
        VectorXd jp1(arms[ids].m_Dofs+1),jp0(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs),tp6(arms[ids].m_Dofs);
        if(ids==0)
        {
            jp1 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            jp0 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);

            tp0<<pos[0],pos[1],pos[2],0,0,0;

            tp1<<pos[0],pos[1],pos[2]-0.15,0,0,0;
            tp2<<pos[0]-0.15,pos[1],pos[2]-0.15,0,0,0;

            tp3<<pos[0]-0.15,pos[1],pos[2],0,0,0;
            tp4<<pos[0],pos[1],pos[2],0,0,0;

        }

        if(ids==3)
        {
            jp1 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            jp0 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);

            tp0<<pos[0],pos[1],pos[2],0,0,0;

            tp1<<pos[0],pos[1],pos[2]-0.15,0,0,0;
            tp2<<pos[0]-0.15,pos[1],pos[2]-0.15,0,0,0;

            tp3<<pos[0]-0.15,pos[1],pos[2],0,0,0;
            tp4<<pos[0],pos[1],pos[2],0,0,0;

        }



        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp1,vel,acc);

        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/4,acc/4);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp4,vel,acc);

        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/4,acc/4);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp4,vel,acc);


        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}


void action2(int ids)
{
    //     arms[ids] = arms[ids];
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd jp1(arms[ids].m_Dofs+1),jp0(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs),tp6(arms[ids].m_Dofs);
        if(ids==0)
        {
            jp1 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            jp0 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);
            tp0<<pos[0],pos[1],pos[2],0,0,0;

            tp1 << pos[0],pos[1],pos[2],-45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp2 << pos[0],pos[1],pos[2],45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp3 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,-20.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp4 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,20.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp5 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,-30.0f * PI / 180.0f;
            tp6 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,30.0f * PI / 180.0f;
        }

        if(ids==3)
        {
            jp1 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            jp0 << 0.0f * PI / 180.0f, 70.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            arms[ids].m_dynamics.forwardKinematics(jp0,pos,rpy);

            tp0<<pos[0],pos[1],pos[2],0,0,0;


            tp1 << pos[0],pos[1],pos[2],-45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp2 << pos[0],pos[1],pos[2],45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp3 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,-20.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp4 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,20.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp5 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,-30.0f * PI / 180.0f;
            tp6 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,30.0f * PI / 180.0f;
        }



        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp1,vel,acc);

        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);

        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp4,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);

        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp5,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp6,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);

        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}



void birdhead(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::BIRDHEAD);
        printf("test BIRDHEAD motion\n");
    }
}


void birdhead_rob(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::BIRDHEAD_Rod);
        printf("test BIRDHEAD_Rob motion\n");
    }
}

void birdhead_planing(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::BIRDHEAD_Planing);
        printf("test BIRDHEAD_Planing motion\n");
    }
}

void test6_s(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 60.0f*gSpeed;
        float acc = 12.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd jp(arms[ids].m_Dofs);
        VectorXd jp0(arms[ids].m_Dofs+1),jp1(arms[ids].m_Dofs+1);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs),tp12(arms[ids].m_Dofs);
        //        printf("arms[ids].m_Dofs:%d\n",arms[ids].m_Dofs);
        jp << 0,60.0f * PI / 180.0f,-90.0f* PI / 180.0f,0.0f * PI / 180.0f,30.0f * PI / 180.0f,0.0f* PI / 180.0f;
        jp0 << 0,60.0f * PI / 180.0f,-90.0f* PI / 180.0f,0.0f * PI / 180.0f,30.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;
        jp1 << 0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        //        arms[ids].m_dynamics.forwardKinematics(jp,pos,rpy);

        arms[ids].m_dynamics.forwardKinematics(jp,pos,rpy);
        printf("rpy:%f,%f,%f\n",rpy[0],rpy[1],rpy[2]);
        printf("jp1:%f,%f,%f\n",pos[0],pos[1],pos[2]);
        //Abs

        //        tp0 << 0.69985389709473, -1.4528632164001e-07, 0,0.52359884977341, 0.26179945468903, 0.26179951429367;
        //        tp1 << 0.69985389709473, -0.12500013411045, 0,0.8726646900177, 0.52359890937805, 0.52359879016876;


        tp0 <<  pos[0],pos[1],pos[2],0,0,0;

        tp1 <<  pos[0]+0.1,pos[1],pos[2],0,0,0;
        tp2 <<  pos[0]-0.1,pos[1],pos[2],0,0,0;

        tp3 <<  pos[0],pos[1]+0.1,pos[2],0,0,0;
        tp4 <<  pos[0],pos[1]-0.1,pos[2],0,0,0;


        tp5 <<  pos[0],pos[1],pos[2]+0.1,0,0,0;
        tp6 <<  pos[0],pos[1],pos[2]-0.1,0,0,0;


        tp7 << pos[0],pos[1],pos[2],-45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
        tp8 << pos[0],pos[1],pos[2],45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;

        tp9 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,-45.0f * PI / 180.0f,0.0f * PI / 180.0f;
        tp10 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,45.0f * PI / 180.0f,0.0f * PI / 180.0f;

        tp11 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,-45.0f * PI / 180.0f;
        tp12<< pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,45.0f * PI / 180.0f;


        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp0,3.0f*vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp4,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp5,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp6,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);

        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp7,4*vel,4*acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,4*vel,4*acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp8,4*vel,4*acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,4*vel,4*acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp9,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp10,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp11,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp12,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp1,3.0f*vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test6\n");
    }
}

void test6_p1(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed/10;
        float acc = 2.0f*gSpeed/10;
        Math::Vector3d pos,rpy;
        VectorXd jp0(3);

        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);
        arms[ids].m_dynamics.forwardKinematics(arms[ids].m_Desire.posd,pos,rpy);
        //Abs
        if(ids ==0)
        {
            tp0 <<  pos[0],pos[1],pos[2],0.0f * PI / 180.0f,60.0f * PI / 180.0f,0.0f * PI / 180.0f;

            tp1 <<  pos[0],pos[1]+0.02,pos[2]-0.15,0.0f * PI / 180.0f, 60.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp2 <<  pos[0],pos[1],pos[2]-0.15,0.0f * PI / 180.0f, 60.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp3 <<  pos[0]+0.1,pos[1],pos[2]+0.05,0.0f * PI / 180.0f, 45.0f * PI / 180.0f,0.0f * PI / 180.0f;
        }

        if(ids ==3)
        {
            tp0 <<  pos[0],pos[1],pos[2],0.0f * PI / 180.0f,60.0f * PI / 180.0f,0.0f * PI / 180.0f;

            tp1 <<  pos[0],pos[1]-0.02,pos[2]-0.15,0.0f * PI / 180.0f, 60.0f * PI / 180.0f,0.0f * PI / 180.0f;

            tp2 <<  pos[0],pos[1],pos[2]-0.15,0.0f * PI / 180.0f, 45.0f * PI / 180.0f,0.0f * PI / 180.0f;
            tp3 <<  pos[0]+0.1,pos[1],pos[2]+0.05,0.0f * PI / 180.0f, 45.0f * PI / 180.0f,0.0f * PI / 180.0f;
        }
        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel,acc);


        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test6\n");

    }
}


void test6_p2(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd jp0(3);

        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);
        arms[ids].m_dynamics.forwardKinematics(arms[ids].m_Desire.posd,pos,rpy);
        //            arms[ids].m_dynamics.getEndPosition(pos);


        //Abs



        tp0 <<  pos[0],pos[1],pos[2],0.0f * PI / 180.0f,90.0f * PI / 180.0f,0.0f * PI / 180.0f;


        tp1 <<  pos[0],pos[1],pos[2]+0.3,0.0f * PI / 180.0f,90.0f * PI / 180.0f,0.0f * PI / 180.0f;

        //         if(ids ==0)
        //         {
        //             arms[ids].clearRmlPoints();
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/8,acc/2);
        //             arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        //         }
        //         if(ids ==3)
        //         {
        //             arms[ids].clearRmlPoints();
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/4,acc/3);
        //             arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        //         }

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/8,acc/4);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test6\n");
    }
}


void test6_p3(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd jp0(3);

        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);

        //        jp0 << (arms[ids].m_canComm ->encoders[0]->pos_init)* PI / 180.0f,(arms[ids].m_canComm ->encoders[1]->pos_init)* PI / 180.0f,(arms[ids].m_canComm ->encoders[2]->pos_init)* PI / 180.0f;     //init position;
        //        jp0<<arms[ids].m_Real.posr[0],arms[ids].m_Real.posr[1],arms[ids].m_Real.posr[2];
        //        arms[ids].m_dynamics.setJointPositions(arms[ids].m_Desire.posd);

        //        cout <<"jp0:"<<arms[ids].m_Desire.posd<<endl;
        //        arms[ids].m_dynamics.getEndPosition(pos);
        arms[ids].m_dynamics.forwardKinematics(arms[ids].m_Desire.posd,pos,rpy);
        //Abs
        tp0 <<  pos[0],pos[1],pos[2],0.0f * PI / 180.0f,90.0f * PI / 180.0f,0.0f * PI / 180.0f;


        tp1 <<  pos[0],pos[1],pos[2]-0.15,0.0f * PI / 180.0f,90.0f * PI / 180.0f,0.0f * PI / 180.0f;

        tp2 <<  pos[0]-0.05,pos[1],pos[2]-0.15,0.0f * PI / 180.0f,90.0f * PI / 180.0f,0.0f * PI / 180.0f;
        tp3 <<  pos[0]-0.05,pos[1],pos[2],0.0f * PI / 180.0f,90.0f * PI / 180.0f,0.0f * PI / 180.0f;

        //         if(ids == 0)
        //         {
        //             arms[ids].clearRmlPoints();
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/4,acc/2);
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel/2,acc/2);
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel/2,acc/2);
        //             arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        //         }
        //         if(ids == 3)
        //         {
        //             arms[ids].clearRmlPoints();
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/2,acc/2);
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel/2,acc/2);
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel/2,acc/2);
        //             arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        //             printf("test6\n");
        //         }
        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/2,acc/2);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel/2,acc/2);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel/8,acc/4);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test6\n");

    }
}

void test6_p4(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd jp0(3);

        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);

        //        jp0 << (arms[ids].m_canComm ->encoders[0]->pos_init)* PI / 180.0f,(arms[ids].m_canComm ->encoders[1]->pos_init)* PI / 180.0f,(arms[ids].m_canComm ->encoders[2]->pos_init)* PI / 180.0f;     //init position;
        //        jp0<<arms[ids].m_Real.posr[0],arms[ids].m_Real.posr[1],arms[ids].m_Real.posr[2];
        //        arms[ids].m_dynamics.setJointPositions(arms[ids].m_Desire.posd);

        //        cout <<"jp0:"<<arms[ids].m_Desire.posd<<endl;
        //        arms[ids].m_dynamics.getEndPosition(pos);
        arms[ids].m_dynamics.forwardKinematics(arms[ids].m_Desire.posd,pos,rpy);
        //Abs
        tp0 <<  pos[0],pos[1],pos[2],0.0f * PI / 180.0f,90.0f * PI / 180.0f,0.0f * PI / 180.0f;


        tp1 <<  pos[0],pos[1],pos[2]-0.15,0.0f * PI / 180.0f,90.0f * PI / 180.0f,0.0f * PI / 180.0f;

        tp2 <<  pos[0]-0.05,pos[1],pos[2]-0.15,0.0f * PI / 180.0f,90.0f * PI / 180.0f,0.0f * PI / 180.0f;
        tp3 <<  pos[0],pos[1],pos[2]+0.15,0.0f * PI / 180.0f,90.0f * PI / 180.0f,0.0f * PI / 180.0f;

        //         if(ids == 0)
        //         {
        //             arms[ids].clearRmlPoints();
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/4,acc/2);
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel/2,acc/2);
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel/2,acc/2);
        //             arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        //         }
        //         if(ids == 3)
        //         {
        //             arms[ids].clearRmlPoints();
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/2,acc/2);
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel/2,acc/2);
        //             arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel/2,acc/2);
        //             arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        //             printf("test6\n");
        //         }
        arms[ids].clearRmlPoints();
        //         arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/2,acc/2);
        //         arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel/2,acc/2);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp3,vel/8,acc/4);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test6\n");

    }
}


void supporting(int ids)
{
    //     arms[ids] = arms[ids];
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;

        //        float vel = 5.0f*gSpeed;
        //        float acc = 10.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd tp0(arms[ids].m_Dofs+1),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
        if(ids == 0)
        {
            tp0 <<  16.0f * PI / 180.0f, 115.0f * PI / 180.0f, -115.0f* PI / 180.0f, 0.0 * PI / 180.0f, 88.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

        }
        if(ids == 3)
        {
            tp0 <<  -16.0f * PI / 180.0f, 115.0f * PI / 180.0f, -116.0f* PI / 180.0f, 0.0 * PI / 180.0f, 88.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

        }


        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp0,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}

void supporting_test(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        Eigen::VectorXd tp1(7);
        VectorXd tp00(6),tp0(6), jp1(6),jp(6);

        arms[ids].m_dynamics.getJointPositions(jp);
        //        jp1<<jp[0],jp[1],jp[2],jp[3],1.57,jp[5];
        arms[ids].m_dynamics.forwardKinematics(jp,pos,rpy);
        //cout<<"rpy:"<<rpy<<endl;

        tp00 << pos[0]-0.005,pos[1],pos[2]+0.025, -90.0f * PI / 180.0f, -80.0f * PI / 180.0f, -90.0f * PI / 180.0f;

        if(ids == 0)
        {
            tp0 << pos[0],pos[1],pos[2]-0.075, -90.0f * PI / 180.0f, -80.0f * PI / 180.0f, -90.0f * PI / 180.0f;//-17.1f * PI / 180.0f,  -47.7f * PI / 180.0f, -22.6f * PI / 180.0f;// 70.6f * PI / 180.0f,  -79.4f * PI / 180.0f, 70.9f * PI / 180.0f;//0,  -80.0f * PI / 180.0f, 0.0f * PI / 180.0f;//

        }
        if(ids == 3)
        {
            tp0 << pos[0],pos[1],pos[2]-0.075, -90.0f * PI / 180.0f, -80.0f * PI / 180.0f, -90.0f * PI / 180.0f;//-17.1f * PI / 180.0f,  -47.7f * PI / 180.0f, -22.6f * PI / 180.0f;//70.6f * PI / 180.0f,  -79.4f * PI / 180.0f, 70.9f * PI / 180.0f;//0,  -80.0f * PI / 180.0f, 0.0f * PI / 180.0f;//

        }

        //        if(ids == 0)
        //        {
        //            tp0 << pos[0]-0.03,pos[1]-0.05,pos[2]-0.05, 0.0f * PI / 180.0f,  -0.0f * PI / 180.0f, 0.0f * PI / 180.0f;//0,  -80.0f * PI / 180.0f, 0.0f * PI / 180.0f;//

        //        }
        //        if(ids == 3)
        //        {
        //            tp0 << pos[0]-0.03,pos[1]+0.05,pos[2]-0.05, 0.0f * PI / 180.0f,  -0.0f * PI / 180.0f, 0.0f * PI / 180.0f;//0,  -80.0f * PI / 180.0f, 0.0f * PI / 180.0f;//

        //        }
        //tp0 << pos[0]-0.03,pos[1],pos[2]-0.05, 70.6f * PI / 180.0f,  -79.4f * PI / 180.0f, 70.9f * PI / 180.0f;//0,  -80.0f * PI / 180.0f, 0.0f * PI / 180.0f;//
        tp1 <<  0.0f * PI / 180.0f, 60.0f * PI / 180.0f, -150.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp00,vel/4,acc/4);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel/4,acc/4);
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp1,vel/2,acc/2);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        rt_printf("leave the  gblance mode\n");
    }
}

void supporting_init(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd tp0(arms[ids].m_Dofs+1),tp1(arms[ids].m_Dofs+1),tp2(arms[ids].m_Dofs),tp(arms[ids].m_Dofs+1),tpp(arms[ids].m_Dofs+1);

        if(ids==0)
        {
            tp0 << 10.0f * PI / 180.0f, 98.0f * PI / 180.0f, -152.0f* PI / 180.0f, 0.0 * PI / 180.0f, 88.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            //            jp0 << 3.1f * PI / 180.0f, 102.3f * PI / 180.0f, -133.0f* PI / 180.0f, 0.0 * PI / 180.0f, 88.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
        }
        if(ids==3)
        {
            tp0 << -10.0f * PI / 180.0f, 98.0f * PI / 180.0f, -152.0f* PI / 180.0f, 0.0 * PI / 180.0f, 88.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
        }
        //        tp1 <<  0.0f * PI / 180.0f, 84.0f * PI / 180.0f, -89.0f* PI / 180.0f, 0.0 * PI / 180.0f, 88.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp0,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}

void banyunINI(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        VectorXd tp0(arms[ids].m_Dofs+1);
        if(ids==0)
        {
            tp0 <<  8.0f * PI / 180.0f, 60.0f * PI / 180.0f, -120.0f* PI / 180.0f, 0.0 * PI / 180.0f, -30.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
        }
        if(ids==3)
        {
            //            tp0 <<  -8.0f * PI / 180.0f, 34.0f * PI / 180.0f, -115.0f* PI / 180.0f, 0.0 * PI / 180.0f, -0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

            tp0 <<  -8.0f * PI / 180.0f, 60.0f * PI / 180.0f, -120.1f* PI / 180.0f, 0.0 * PI / 180.0f, -30.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
        }
        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp0,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
    }
}

void xuanguaINI(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd tp0(arms[ids].m_Dofs+1),tp1(arms[ids].m_Dofs+1),tp2(arms[ids].m_Dofs),tp(arms[ids].m_Dofs+1),tpp(arms[ids].m_Dofs+1);
        if(ids==0)
        {
            tp0 <<  8.5f * PI / 180.0f, 108.0f * PI / 180.0f, -151.0f* PI / 180.0f, 0.0 * PI / 180.0f, 88.0f * PI / 180.0f, 90.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position


        }
        if(ids==3)
        {
            tp0 <<  -8.5f * PI / 180.0f, 108.0f * PI / 180.0f, -151.0f* PI / 180.0f, 0.0 * PI / 180.0f, 88.0f * PI / 180.0f, 90.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

        }//

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp0,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}

void xiaokongVidio(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd tp0(arms[ids].m_Dofs+1),tp1(arms[ids].m_Dofs+1),tp2(arms[ids].m_Dofs),tp(arms[ids].m_Dofs+1),tpp(arms[ids].m_Dofs+1);
        if(ids==0)
        {
            tp0 <<  5.6f * PI / 180.0f, 117.0f * PI / 180.0f, -110.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

        }
        if(ids==3)
        {
            tp0 <<  -5.6f * PI / 180.0f, 117.0f * PI / 180.0f, -110.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 90.0f * PI / 180.0f, -50.0f * PI / 180.0f;     //work position

        }//

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp0,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}




void xiaokongINI(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd tp0(arms[ids].m_Dofs+1),tp1(arms[ids].m_Dofs+1),tp2(arms[ids].m_Dofs),tp(arms[ids].m_Dofs+1),tpp(arms[ids].m_Dofs+1);
        if(ids==0)
        {
            tp0 <<  7.0f * PI / 180.0f, 100.0f * PI / 180.0f, -142.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
            vel = vel/4.0;
            acc = acc/4.0;

        }
        if(ids==3)
        {
            tp0 <<  -7.0f * PI / 180.0f, 100.0f * PI / 180.0f, -142.0f* PI / 180.0f, 0.0 * PI / 180.0f, 50.0f * PI / 180.0f, 90.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

        }//

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp0,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}

void HanJieINI(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd tp0(arms[ids].m_Dofs+1),tp1(arms[ids].m_Dofs+1),tp2(arms[ids].m_Dofs),tp(arms[ids].m_Dofs+1),tpp(arms[ids].m_Dofs+1);
        if(ids==0)
        {
            tp0 <<  -10.0f * PI / 180.0f, 60.0f * PI / 180.0f, -105.0f* PI / 180.0f, 90.0f * PI / 180.0f, 0.0f * PI / 180.0f, 45.0f * PI / 180.0f, -50.0f * PI / 180.0f;     //work position

        }
        if(ids==3)
        {
            tp0 <<  -2.6f * PI / 180.0f, 62.0f * PI / 180.0f, -111.0f* PI / 180.0f, 0.0f * PI / 180.0f, -45.0f * PI / 180.0f, 90.0f * PI / 180.0f, -50.0f * PI / 180.0f;     //work position

        }//

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp0,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        //printf("test1\n");
    }
}
void HanJieBack(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd tp0(arms[ids].m_Dofs+1),tp1(arms[ids].m_Dofs+1),tp2(arms[ids].m_Dofs),tp(arms[ids].m_Dofs+1),tpp(arms[ids].m_Dofs+1);
        if(ids==0)
        {
            tp0 <<  -10.0f * PI / 180.0f, 60.0f * PI / 180.0f, -105.0f* PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, -50.0f * PI / 180.0f;     //work position

        }
        if(ids==3)
        {
            tp0 <<  -10.0f * PI / 180.0f, 60.0f * PI / 180.0f, -105.0f* PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, -50.0f * PI / 180.0f;     //work position

        }//

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp0,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        //printf("test1\n");
    }
}

void xianlanINI(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd tp0(arms[ids].m_Dofs+1),tp1(arms[ids].m_Dofs+1),tp2(arms[ids].m_Dofs),tp(arms[ids].m_Dofs+1),tpp(arms[ids].m_Dofs+1);
        if(ids==0)
        {
            tp0 <<  -10.0f * PI / 180.0f, 50.0f * PI / 180.0f, -95.0f* PI / 180.0f, 0.0 * PI / 180.0f, -45.0f * PI / 180.0f, 0.0f * PI / 180.0f, -50.0f * PI / 180.0f;     //work position

        }
        if(ids==3)
        {
            tp0 <<  -10.0f * PI / 180.0f, 50.0f * PI / 180.0f, -95.0f* PI / 180.0f, 0.0 * PI / 180.0f, -45.0f * PI / 180.0f, 0.0f * PI / 180.0f, -50.0f * PI / 180.0f;     //work position

        }//

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,tp0,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::JOINT_RML,tp,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("test1\n");
    }
}

void xianlan_d435(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        Math::Matrix3d mat;
        VectorXd jp0(arms[ids].m_Dofs+1),jp1(arms[ids].m_Dofs+1),jp(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);

        tp0 << Tar_p[0], Tar_p[1]+0.05,Tar_p[2]-0.02, 0.0f * PI / 180.0f,  90.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        //        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

    }
}

void g_blance(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::G_Blance);
        printf("test G_Blance motion\n");
    }
}

void support_force(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::SUPPORT);
        printf("test G_Blance motion\n");
    }
}


void g_blance_xuangua(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::G_Blance_Xuangua);

        printf("test G_Blance motion\n");
    }
}

void supporting_d435(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        Math::Matrix3d mat;
        VectorXd jp0(arms[ids].m_Dofs+1),jp1(arms[ids].m_Dofs+1),jp(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);
        //        printf("arm.m_Dofs:%d\n",arms[ids].m_Dofs);
        //        jp << 0.0f * PI / 180.0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f;     //work position;
        //        jp0 << 0.0f * PI / 180.0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;


        jp << 0.0f * PI / 180.0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,30.0f * PI / 180.0f,0.0f* PI / 180.0f;     //work position;
        jp0 << -80.0f * PI / 180.0,60.0f * PI / 180.0f,-150.0f* PI / 180.0f,0.0f * PI / 180.0f,45.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        jp1 << 0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        arms[ids].m_dynamics.forwardKinematics(jp,pos,rpy);
        arms[ids].m_dynamics.forwardKinematics(jp,pos,mat);
        //      arm.m_dynamics.forwardKinematics(jp,pos,rpy);
        printf("rpy:%f,%f,%f\n",rpy[0],rpy[1],rpy[2]);
        printf("jp1:%f,%f,%f\n",pos[0],pos[1],pos[2]);
        cout <<"mat:"<<mat<<endl;

        //Abs
        //tp0 << Tar_p[0], Tar_p[1], Tar_p[2], 0,  -90.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        tp1 << 0.49170416593552, -0.11686009168625, 0.15303391218185, 0.52359890937805, 1.0471975803375, 1.0471979379654;

        //         tp0 <<  pos[0],pos[1],pos[2],0,0,0;

        //         tp1 <<  pos[0]-0.1,pos[1],pos[2],0,0,0;

        tp2 <<  pos[0],pos[1]+0.1,pos[2],0,0,0;

        tp3 <<  pos[0],pos[1],pos[2]-0.1,0,0,0;


        tp4 << pos[0],pos[1],pos[2],-45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
        tp5 << pos[0],pos[1],pos[2],45.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f;
        tp6 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,-45.0f * PI / 180.0f,0.0f * PI / 180.0f;
        tp7 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,45.0f * PI / 180.0f,0.0f * PI / 180.0f;
        tp8 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,-45.0f * PI / 180.0f;
        tp9 << pos[0],pos[1],pos[2],0.0f * PI / 180.0f,0.0f * PI / 180.0f,45.0f * PI / 180.0f;


        arms[ids].clearRmlPoints();
        //          arm.addRmlPoint(MOTION_TYPE::JOINT_RML,jp0,3.0f*vel,acc);
        //        arm.addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
        if(ids==0)
        {
            tp0 << Tar_p[0], -Tar_p[1], Tar_p[2], 0,  -90.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        }
        else
        {
            tp0 << Tar_p[0], Tar_p[1], Tar_p[2], 0, -90.0f * PI / 180.0f, 0.0f * PI / 180.0f;

        }
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel/2,acc/2);

        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
    }
}


void ing_d435_2(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        Math::Matrix3d mat;
        VectorXd jp0(arms[ids].m_Dofs+1),jp1(arms[ids].m_Dofs+1),jp(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);
        //        printf("arm.m_Dofs:%d\n",arms[ids].m_Dofs);
        //        jp << 0.0f * PI / 180.0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f;     //work position;
        //        jp0 << 0.0f * PI / 180.0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        arms[ids].m_dynamics.getJointPositions(jp);

        jp0 << jp[0],jp[1],jp[2],jp[3],1.57,jp[5],0;     //work position;

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp0,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
    }
}

void ing_forword(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        Math::Matrix3d mat;
        VectorXd jp0(arms[ids].m_Dofs+1),jp1(arms[ids].m_Dofs+1),jp(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);
        //        printf("arm.m_Dofs:%d\n",arms[ids].m_Dofs);
        //        jp << 0.0f * PI / 180.0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f;     //work position;
        //        jp0 << 0.0f * PI / 180.0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        arms[ids].m_dynamics.getJointPositions(jp);
        if(ids==0)
        {

            jp0 << 3.1f * PI / 180.0f, 102.3f * PI / 180.0f, -133.0f* PI / 180.0f, 0.0 * PI / 180.0f, 88.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position


        }
        if(ids==3)
        {

            jp0 << -3.1f * PI / 180.0f, 102.3f * PI / 180.0f, -133.0f* PI / 180.0f, 0.0 * PI / 180.0f, 88.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

        }
        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp0,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
    }
}

void banyun_d435_1(int ids)
{
    if( arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        VectorXd tp0( arms[ids].m_Dofs),tp1( arms[ids].m_Dofs),tp2( arms[ids].m_Dofs);
        //        cout<<""<<endl;
        if(ids ==0)
        {
            tp0 << Tar_p[0]-0.14,  (0.575-(-Tar_p[1])-0.317)-0.02, Tar_p[2]+0.03, 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;
            tp1 << Tar_p[0]-0.14, (0.575-(-Tar_p[1])-0.317)-0.02, Tar_p[2]-0.01, 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;
        }

        if(ids==3)
        {


            tp0 << Tar_p[0]-0.13, (Tar_p[1]+0.02), Tar_p[2]+0.03, 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;
            tp1 << Tar_p[0]-0.13, Tar_p[1]+0.02, Tar_p[2]-0.02, 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;


            //                        tp0 << Tar_p[0], (Tar_p[1]), Tar_p[2], 0.0f * PI / 180.0f,  90.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            //                        tp1 << Tar_p[0], Tar_p[1], Tar_p[2], 0.0f * PI / 180.0f,  90.0f * PI / 180.0f, 0.0f * PI / 180.0f;

        }


        arms[ids].clearRmlPoints();

        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel/2,acc/2);
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/2,acc/2);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

    }
}

void banyun_d435_2(int ids)
{
    if( arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed/16;
        float acc = 2.0f*gSpeed/16;
        VectorXd tp0( arms[ids].m_Dofs),tp1( arms[ids].m_Dofs),tp2( arms[ids].m_Dofs);
        if(ids ==0)
        {
            tp2 << Tar_p[0],-Tar_p[1], Tar_p[2]+0.32, 0,  90.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        }
        if(ids==3)
        {
            tp2 << Tar_p[0], Tar_p[1], Tar_p[2]+0.30, 0,  90.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        }

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);

        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

    }
}


void supporting_back(int ids)
{
    cout<<"00000:"<<arms[ids].m_SM.state<<endl;
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        cout<<"********b"<<endl;
        float vel = 15.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        VectorXd jp(arms[ids].m_Dofs), jp1(arms[ids].m_Dofs),tp0(arms[ids].m_Dofs),jp0(arms[ids].m_Dofs+1);

        arms[ids].m_dynamics.getJointPositions(jp);
        jp1<<jp[0],jp[1],jp[2],jp[3],1.57,jp[5];
        arms[ids].m_dynamics.forwardKinematics(jp1,pos,rpy);
        cout<<"11111"<<endl;

        tp0 << pos[0],pos[1],pos[2]-0.03, 0,  -80.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        jp0 <<  0.0f * PI / 180.0f, 60.0f * PI / 180.0f, -150.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position

        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        cout<<"22222"<<endl;
        arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp0,vel,acc);
        cout<<"33333"<<endl;
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
        printf("supporting_back\n");
    }
}


void  banyunBack(int ids)
{
    float vel = 5.0f*gSpeed/8;
    float acc = 2.0f*gSpeed/8;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    tp0 << pos[0], pos[1], pos[2]-0.1, 0,  90.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    tp1 << pos[0]-0.05, pos[1], pos[2]-0.1, 0,  90.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    tp2 << pos[0]-0.05, pos[1], pos[2], 0,  90.0f * PI / 180.0f, 0.0f * PI / 180.0f;

    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp2,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

}


void  banyunUp(int ids)
{
    float vel = 5.0f*gSpeed/8;
    float acc = 2.0f*gSpeed/8;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    tp0 << pos[0], pos[1], pos[2]+0.15, 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;

    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

}

void  banyunLeft(int ids)
{
    float vel = 5.0f*gSpeed/8;
    float acc = 2.0f*gSpeed/8;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    tp0 << pos[0], pos[1]+0.02, pos[2], 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;

    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

}


void  xuanguaup(int ids)
{
    float vel = 5.0f*gSpeed/8;
    float acc = 2.0f*gSpeed/8;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    tp0 << pos[0], pos[1], pos[2]+0.02, 0.0f * PI / 180.0f,  -45.0f * PI / 180.0f, 0.0f * PI / 180.0f;

    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

}


void  up(int ids)
{
    float vel = 5.0f*gSpeed/8;
    float acc = 2.0f*gSpeed/8;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    //模式提示
    if (m_task_state==0) //辅助搬运
    {
        //        cout<<"辅助搬运"<<endl;
        tp0 << pos[0], pos[1], pos[2]+0.02, 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;
    }
    else if(m_task_state==1) //过顶支撑
    {
        tp0 << pos[0], pos[1], pos[2]+0.02, 0.0f * PI / 180.0f,  0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==2) //销孔装配
    {
        tp0 << pos[0], pos[1], pos[2]+0.02, 0.0f * PI / 180.0f,  -8.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==3) //悬挂作业
    {
        //        cout<<"悬挂作业为"<<endl;
        tp0 << pos[0], pos[1], pos[2]+0.02, 0.0f * PI / 180.0f,  -45.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==4) //焊接开关
    {
        tp0 << pos[0], pos[1], pos[2]+0.02, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }


    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
}

void  down(int ids)
{
    float vel = 5.0f*gSpeed/8;
    float acc = 2.0f*gSpeed/8;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    //模式提示
    if (m_task_state==0) //辅助搬运
    {
        //        cout<<"辅助搬运"<<endl;
        tp0 << pos[0], pos[1], pos[2]-0.02, 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;
    }
    else if(m_task_state==1) //过顶支撑
    {
        tp0 << pos[0], pos[1], pos[2]-0.02, 0.0f * PI / 180.0f,  0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==2) //销孔装配
    {
        tp0 << pos[0], pos[1], pos[2]-0.02, 0.0f * PI / 180.0f,  -8.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==3) //悬挂作业
    {
        //        cout<<"悬挂作业为"<<endl;
        tp0 << pos[0], pos[1], pos[2]-0.02, 0.0f * PI / 180.0f,  -45.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==4) //焊接开关
    {
        tp0 << pos[0], pos[1], pos[2]-0.02, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }


    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

}

void  left(int ids)
{
    float vel = 5.0f*gSpeed/8;
    float acc = 2.0f*gSpeed/8;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    //模式提示
    if (m_task_state==0) //辅助搬运
    {
        //        cout<<"辅助搬运"<<endl;
        tp0 << pos[0], pos[1]+0.02, pos[2], 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;
    }
    else if(m_task_state==1) //过顶支撑
    {
        tp0 << pos[0], pos[1]+0.02, pos[2], 0.0f * PI / 180.0f,  0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==2) //销孔装配
    {
        tp0 << pos[0], pos[1]+0.02, pos[2], 0.0f * PI / 180.0f,  -8.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==3) //悬挂作业
    {
        //        cout<<"悬挂作业为"<<endl;
        tp0 << pos[0], pos[1]+0.02, pos[2], 0.0f * PI / 180.0f,  -45.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==4) //焊接开关
    {
        tp0 << pos[0], pos[1]+0.02, pos[2], 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
}

void right(int ids)
{
    float vel = 5.0f*gSpeed/8;
    float acc = 2.0f*gSpeed/8;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    //模式提示
    if (m_task_state==0) //辅助搬运
    {
        //        cout<<"辅助搬运"<<endl;
        tp0 << pos[0], pos[1]-0.02, pos[2], 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;
    }
    else if(m_task_state==1) //过顶支撑
    {
        tp0 << pos[0], pos[1]-0.02, pos[2], 0.0f * PI / 180.0f,  0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==2) //销孔装配
    {
        tp0 << pos[0], pos[1]-0.02, pos[2], 0.0f * PI / 180.0f,  -8.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==3) //悬挂作业
    {
        //        cout<<"悬挂作业为"<<endl;
        tp0 << pos[0], pos[1]-0.02, pos[2], 0.0f * PI / 180.0f,  -45.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==4) //焊接开关
    {
        tp0 << pos[0], pos[1]-0.02, pos[2], 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
}

void  forward(int ids)
{
    float vel = 5.0f*gSpeed/8;
    float acc = 2.0f*gSpeed/8;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    //模式提示
    if (m_task_state==0) //辅助搬运
    {
        //        cout<<"辅助搬运"<<endl;
        tp0 << pos[0]+0.02, pos[1], pos[2], 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0f;
    }
    else if(m_task_state==1) //过顶支撑
    {
        tp0 << pos[0]+0.02, pos[1], pos[2], 0.0f * PI / 180.0f,  0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==2) //销孔装配
    {
        tp0 << pos[0]+0.08, pos[1], pos[2], 0.0f * PI / 180.0f,  -8.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==3) //悬挂作业
    {
        //        cout<<"悬挂作业为"<<endl;
        tp0 << pos[0]+0.02, pos[1], pos[2], 0.0f * PI / 180.0f,  -45.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    else if(m_task_state==4) //焊接开关
    {
        tp0 << pos[0]+0.02, pos[1], pos[2], 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }
    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
}

void  forward_xiaokong(int ids)
{
    float vel = 5.0f*gSpeed/4;
    float acc = 2.0f*gSpeed/4;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    if(ids ==0)
    {
        tp0 << pos[0]+0.05, pos[1], pos[2], 0.0f * PI / 180.0f,  -13.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }

    if(ids ==3)
    {
        tp0 << pos[0]+0.05, pos[1], pos[2], 0.0f * PI / 180.0f,  -5.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }

    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel/18,acc/18);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
}


void  xiaokongback(int ids)
{
    float vel = 5.0f*gSpeed;
    float acc = 2.0f*gSpeed;
    Math::Vector3d pos;
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getEndPosition(pos);
    cout <<"pos"<< pos <<endl;
    if(ids==0)
    {
        tp0 << pos[0]-0.05, pos[1], pos[2], 0.0f * PI / 180.0f,  0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        tp1 << pos[0]-0.05, pos[1]+0.08, pos[2], 0.0f * PI / 180.0f,  0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }

    if(ids==3)
    {
        tp0 << pos[0]-0.05, pos[1], pos[2], 0.0f * PI / 180.0f,  0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        tp1 << pos[0]-0.05, pos[1]-0.03, pos[2], 0.0f * PI / 180.0f,  0.0f * PI / 180.0f, 0.0f * PI / 180.0f;
    }


    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
    arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

}


void Disable485(int ids)
{
    if(ids ==0)
    {
        for(int i=0;i<4;i++)
        {
            arms[ids].m_485Comm.dynamixels[i]->enableDynamixel(0);
        }
    }
    if(ids ==3)
    {
        for(int i=0;i<4;i++)
        {
            arms[ids].m_485Comm.dynamixels[i]->enableDynamixel(0);
        }
    }

}

void Enable485(int ids)
{

    if(ids ==0)
    {
        for(int i=0;i<4;i++)
        {
            arms[0].m_485Comm.dynamixels[i]->enableDynamixel(1);
            //            arms[0].m_485Comm.dynamixels[i]->getPos();
        }
    }
    if(ids ==3)
    {
        for(int i=0;i<4;i++)
        {
            arms[3].m_485Comm.dynamixels[i]->enableDynamixel(1);

            //            arms[3].m_485Comm.dynamixels[i]->getPos();
        }
    }

}

void shouzhua_open()
{
    if(ids ==0)
    {

        arms[0].m_485Comm.dynamixels[3]->setPos(0);

    }
    if(ids ==3)
    {

        arms[3].m_485Comm.dynamixels[3]->setPos(0);
    }
}


void shouzhua_close()
{

    if(ids ==0)
    {

        arms[0].m_485Comm.dynamixels[3]->setPos(48);

    }
    if(ids ==3)
    {

        arms[3].m_485Comm.dynamixels[3]->setPos(48);
    }
}

void shouzhua_open_tor(int ids)
{
    if(ids ==0)
    {
        for(int i=0;i<50;i++)
        {
            arms[0].m_485Comm.dynamixels[3]->openGripper(-14);
        }

    }
    if(ids ==3)
    {

        for(int i=0;i<50;i++)
        {
            arms[3].m_485Comm.dynamixels[3]->openGripper(-16);
        }

    }
}

void shouzhua_open_tor_sup(int ids)
{
    if(ids ==0)
    {
        for(int i=0;i<50;i++)
        {
            arms[0].m_485Comm.dynamixels[3]->openGripper(-22);
        }

    }
    if(ids ==3)
    {

        for(int i=0;i<50;i++)
        {
            arms[3].m_485Comm.dynamixels[3]->openGripper(-24);
        }

    }
}


void shouzhua_close_tor(int ids)
{
    if(ids ==0)
    {
        for(int i=0;i<50;i++)
        {
            arms[0].m_485Comm.dynamixels[3]->closeGripper(90);
        }

    }
    if(ids ==3)
    {

        for(int i=0;i<50;i++)
        {
            arms[3].m_485Comm.dynamixels[3]->closeGripper(90);
        }

    }
}


void shouzhua_stop_tor(int ids)
{
    if(ids ==0)
    {
        for(int i=0;i<50;i++)
        {
            arms[0].m_485Comm.dynamixels[3]->stopGripper();
        }

    }
    if(ids ==3)
    {

        for(int i=0;i<50;i++)
        {
            arms[3].m_485Comm.dynamixels[3]->stopGripper();
        }

    }
}

void xuangua_d435(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        Math::Matrix3d mat;
        VectorXd jp0(arms[ids].m_Dofs+1),jp1(arms[ids].m_Dofs+1),jp(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);

        jp << 0.0f * PI / 180.0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,30.0f * PI / 180.0f,0.0f* PI / 180.0f;     //work position;
        //        jp0 << -80.0f * PI / 180.0,60.0f * PI / 180.0f,-150.0f* PI / 180.0f,0.0f * PI / 180.0f,45.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        jp1 << 0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        arms[ids].m_dynamics.forwardKinematics(jp,pos,rpy);
        arms[ids].m_dynamics.forwardKinematics(jp,pos,mat);
        //      arm.m_dynamics.forwardKinematics(jp,pos,rpy);
        printf("rpy:%f,%f,%f\n",rpy[0],rpy[1],rpy[2]);
        printf("jp1:%f,%f,%f\n",pos[0],pos[1],pos[2]);
        cout <<"mat:"<<mat<<endl;
        if(ids==0)
        {
            tp0 << Tar_p[0]+0.04, -(Tar_p[1]-0.07), Tar_p[2]+0.04,  0.0f * PI / 180.0f,  -45.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        }
        if(ids==3)
        {
            tp0 << Tar_p[0]+0.04, Tar_p[1]-0.07, Tar_p[2]+0.04,  0.0f * PI / 180.0f,  -45.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        }
        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

    }
}

void xiaokong_d435(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        Math::Matrix3d mat;
        VectorXd jp0(arms[ids].m_Dofs+1),jp1(arms[ids].m_Dofs+1),jp(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);

        jp << 0.0f * PI / 180.0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,30.0f * PI / 180.0f,0.0f* PI / 180.0f;     //work position;
        //        jp0 << -80.0f * PI / 180.0,60.0f * PI / 180.0f,-150.0f* PI / 180.0f,0.0f * PI / 180.0f,45.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        jp1 << 0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        arms[ids].m_dynamics.forwardKinematics(jp,pos,rpy);
        arms[ids].m_dynamics.forwardKinematics(jp,pos,mat);
        //      arm.m_dynamics.forwardKinematics(jp,pos,rpy);
        printf("rpy:%f,%f,%f\n",rpy[0],rpy[1],rpy[2]);
        printf("jp1:%f,%f,%f\n",pos[0],pos[1],pos[2]);
        cout <<"mat:"<<mat<<endl;
        if(ids==0)
        {
            cout <<"Tar_p_l:"<<Tar_p<<endl;
            tp0 << Tar_p[0]-0.09, 0.460+ (Tar_p[1])- 0.317 -0.05, Tar_p[2]-0.055,  0.0f * PI / 180.0f,  -8.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            vel= vel/24;
            acc= acc/24;
        }
        if(ids==3)
        {
            cout <<"Tar_p_r:"<<Tar_p<<endl;
            tp0 << Tar_p[0]-0.06, Tar_p[1], Tar_p[2]-0.075,  0.0f * PI / 180.0f,  -5.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        }
        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

    }
}


void xiaokong_d435_INI(int ids)
{
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        float vel = 5.0f*gSpeed;
        float acc = 2.0f*gSpeed;
        Math::Vector3d pos,rpy;
        Math::Matrix3d mat;
        VectorXd jp0(arms[ids].m_Dofs+1),jp1(arms[ids].m_Dofs+1),jp(arms[ids].m_Dofs);
        VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
        VectorXd tp6(arms[ids].m_Dofs),tp7(arms[ids].m_Dofs),tp8(arms[ids].m_Dofs),tp9(arms[ids].m_Dofs),tp10(arms[ids].m_Dofs),tp11(arms[ids].m_Dofs);

        jp << 0.0f * PI / 180.0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,30.0f * PI / 180.0f,0.0f* PI / 180.0f;     //work position;
        //        jp0 << -80.0f * PI / 180.0,60.0f * PI / 180.0f,-150.0f* PI / 180.0f,0.0f * PI / 180.0f,45.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        jp1 << 0,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f * PI / 180.0f,0.0f * PI / 180.0f,0.0f* PI / 180.0f,0.0f* PI / 180.0f;     //work position;

        arms[ids].m_dynamics.forwardKinematics(jp,pos,rpy);
        arms[ids].m_dynamics.forwardKinematics(jp,pos,mat);
        //      arm.m_dynamics.forwardKinematics(jp,pos,rpy);
        printf("rpy:%f,%f,%f\n",rpy[0],rpy[1],rpy[2]);
        printf("jp1:%f,%f,%f\n",pos[0],pos[1],pos[2]);
        cout <<"mat:"<<mat<<endl;
        if(ids==0)
        {
            cout <<"Tar_p_l:"<<Tar_p<<endl;
            tp0 << Tar_p[0]-0.25, 0.460+ (Tar_p[1])- 0.317 -0.05, Tar_p[2]-0.055,  0.0f * PI / 180.0f,  -8.0f * PI / 180.0f, 0.0f * PI / 180.0f;
            vel= vel/12;
            acc= acc/12;
        }
        if(ids==3)
        {
            cout <<"Tar_p_r:"<<Tar_p<<endl;
            tp0 << Tar_p[0]-0.07, Tar_p[1], Tar_p[2]-0.075,  0.0f * PI / 180.0f,  -5.0f * PI / 180.0f, 0.0f * PI / 180.0f;
        }
        arms[ids].clearRmlPoints();
        arms[ids].addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp0,vel,acc);
        arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);

    }
}


void  xuanguaBack(int ids)
{

    float vel = 5.0f*gSpeed;
    float acc = 2.0f*gSpeed;
    VectorXd jp0(arms[ids].m_Dofs+1), jp1(arms[ids].m_Dofs+1), jp2(arms[ids].m_Dofs+1);
    VectorXd jp(arms[ids].m_Dofs), jp_init(arms[ids].m_Dofs);
    VectorXd tp0(arms[ids].m_Dofs),tp1(arms[ids].m_Dofs),tp2(arms[ids].m_Dofs),tp3(arms[ids].m_Dofs),tp4(arms[ids].m_Dofs),tp5(arms[ids].m_Dofs);
    arms[ids].m_dynamics.getJointPositions(jp);

    if(ids==0)
    {

        jp0 <<  jp[0],  jp[0], jp[0],  jp[0],  jp[0], -180.0f * PI / 180.0f, 0.0;
    }
    if(ids==3)
    {

        jp0 <<  -60.0f * PI / 180.0f,  90.0f * PI / 180.0f,  -150.0f * PI / 180.0f,  0.0,  -30.0f * PI / 180.0f,  0.0, 0.0;
    }

    arms[ids].m_dynamics.setJointPositions(jp_init);
    arms[ids].clearRmlPoints();
    arms[ids].addRmlPoint(MOTION_TYPE::JOINT_RML,jp0,vel,acc);
    arms[ids].beginMotion(MOTION_TYPE::JOINT_RML);
}

void joint_planing(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::JOINT_Planing);
        printf("test JOINT_Planing motion\n");
    }
}

void Gripper_Open(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::GRIPPER_Open);
        printf("test GRIPPER_Open motion\n");
    }
}

void Gripper_Close(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::GRIPPER_Close);
        printf("test GRIPPER_Close motion\n");
    }
}

void joint_error_test(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::JOINT_Error);

        printf("test JOINT_Error motion\n");
    }
}

void Identify_test(int ids)
{
    printf("MotionState:%d\n",arms[ids].getMotionState());
    if(arms[ids].getMotionState() == SM_STATE::STOPPED)
    {
        arms[ids].beginMotion(MOTION_TYPE::Identify);

        printf("test Identify motion\n");
    }
}

void savedata_arm1()
{
    ofstream oFile[3];
    oFile[0].open("/home/yuan_tang/zqh/arm1_joint1_data.csv",ios::out|ios::trunc);
    oFile[1].open("/home/yuan_tang/zqh/arm1_joint2_data.csv",ios::out|ios::trunc);
    oFile[2].open("/home/yuan_tang/zqh/arm1_joint3_data.csv",ios::out|ios::trunc);
    for(int k=0;k<3;k++) oFile[k]<<"t"<<","
                                <<"mpos_r"<<","<<"mvel_r"<<","<<"macc_r"<<","<<"mtor_r"<<","
                               <<"mpos_d"<< ","<<"mvel_d"<< ","<<"macc_d"<<","<<"mtor_r"<<","
                              <<"jpos_r"<< ","<<"jvel_r"<< ","<<"jacc_r"<<","<<"jtor_r"<<","
                             <<"jpos_d"<< ","<<"jvel_d"<< ","<<"jacc_d"<<","<<"jtor_d"<<","
                            <<"cjpos"<< ","<<"cjvel"<< ","<<"cjacc"<<","<<"cjtor"
                           <<endl;
    for(int i=0;i<arms[0].t.size();i++)
    {
        for(int k=0;k<3;k++) oFile[k]<<arms[ids].t[i];
        for(int j=0;j<3;j++)
        {
            oFile[j]<<","<<arms[0].joint_date[j].mpos_r[i]<<","<<arms[0].joint_date[j].mvel_r[i]<<","<<arms[0].joint_date[j].macc_r[i]<<","<<arms[0].joint_date[j].mtor_r[i]<<","
                   <<arms[0].joint_date[j].mpos_d[i]<<","<<arms[0].joint_date[j].mvel_d[i]<<","<<arms[0].joint_date[j].macc_d[i]<<","<<arms[0].joint_date[j].mtor_d[i]<<","
                  <<arms[0].joint_date[j].jpos_r[i]<<","<<arms[0].joint_date[j].jvel_r[i]<<","<<arms[0].joint_date[j].jacc_r[i]<<","<<arms[0].joint_date[j].jtor_r[i]<<","
                 <<arms[0].joint_date[j].jpos_d[i]<<","<<arms[0].joint_date[j].jvel_d[i]<<","<<arms[0].joint_date[j].jacc_d[i]<<","<<arms[0].joint_date[j].jtor_d[i]<<","
                <<arms[0].joint_date[j].cjpos[i]<<","<<arms[0].joint_date[j].cjvel[i]<<","<<arms[0].joint_date[j].cjacc[i]<<","<<arms[0].joint_date[j].cjtor[i]
                  <<endl;
            //           printf("%d\n",i);
        }
    }
    for(int k=0;k<3;k++) oFile[k].close();
}

void savedata_arm4()
{
    ofstream oFile[3];
    oFile[0].open("/home/yuan_tang/zqh/arm4_joint1_data.csv",ios::out|ios::trunc);
    oFile[1].open("/home/yuan_tang/zqh/arm4_joint2_data.csv",ios::out|ios::trunc);
    oFile[2].open("/home/yuan_tang/zqh/arm4_joint3_data.csv",ios::out|ios::trunc);
    for(int k=0;k<3;k++) oFile[k]<<"t"<<","
                                <<"mpos_r"<<","<<"mvel_r"<<","<<"macc_r"<<","<<"mtor_r"<<","
                               <<"mpos_d"<< ","<<"mvel_d"<< ","<<"macc_d"<<","<<"mtor_r"<<","
                              <<"jpos_r"<< ","<<"jvel_r"<< ","<<"jacc_r"<<","<<"jtor_r"<<","
                             <<"jpos_d"<< ","<<"jvel_d"<< ","<<"jacc_d"<<","<<"jtor_d"<<","
                            <<"cjpos"<< ","<<"cjvel"<< ","<<"cjacc"<<","<<"cjtor"
                           <<endl;
    for(int i=0;i<arms[3].t.size();i++)
    {
        for(int k=0;k<3;k++) oFile[k]<<arms[ids].t[i];
        for(int j=0;j<3;j++)
        {
            oFile[j]<<","<<arms[3].joint_date[j].mpos_r[i]<<","<<arms[3].joint_date[j].mvel_r[i]<<","<<arms[3].joint_date[j].macc_r[i]<<","<<arms[3].joint_date[j].mtor_r[i]<<","
                   <<arms[3].joint_date[j].mpos_d[i]<<","<<arms[3].joint_date[j].mvel_d[i]<<","<<arms[3].joint_date[j].macc_d[i]<<","<<arms[3].joint_date[j].mtor_d[i]<<","
                  <<arms[3].joint_date[j].jpos_r[i]<<","<<arms[3].joint_date[j].jvel_r[i]<<","<<arms[3].joint_date[j].jacc_r[i]<<","<<arms[3].joint_date[j].jtor_r[i]<<","
                 <<arms[3].joint_date[j].jpos_d[i]<<","<<arms[3].joint_date[j].jvel_d[i]<<","<<arms[3].joint_date[j].jacc_d[i]<<","<<arms[3].joint_date[j].jtor_d[i]<<","
                <<arms[3].joint_date[j].cjpos[i]<<","<<arms[3].joint_date[j].cjvel[i]<<","<<arms[3].joint_date[j].cjacc[i]<<","<<arms[3].joint_date[j].cjtor[i]
                  <<endl;
            //           printf("%d\n",i);
        }
    }
    for(int k=0;k<3;k++) oFile[k].close();
}

int savedata_support()
{
    ofstream fout;
    fout.open("support.csv",ios::out|ios::trunc);
    if (!fout.is_open()) {
        cout << "文件打开失败" <<endl;
        return 1;
    }
    fout <<"t" << "," <<"force_r"<< "," << "force_l"<< ","
        << "hand_x_r" << "," << "hand_y_r" << ","  << "hand_z_r" << "," << "hand_x_l" << "," << "hand_y_l" << "," << "hand_z_l" << ","
        << "limb_x_r" << "," << "limb_y_r" << ","  << "limb_z_r" << "," << "limb_x_l" << "," << "limb_y_l" << "," << "limb_z_l" << ","
        << "hand_vxy_mean_r" << "," << "hand_vxy_mean_l" << ","  << "hand_vz_mean_r" << "," << "hand_vz_mean_r" <<endl;

    for(int i=0;i< m_support.t.size();i++)
    {
        fout <<m_support.t[i] << "," <<m_support.force_r[i]<< "," << m_support.force_l[i]<< ","
            << m_support.hand_x_r[i] << "," << m_support.hand_y_r[i] << ","  << m_support.hand_z_r[i] << ","
            << m_support.hand_x_l[i] << "," << m_support.hand_y_l[i] << "," << m_support.hand_z_l[i] << ","
            << m_support.limb_x_r[i] << "," << m_support.limb_y_r[i] << ","  << m_support.limb_z_r[i] << ","
            << m_support.limb_x_l[i] << "," << m_support.limb_y_l[i] << "," << m_support.limb_z_l[i] << ","
            << m_support.vxy_mean_r[i]<< "," << m_support.vxy_mean_l[i] << ","
            << m_support.vz_mean_r[i] << "," << m_support.vz_mean_l[i]<<endl;
    }
    fout.close();
    return 0;
}

void plotcurve()
{
    while(true)
    {
        //        cout<<"plotcurve"<<endl;
        usleep(50000);
        curvedata[0] = arms[3].m_Desire.posd[1];
        curvedata[1] = arms[3].m_Desire.veld[1];
        curvedata[2] = arms[3].m_Desire.accd[1];
        vrep.addGraphData(curvedata,3,"GraphData1");

    }
}
static void Top_data_get(void* arg)
{
    unsigned long ov;
    float rtPeroid = arms[3].m_TStep*1000000000;
    rt_task_set_periodic(NULL, TM_NOW, rtPeroid);

    while(!Quitflag)
    {
        Hand_End_P<<  m_mujoco.d->sensordata[0], m_mujoco.d->sensordata[1], m_mujoco.d->sensordata[2];
        Hand_End_V<< m_mujoco.d->sensordata[0], m_mujoco.d->sensordata[1], m_mujoco.d->sensordata[2];

        Hand_End_P1<<m_mujoco.d->sensordata[3], m_mujoco.d->sensordata[4], m_mujoco.d->sensordata[5];
        Hand_End_V1<< m_mujoco.d->sensordata[15], m_mujoco.d->sensordata[16], m_mujoco.d->sensordata[17];

        //        cout<<"TipToOrigin_P0:"<<Hand_End_P[2]<<endl;
        //        cout<<"TipToOrigin_P1:"<<Hand_End_P1[2]<<endl;


        count1++;
        if(count1%200==0)
        {   count2++;
            rt_printf("time:%d\n",count2);
        }
        rt_task_wait_period(&ov);
    }
}

static void TopCtrl(void* arg)
{
    unsigned long ov;
    float rtPeroid = arms[3].m_TStep*1000000000;
    rt_task_set_periodic(NULL, TM_NOW, rtPeroid);
    Hand_End_P_Last<<  m_mujoco.d->sensordata[0], m_mujoco.d->sensordata[1], m_mujoco.d->sensordata[2];
    Hand_End_P1_Last<<m_mujoco.d->sensordata[3], m_mujoco.d->sensordata[4], m_mujoco.d->sensordata[5];
    Hand_End_V_Last << 0.0, 0.0, 0.0;
    Hand_End_V1_Last << 0.0, 0.0, 0.0;

    //    m_support.hand_v200.erase(m_support.hand_v200.begin());
    //    float time=
    //  cout<<"initposz:"<<Hand_End_P_Last[2]<<endl;
    //        cout<<"TipToOrigin_P1:"<<Hand_End_P1[2]<<endl;
    while(true)
    {
        m_ftsensor.forceControl(1);
        m_ftsensor.forceControl(2);
        Hand_End_P <<  m_mujoco.d->sensordata[0], m_mujoco.d->sensordata[1], m_mujoco.d->sensordata[2];
        Limb_End_P <<  m_mujoco.d->sensordata[6], m_mujoco.d->sensordata[7], m_mujoco.d->sensordata[8];

//        cout<<"topctlP:"<<Hand_End_P[2]<<endl;
        for(int i =0;i<3;i++)
        {
            Hand_End_V[i]  =  (Hand_End_P[i] - Hand_End_P_Last[i])/arms[3].m_TStep;
            Hand_End_P_Last[i] =  Hand_End_P[i];
        }
        Hand_End_P1<<m_mujoco.d->sensordata[3], m_mujoco.d->sensordata[4], m_mujoco.d->sensordata[5];
        Limb_End_P1 <<  m_mujoco.d->sensordata[9], m_mujoco.d->sensordata[10], m_mujoco.d->sensordata[11];
        for(int i =0;i<3;i++)
        {
            Hand_End_V1[i]  =  (Hand_End_P1[i] - Hand_End_P1_Last[i])/arms[3].m_TStep;
            Hand_End_P1_Last[i] =  Hand_End_P1[i];
        }

        //速度滤波
        for(int i =0;i<3;i++)
        {
            Hand_End_V[i]  =  0.8*Hand_End_V_Last[i] + 0.2*Hand_End_V[i];
            Hand_End_V1[i]  =  0.8*Hand_End_V1_Last[i] + 0.2*Hand_End_V1[i];

            Hand_End_V_Last[i] =  Hand_End_V[i];
            Hand_End_V1_Last[i] =  Hand_End_V1[i];
        }
        //求解V_xy
        Hand_End_V_xy  = sqrt(Hand_End_V[0]*Hand_End_V[0] + Hand_End_V[1]*Hand_End_V[1]);
        Hand_End_V1_xy = sqrt(Hand_End_V1[0]*Hand_End_V1[0] + Hand_End_V1[1]*Hand_End_V1[1]);

        //求解右臂平均速度
        int len =  m_support.hand_vxy200_r.size();
        m_support.hand_vxy200_r.push_back(Hand_End_V_xy);
        m_support.hand_vz200_r.push_back(Hand_End_V[2]);

        if(len == 81)
        {
            m_support.hand_vxy200_r.erase( m_support.hand_vxy200_r.begin());
            m_support.hand_vz200_r.erase( m_support.hand_vz200_r.begin());

            for(int i=0; i<len; i++)
            {
                if(m_support.hand_vxy200_r[i] <0)  m_support.hand_vxy200_r[i] = -m_support.hand_vxy200_r[i];
                if(m_support.hand_vz200_r[i] <0)  m_support.hand_vz200_r[i] = -m_support.hand_vz200_r[i];

            }
            m_support.hand_vxy_mean_r = sum_vector(m_support.hand_vxy200_r)/(len-1);
            m_support.hand_vz_mean_r = sum_vector(m_support.hand_vz200_r)/(len-1);
        }

        //求解左臂平均速度
        int len1 =  m_support.hand_vxy200_l.size();
        m_support.hand_vxy200_l.push_back(Hand_End_V1_xy);
        m_support.hand_vz200_l.push_back(Hand_End_V1[2]);

        if(len1 == 6)
        {
            m_support.hand_vxy200_l.erase( m_support.hand_vxy200_l.begin());
            m_support.hand_vz200_l.erase( m_support.hand_vz200_l.begin());

            for(int i=0; i<len1; i++)
            {
                if(m_support.hand_vxy200_l[i] <0)  m_support.hand_vxy200_l[i] = -m_support.hand_vxy200_l[i];
                if(m_support.hand_vz200_l[i] <0)  m_support.hand_vz200_l[i] = -m_support.hand_vz200_l[i];

            }
            m_support.hand_vxy_mean_l = sum_vector(m_support.hand_vxy200_l)/(len1-1);
            m_support.hand_vz_mean_l = sum_vector(m_support.hand_vz200_l)/(len1-1);
        }

        //        cout<<"TipToOrigin_P0:"<<Hand_End_P[2]<<endl;
        // cout<<"TipToOrigin_P1:"<<Hand_End_P1[2]<<endl;
        //          cout<<"TipToOrigin_P1:"<<Hand_End_V_xy<<endl;

        //             cout<<"Limb_End_P:"<<Limb_End_P1<<endl;



        //数据保存
        m_support.t.push_back(m_time);
        m_support.force_l.push_back(m_ftsensor.ft2[2]);
        m_support.force_r.push_back(m_ftsensor.ft3[2]);

        m_support.hand_x_r.push_back(Hand_End_P[0]);
        m_support.hand_y_r.push_back(Hand_End_P[1]);
        m_support.hand_z_r.push_back(Hand_End_P[2]);

        m_support.hand_x_l.push_back(Hand_End_P1[0]);
        m_support.hand_y_l.push_back(Hand_End_P1[1]);
        m_support.hand_z_l.push_back(Hand_End_P1[2]);

        m_support.limb_x_r.push_back(Limb_End_P[0]);
        m_support.limb_y_r.push_back(Limb_End_P[1]);
        m_support.limb_z_r.push_back(Limb_End_P[2]);

        m_support.limb_x_l.push_back(Limb_End_P1[0]);
        m_support.limb_y_l.push_back(Limb_End_P1[1]);
        m_support.limb_z_l.push_back(Limb_End_P1[2]);


        m_support.vxy_mean_r.push_back(m_support.hand_vxy_mean_r);
        m_support.vxy_mean_l.push_back(m_support.hand_vxy_mean_l);
        m_support.vz_mean_r.push_back(m_support.hand_vz_mean_r);
        m_support.vz_mean_l.push_back(m_support.hand_vz_mean_l);


        count1++;
        if(count1%200==0)
        {   count2++;
            //            rt_printf("time:%d\n",count2);
        }
        m_time = m_time + arms[3].m_TStep;
        rt_task_wait_period(&ov);
    }
}


void state_change()
{
    while(true)
    {
        usleep(5000);
        bool flag_p = Hand_End_P[2]>0.23 && Hand_End_P1[2]>0.23;

        bool flag_vxy = m_support.hand_vxy_mean_r > 0.15 && m_support.hand_vxy_mean_l > 0.15;

        bool flag_vz =  m_support.hand_vz_mean_r< 0.05  && m_support.hand_vz_mean_l< 0.05;
//        cout <<"flag_p:"  << flag_p   <<"value:"<<Hand_End_P[2]<<endl;
//        cout <<"flag_vxy:"<< flag_vxy <<endl;
//        cout <<"flag_vz:" << flag_vz  <<endl;
//        printf("\n");
        if(state_support == task_state_support::none /*&& m_mujoco.d->ncon == 0*/ && flag_p && flag_vxy && flag_vz)
        {
            printf("进入过顶支撑构型\n");
            supporting(0);
            supporting(3);
            usleep(6000000);
            state_support = task_state_support::support;
        }

        flag_p = Hand_End_P[2]>0.23 && Hand_End_P1[2]>0.23;
        flag_vxy = m_support.hand_vxy_mean_r < 0.02 && m_support.hand_vxy_mean_l < 0.02;
        flag_vz =  m_support.hand_vz_mean_r< 0.02  && m_support.hand_vz_mean_l< 0.02;
        //        cout <<"flag_p:"  << flag_p   <<endl;
        //        cout <<"flag_vxy:"<< flag_vxy <<endl;
        //        cout <<"flag_vz:" << flag_vz  <<endl;
        //         printf("\n");
        if(state_support == task_state_support::support /*&& m_mujoco.d->ncon == 0*/ && flag_p && flag_vxy && flag_vz)
        {
            printf("进入力控模式\n");
            state_support = task_state_support::force;
            support_force(0);
            support_force(3);
        }

        flag_p = Hand_End_P[2]<-0.4 && Hand_End_P1[2]<-0.4;
        flag_vxy = m_support.hand_vxy_mean_r < 0.02 && m_support.hand_vxy_mean_l < 0.02;
        flag_vz =  m_support.hand_vz_mean_r< 0.02  && m_support.hand_vz_mean_l< 0.02;

        if(state_support == task_state_support::force /*&& m_mujoco.d->ncon == 0*/ && flag_p &&  flag_vxy && flag_vz)
        {
            //            printf("进入初始构型\n");
            state_support = task_state_support::none;
        }

        //        if(m_mujoco.d->ncon >0)
        //        {
        //            printf("进入重力平衡模式\n");
        //            //            state_support = task_state_support::gblance;
        //            state_support = task_state_support::none;
        //            g_blance(0);
        //            g_blance(3);
        //        }
        //          m_ftsensor.forceControl(3);
        //                  cout <<"F2:"<<m_ftsensor.ft2<<endl;
        //                  cout <<"F3:"<<m_ftsensor.ft3<<endl;
        //                cout <<"Hand_End_P"<<Hand_End_P<<endl;
        //                cout <<"Hand_End_P1"<<Hand_End_P1<<endl;
        //         cout <<"contactnum:" <<  m_mujoco.d->ncon<<endl;
        //       printf("进入过顶支撑构型\n");
    }



}

void keyscan()
{
    bool beginTest = false;
    bool enableflag = false;
    int testNum = 0;
    char cmdinfo[1024];
    sprintf(cmdinfo,"m:\t toggle enable\n 1-9:\t test num\n q:\t quit\n h:\t help\n:",enableflag,testNum,beginTest?"running":"stopped");
    cout<<cmdinfo;
    rt_printf("zqh:%s, %s\n",b[ids],a[m_task_state]);

    while(true)
    {
        cin>>cmd;
        if(cmd=='a')
        {
            ids=ids+1;
            ids = ids%3;
            //            rt_printf("%s\n",b[ids]);
        }

        if(cmd=='+')
        {
            m_task_state=m_task_state+1;
            m_task_state = m_task_state % 6;
            //            rt_printf("%s\n",a[m_task_state]);
        }
        rt_printf("%s,      %s\n",b[ids],a[m_task_state]);

        if(cmd == 'Q' || cmd == 'q'){
            Quitflag = true;
            vrep.close();
            savedata_support();
            phone.EndCPython();

            break;
        }
        switch(cmd)
        {
        case 'h':
            cout<<cmdinfo;
            break;
        case 'm':
            if(ids==0)
            {
                enableflag = !enableflag;
                arms[0].enableMotors(enableflag);
            }
            else if(ids==1)
            {
                enableflag = !enableflag;
                arms[3].enableMotors(enableflag);
            }
            else if(ids==2)
            {
                enableflag = !enableflag;
                arms[0].enableMotors(enableflag);
                arms[3].enableMotors(enableflag);
            }
            break;
        case 'z':
            arms[0].setZeroPos();
            arms[3].setZeroPos();
            break;
        case 's':
            arms[0].stopMotion();
            arms[3].stopMotion();
            printf("stop motion\n");
            break;
        case '0':
            gohome(0);
            gohome(3);
            printf("action1\n");
            break;
        case '1':
            action2(3);
            break;
        case '5':
            //            birdhead_planing(0);
            birdhead_planing(3);
            //            sleep();
            break;
        case 'i':
            initpos(0);
            initpos(3);
            break;
        case 'p':
            supporting(0);
            supporting(3);
            break;
        case 'f':
//            support_force(0);
//            support_force(3);
            flag_f = 1;
            break;

        case 'g':

            if(ids==0)
            {
                printf("g_blance_two limb\n");
                //                g_blance(0);
                g_blance(3);
            }
            else if(ids==1)
            {
                printf("g_blance_r limb\n");
                g_blance(3);
            }
            else if(ids==2)
            {
                printf("g_blance_l limb\n");
                //                g_blance(0);
                g_blance(3);
            }

            break;
        case 'o':
            if(ids==0)
            {
                printf("open_d\n");
                if(m_task_state==task_state::gdzc)
                {
                    //                    shouzhua_open_tor_sup(0);
                    shouzhua_open_tor_sup(3);
                    sleep(2);
                    //                    shouzhua_stop_tor(0);
                    shouzhua_stop_tor(3);
                }
                else
                {
                    //                    shouzhua_open_tor(0);
                    shouzhua_open_tor(3);
                    sleep(2);
                    //                    shouzhua_stop_tor(0);
                    shouzhua_stop_tor(3);
                }
            }
            else if(ids==1)
            {
                printf("open_r\n");
                shouzhua_open_tor(3);
                sleep(2);
                shouzhua_stop_tor(3);
            }
            else if(ids==2)
            {
                printf("open_l\n");
                //                shouzhua_open_tor(0);
                sleep(2);
                //                shouzhua_stop_tor(0);

            }
            break;
        case 'c':
            if(ids==0)
            {
                //                shouzhua_close_tor(0);
                shouzhua_close_tor(3);
            }
            else if(ids==1)
            {
                shouzhua_close_tor(3);
            }
            else if(ids==2)
            {
                //                shouzhua_close_tor(0);
            }

            break;
        case'y':
            //            Enable485(0);
            Enable485(3);
            break;
        case'n':
            //            Disable485(0);
            Disable485(3);
            break;
        default:
            break;
        }
    }
}

void processt265()
{
    Eigen::Vector3f    cam_p,cam_p1,cam_p2,cam_p3,cam_p4;
    Eigen::Vector3f    cam_pry,cam_pry1,cam_pry2,cam_pry3,cam_pry4;
    Eigen::Quaternionf cam_q,cam_q1,cam_q2,cam_q3,cam_q4,cam_q0;
    Eigen::Vector3f    cam_v,cam_v1,cam_v2,cam_v3,cam_v4;
    Eigen::Vector3f    cam_a,cam_a1,cam_a2,cam_a3,cam_a4;
    while(true)
    {
        usleep(10000);
        /*********************get two t265date******************************/
        bool ret1 = m_t265.getPose(cam_p,cam_pry,cam_q,cam_v,cam_a,1);
        bool ret2 = m_t265.getPose(cam_p1,cam_pry1,cam_q1,cam_v1,cam_a1,2);
        bool ret3 = m_t265.getPose(cam_p2,cam_pry2,cam_q2,cam_v2,cam_a2,3);
        bool ret4 = m_t265.getPose(cam_p3,cam_pry3,cam_q3,cam_v3,cam_a3,4);
        bool ret5 = m_t265.getPose(cam_p4,cam_pry4,cam_q4,cam_v4,cam_a4,5);

        bool ret = ret1 && ret2 && ret3 && ret4 && ret5;
        if(ret)
        {
            /********************camer_base*******************/
            cam[0].p = cam_p;
            cam[0].pry = cam_pry;
            cam[0].q = cam_q;
            cam[0].v = cam_v;
            cam[0].a = cam_a;

            Eigen::Matrix3d Rot_init,Rot_x,Rot_y,Rot_z;

            double theta =-180.0*PI/180,theta1;

            Rot_y<< cos(theta), 0, sin(theta),
                    0,          1,         0,
                    -sin(theta), 0, cos(theta);
            Rot_init = cam[0].q.normalized().toRotationMatrix();

            cam[0].mat = Rot_init*Rot_y;
            //                    cam.mat = Rot_y*Rot_init;
            cam[0].q= Eigen::Quaterniond(cam[0].mat);
            cam[0].q.w()= -cam[0].q.w();
            cam[0].q.x()= -cam[0].q.x();

            cam[0].q = cam[0].q_last.slerp(0.1,cam[0].q);
            cam[0].q_last = cam[0].q;
            cam[0].mat  = cam[0].q.normalized().toRotationMatrix();


            /********************camer link1_r*******************/
            cam[1].p = cam_p1;
            cam[1].pry = cam_pry1;
            cam[1].q = cam_q1;
            cam[1].v = cam_v1;
            cam[1].a = cam_a1;

            //            cam.q.normalized().to
            Eigen::Matrix3d R1= cam[1].q.normalized().toRotationMatrix();
            Eigen::Vector3d eulerAngle=R1.eulerAngles(2,1,0);
            //            printf("1111eulerAngle:[0]: %0.3f, eulerAngle[1]: %0.3f, eulerAngle[2]: %0.3f\n",eulerAngle[0]*180/PI,eulerAngle[1]*180/PI,eulerAngle[2]*180/PI);
            theta = -90.0*PI/180;
            Rot_y<< cos(theta), 0, sin(theta),
                    0,          1,         0,
                    -sin(theta), 0, cos(theta);

            Rot_z<< cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta),  0,
                    0,              0,       1;

            cam[1].q.w()= cam[1].q.w();
            cam_q0.x()= cam[1].q.x();

            cam[1].q.x()= cam[1].q.y();
            cam[1].q.y()= -cam_q0.x();
            cam[1].mat  =cam[1].q.normalized().toRotationMatrix();

            /*********************camer link2_r********************/
            cam[2].p = cam_p2;
            cam[2].pry = cam_pry2;
            cam[2].q = cam_q2;

            cam[2].q.w()=cam[2].q.w();
            cam_q0.x()= cam[2].q.x();

            cam[2].q.x()= cam[2].q.y();
            cam[2].q.y()= -cam_q0.x();
            cam[2].mat  = cam[2].q.normalized().toRotationMatrix();


            /*********************camer link1_l********************/
            cam[3].p = cam_p3;
            cam[3].pry = cam_pry3;
            cam[3].q = cam_q3;

            theta =-180.0*PI/180;
            theta1 =90.0*PI/180;
            Rot_y<< cos(theta), 0, sin(theta),
                    0,          1,         0,
                    -sin(theta), 0, cos(theta);

            Rot_z<< cos(theta1), -sin(theta1), 0,
                    sin(theta1), cos(theta1),  0,
                    0,              0,       1;

            Rot_init = cam[3].q.normalized().toRotationMatrix();

            cam[3].mat = Rot_init*Rot_y;
            cam[3].q= Eigen::Quaterniond(cam[3].mat);
            cam[3].q.z()= -cam[3].q.z();
            cam_q0.x()= cam[3].q.y();
            cam[3].q.y()= -cam[3].q.x();
            cam[3].q.x()= -cam_q0.x();
            cam[3].mat  = cam[3].q.normalized().toRotationMatrix();

            /*********************camer link2_l********************/
            cam[4].p = cam_p4;
            cam[4].pry = cam_pry4;
            cam[4].q = cam_q4;

            theta =-180.0*PI/180;
            theta1 =90.0*PI/180;
            Rot_y<< cos(theta), 0, sin(theta),
                    0,          1,         0,
                    -sin(theta), 0, cos(theta);

            Rot_z<< cos(theta1), -sin(theta1), 0,
                    sin(theta1), cos(theta1),  0,
                    0,              0,       1;

            Rot_init = cam[4].q.normalized().toRotationMatrix();

            cam[4].mat = Rot_init*Rot_y;
            cam[4].q= Eigen::Quaterniond(cam[4].mat);
            cam[4].q.z()= -cam[4].q.z();
            cam_q0.x()= cam[4].q.y();
            cam[4].q.y()= -cam[4].q.x();
            cam[4].q.x()= -cam_q0.x();
            cam[4].mat  = cam[4].q.normalized().toRotationMatrix();

            /****************************************右臂肘关节关节角度计算及对比*******************************************/
            //joint1_r
            joint1_r.mat =  (cam[0].mat.inverse())*cam[1].mat;
            joint1_r.q = Eigen::Quaterniond(joint1_r.mat);


            joint1_r.euler = joint1_r.q.matrix().eulerAngles(0,1,2);
            if(joint1_r.euler[0]>1.57)
            {
                joint1_r.euler[0] = joint1_r.euler[0] - PI;
            }


            if(joint1_r.euler[1]>=1.57)
            {
                joint1_r.euler[1] = PI - joint1_r.euler[1] ;
            }
            if(joint1_r.euler[1]<= -1.57)
            {
                joint1_r.euler[1] = -PI - joint1_r.euler[1] ;
            }

            if(joint1_r.euler[2]>=1.57)
            {
                joint1_r.euler[2] = joint1_r.euler[2]-PI ;
            }
            if(joint1_r.euler[2]<=-1.57)
            {
                joint1_r.euler[2] = joint1_r.euler[2]+PI ;
            }

            //joint2_r
            Eigen::Matrix3d mat_j2;
            joint2_r.mat = (cam[1].mat.inverse())*cam[2].mat;
            mat_j2 = joint2_r.mat;
            theta = acos((mat_j2(0,0)+mat_j2(1,1)+mat_j2(2,2)-1)/2);
            double axs_x = (mat_j2(2,1)-mat_j2(1,2))/2*sin(theta);
            double axs_y = (mat_j2(0,2)-mat_j2(2,0))/2*sin(theta);
            double axs_z = (mat_j2(1,0)-mat_j2(0,1))/2*sin(theta);
            double axs_norm = sqrt(pow(axs_x,2)+pow(axs_y,2)+pow(axs_z,2));
            joint2_r.axs << axs_x/axs_norm,axs_y/axs_norm,axs_z/axs_norm;
            joint2_r.theta = theta;

            joint2_r.rotation_vector.fromRotationMatrix(mat_j2);
            joint2_r.theta =  joint2_r.rotation_vector.angle();
            //             cout << "rotation_vector " << "angle is: " << joint2.rotation_vector.angle() * (180 / M_PI)
            //                                              << " axis is: " << joint2.rotation_vector.axis().transpose() << endl;

            //             printf(" joint2.axs[0]: %0.3f,  joint2.axs[1]: %0.3f,  joint2.axs[2]: %0.3f\n", joint2.axs[0], joint2.axs[1], joint2.axs[2]);
            //             printf("joint2.theta: %0.3f\n",joint2.theta*180/PI);

            /****************************************左臂关节关节角度计算及对比*******************************************/
            //joint1_l
            joint1_l.mat =  (cam[0].mat.inverse())*cam[3].mat;
            joint1_l.q = Eigen::Quaterniond(joint1_l.mat);

            //joint2_l
            joint2_l.mat = (cam[3].mat.inverse())*cam[4].mat;
            joint2_l.rotation_vector.fromRotationMatrix(joint2_l.mat);
            joint2_l.theta =  joint2_l.rotation_vector.angle();

        }
    }
}

void mujoco_initpos_set()
{
    Eigen::Vector3f    cam_p,cam_p1,cam_p2,cam_p3,cam_p4;
    Eigen::Vector3f    cam_pry,cam_pry1,cam_pry2,cam_pry3,cam_pry4;
    Eigen::Quaternionf cam_q,cam_q1,cam_q2,cam_q3,cam_q4,cam_q0;
    Eigen::Vector3f    cam_v,cam_v1,cam_v2,cam_v3,cam_v4;
    Eigen::Vector3f    cam_a,cam_a1,cam_a2,cam_a3,cam_a4;
    /*********************get two t265date******************************/
    bool ret1 = m_t265.getPose(cam_p,cam_pry,cam_q,cam_v,cam_a,1);
    bool ret2 = m_t265.getPose(cam_p1,cam_pry1,cam_q1,cam_v1,cam_a1,2);
    bool ret3 = m_t265.getPose(cam_p2,cam_pry2,cam_q2,cam_v2,cam_a2,3);
    bool ret4 = m_t265.getPose(cam_p3,cam_pry3,cam_q3,cam_v3,cam_a3,4);
    bool ret5 = m_t265.getPose(cam_p4,cam_pry4,cam_q4,cam_v4,cam_a4,5);

    bool ret = ret1 && ret2 && ret3 && ret4 && ret5;
    if(ret)
    {
        /********************camer_base*******************/
        cam[0].p = cam_p;
        cam[0].pry = cam_pry;
        cam[0].q = cam_q;
        cam[0].v = cam_v;
        cam[0].a = cam_a;

        Eigen::Matrix3d Rot_init,Rot_x,Rot_y,Rot_z;

        double theta =-180.0*PI/180,theta1;

        Rot_y<< cos(theta), 0, sin(theta),
                0,          1,         0,
                -sin(theta), 0, cos(theta);
        Rot_init = cam[0].q.normalized().toRotationMatrix();

        cam[0].mat = Rot_init*Rot_y;
        //                    cam.mat = Rot_y*Rot_init;
        cam[0].q= Eigen::Quaterniond(cam[0].mat);
        cam[0].q.w()= -cam[0].q.w();
        cam[0].q.x()= -cam[0].q.x();

        cam[0].q = cam[0].q_last.slerp(0.1,cam[0].q);
        cam[0].q_last = cam[0].q;
        cam[0].mat  = cam[0].q.normalized().toRotationMatrix();


        /********************camer link1_r*******************/
        cam[1].p = cam_p1;
        cam[1].pry = cam_pry1;
        cam[1].q = cam_q1;
        cam[1].v = cam_v1;
        cam[1].a = cam_a1;

        //            cam.q.normalized().to
        Eigen::Matrix3d R1= cam[1].q.normalized().toRotationMatrix();
        Eigen::Vector3d eulerAngle=R1.eulerAngles(2,1,0);
        //            printf("1111eulerAngle:[0]: %0.3f, eulerAngle[1]: %0.3f, eulerAngle[2]: %0.3f\n",eulerAngle[0]*180/PI,eulerAngle[1]*180/PI,eulerAngle[2]*180/PI);
        theta = -90.0*PI/180;
        Rot_y<< cos(theta), 0, sin(theta),
                0,          1,         0,
                -sin(theta), 0, cos(theta);

        Rot_z<< cos(theta), -sin(theta), 0,
                sin(theta), cos(theta),  0,
                0,              0,       1;

        cam[1].q.w()= cam[1].q.w();
        cam_q0.x()= cam[1].q.x();

        cam[1].q.x()= cam[1].q.y();
        cam[1].q.y()= -cam_q0.x();
        cam[1].mat  =cam[1].q.normalized().toRotationMatrix();

        /*********************camer link2_r********************/
        cam[2].p = cam_p2;
        cam[2].pry = cam_pry2;
        cam[2].q = cam_q2;

        cam[2].q.w()=cam[2].q.w();
        cam_q0.x()= cam[2].q.x();

        cam[2].q.x()= cam[2].q.y();
        cam[2].q.y()= -cam_q0.x();
        cam[2].mat  = cam[2].q.normalized().toRotationMatrix();
        /*********************camer link1_l********************/
        cam[3].p = cam_p3;
        cam[3].pry = cam_pry3;
        cam[3].q = cam_q3;

        theta =-180.0*PI/180;
        theta1 =90.0*PI/180;
        Rot_y<< cos(theta), 0, sin(theta),
                0,          1,         0,
                -sin(theta), 0, cos(theta);

        Rot_z<< cos(theta1), -sin(theta1), 0,
                sin(theta1), cos(theta1),  0,
                0,              0,       1;

        Rot_init = cam[3].q.normalized().toRotationMatrix();

        cam[3].mat = Rot_init*Rot_y;
        cam[3].q= Eigen::Quaterniond(cam[3].mat);
        cam[3].q.z()= -cam[3].q.z();
        cam_q0.x()= cam[3].q.y();
        cam[3].q.y()= -cam[3].q.x();
        cam[3].q.x()= -cam_q0.x();
        cam[3].mat  = cam[3].q.normalized().toRotationMatrix();

        /*********************camer link2_l********************/
        cam[4].p = cam_p4;
        cam[4].pry = cam_pry4;
        cam[4].q = cam_q4;

        theta =-180.0*PI/180;
        theta1 =90.0*PI/180;
        Rot_y<< cos(theta), 0, sin(theta),
                0,          1,         0,
                -sin(theta), 0, cos(theta);

        Rot_z<< cos(theta1), -sin(theta1), 0,
                sin(theta1), cos(theta1),  0,
                0,              0,       1;

        Rot_init = cam[4].q.normalized().toRotationMatrix();

        cam[4].mat = Rot_init*Rot_y;
        cam[4].q= Eigen::Quaterniond(cam[4].mat);
        cam[4].q.z()= -cam[4].q.z();
        cam_q0.x()= cam[4].q.y();
        cam[4].q.y()= -cam[4].q.x();
        cam[4].q.x()= -cam_q0.x();
        cam[4].mat  = cam[4].q.normalized().toRotationMatrix();

        /****************************************右臂肘关节关节角度计算及对比*******************************************/
        //joint1_r
        joint1_r.mat =  (cam[0].mat.inverse())*cam[1].mat;
        joint1_r.q = Eigen::Quaterniond(joint1_r.mat);

        //joint2_r
        Eigen::Matrix3d mat_j2;
        joint2_r.mat = (cam[1].mat.inverse())*cam[2].mat;
        mat_j2 = joint2_r.mat;
        joint2_r.rotation_vector.fromRotationMatrix(mat_j2);
        joint2_r.theta =  joint2_r.rotation_vector.angle();
        /****************************************左臂关节关节角度计算及对比*******************************************/
        //joint1_l
        joint1_l.mat =  (cam[0].mat.inverse())*cam[3].mat;
        joint1_l.q = Eigen::Quaterniond(joint1_l.mat);

        //joint2_l
        joint2_l.mat = (cam[3].mat.inverse())*cam[4].mat;
        joint2_l.rotation_vector.fromRotationMatrix(joint2_l.mat);
        joint2_l.theta =  joint2_l.rotation_vector.angle();

    }

    m_mujoco.d->qpos[0] =  0.0;
    m_mujoco.d->qpos[1] =  0.0;
    m_mujoco.d->qpos[2] =  0.0;

    m_mujoco.d->qpos[3] =  1.0;
    m_mujoco.d->qpos[4] =  0.0;
    m_mujoco.d->qpos[5] =  0.0;
    m_mujoco.d->qpos[6] =  0.0;

    //右hunan臂位置和姿态
    m_mujoco.d->qpos[19] = joint1_r.q.w();
    m_mujoco.d->qpos[20] = joint1_r.q.x();
    m_mujoco.d->qpos[21] = joint1_r.q.y();
    m_mujoco.d->qpos[22] = joint1_r.q.z();
    m_mujoco.d->qpos[23] = joint2_r.theta;
    //右hunan臂位置和姿态
    m_mujoco.d->qpos[24] = joint1_l.q.w();
    m_mujoco.d->qpos[25] = joint1_l.q.x();
    m_mujoco.d->qpos[26] = joint1_l.q.y();
    m_mujoco.d->qpos[27] = joint1_l.q.z();
    m_mujoco.d->qpos[28] = joint2_l.theta;

    //右SRAs臂位置和姿态
    m_mujoco.d->qpos[7] =  arms[3].m_Desire.posd[0];
    m_mujoco.d->qpos[8] =  arms[3].m_Desire.posd[1];
    m_mujoco.d->qpos[9] =  arms[3].m_Desire.posd[2];

    m_mujoco.d->qpos[10] =  arms[3].m_Desire.posd[3];
    m_mujoco.d->qpos[11] =  arms[3].m_Desire.posd[4];
    m_mujoco.d->qpos[12] =  arms[3].m_Desire.posd[5];

    //左SRAs臂位置和姿态
    m_mujoco.d->qpos[13] =  arms[0].m_Desire.posd[0];
    m_mujoco.d->qpos[14] =  arms[0].m_Desire.posd[1];
    m_mujoco.d->qpos[15] =  arms[0].m_Desire.posd[2];

    m_mujoco.d->qpos[16] =  arms[0].m_Desire.posd[3];
    m_mujoco.d->qpos[17] =  arms[0].m_Desire.posd[4];
    m_mujoco.d->qpos[18] =  arms[0].m_Desire.posd[5];

    m_mujoco.mujocoUpdate();

}

void init(bool vrepEnable,const char* vrepIP)
{
    //parallel chain
    //       Math::Vector3d   Dummpy_pos2 = {0.38502,0.0,0.02958};
    //       char *linkname = "Link3";
    for(int i=0;i<3;i++)
    {
        Hand_End_P_Last[i] =0;
        Hand_End_P1_Last[i] =0;
    }

    //ft sensor
    //    m_ftsensor.init("192.168.1.103");
    //    m_ftsensor.start();

    //serieds chain
    Math::Vector3d   Dummpy_pos1 = /*{0.0157,0.0,0.0};*/{0.0,0.0,0.0};
    Math::Vector3d   Dummpy_pos2 = /*{0.0157,0.0,0.0};*/{0.0,0.0,0.0};

    const char *linkname = "Link6";
    if(vrepEnable)
    {
        if(vrep.connect(vrepIP))
        {
            rt_printf("vrep server(%s) connected!\n",vrepIP);
            //            vrep.setGraphParam(graph_curves,graph_points);
        }
    }
    //    float lu_sensor_angle_init[6]= {191.514, 110.545, 142.669,0.0,0.0,0.0};
    float lu_sensor_angle_init[6]= {-82.046, 39.331, 151.194,0,0,0};
    //    float ld_sensor_angle_init[6]= {0};
    //    float rd_sensor_angle_init[6]= {0};
    float ru_sensor_angle_init[6]= {102.964, -22.127,145.525,0,0,0};

    canPorts[0].init(1,lu_sensor_angle_init);
    canPorts[3].init(4,ru_sensor_angle_init);

    m_485Ports[0].init("/dev/ttyUSB0");
    m_485Ports[1].init("/dev/ttyUSB1");

    rbdls[0].init("/home/mjq/02_project/03_hit_superlimbs/SRL2/model/HitLimb_up.urdf",linkname,Dummpy_pos1);
    //    rbdls[1].init("./HitLimb_up_s1.urdf",linkname,Dummpy_pos2);
    //    rbdls[2].init("./HitLimb_up_s1.urdf",linkname,Dummpy_pos2);
    rbdls[3].init("/home/mjq/02_project/03_hit_superlimbs/SRL2/model/HitLimb_up.urdf",linkname,Dummpy_pos2);

    arms[0].init(&canPorts[0],m_485Ports[0],&vrep,&rbdls[0], "JointPosition1", "GraphData1", 0.005, 8);
    //    arms[1].init(&canPorts[1],m_485Ports[0],&vrep,&rbdls[1],"JointPosition2","GraphData2",0.005,7);
    //    arms[2].init(&canPorts[2],m_485Ports[1],&vrep,&rbdls[2],"JointPosition3","GraphData3",0.005,8);
    arms[3].init(&canPorts[3],m_485Ports[1],&vrep,&rbdls[3], "JointPosition4", "GraphData4", 0.005, 9);

    Holo.init(arms[0],arms[1],arms[2],arms[3],arms[3].m_t265);

    //mjq canPorts[0].startCanRec("cantaskNum0");
    //    canPorts[1].startCanRec("cantaskNum1");
    //    canPorts[2].startCanRec("cantaskNum2");
    //mjq canPorts[3].startCanRec("cantaskNum3");

    arms[0].startCtrl("maintaskNum0");
    //    arms[1].startCtrl("maintaskNum1");
    //    arms[2].startCtrl("maintaskNum2");
    arms[3].startCtrl("maintaskNum3");

//mjq    m_t265.init(1);
//mjq    m_t265.init(2);
//mjq    m_t265.init(3);
//mjq    m_t265.init(4);
//mjq    m_t265.init(5);
//mjq    m_ftsensor.init("192.168.1.120");
//mjq    m_ftsensor.start();

    m_mujoco.init("./Rsras1.xml");
    //    m_mujoco.createWindows();

}

int main()
{
    printf("start!\n");
    init(true, "192.168.1.101");

    thread t0(keyscan);
    thread t1(processt265);
    thread Maintrd(state_change);

    // init the mojuco
    mujoco_initpos_set();

    mlockall(MCL_CURRENT | MCL_FUTURE);
    int err = rt_task_create(&Top_data_Task, "Top_data", 0, 55, 0);
    if (err) {
        printf("Top_data: Failed to create Top_data rt task, code %d\n",
               errno);
    }

    rt_task_start(&Top_data_Task,TopCtrl, NULL);
    if (err) {
        printf("receivetest: Failed to start rt task, code %d\n",
               errno);
    }


    while(true)
    {
        usleep(5000);
        //        m_mujoco.mujocoSceneUpdate();
        m_mujoco.mujocoUpdate();
        //背板姿态
        m_mujoco.d->qpos[0] =  0.0;
        m_mujoco.d->qpos[1] =  0.0;
        m_mujoco.d->qpos[2] =  0.0;

        m_mujoco.d->qpos[3] =  1.0;
        m_mujoco.d->qpos[4] =  0.0;
        m_mujoco.d->qpos[5] =  0.0;
        m_mujoco.d->qpos[6] =  0.0;

        //右hunan臂位置和姿态
        m_mujoco.d->qpos[19] = joint1_r.q.w();
        m_mujoco.d->qpos[20] = joint1_r.q.x();
        m_mujoco.d->qpos[21] = joint1_r.q.y();
        m_mujoco.d->qpos[22] = joint1_r.q.z();
        m_mujoco.d->qpos[23] = joint2_r.theta;
        //右hunan臂位置和姿态
        m_mujoco.d->qpos[24] = joint1_l.q.w();
        m_mujoco.d->qpos[25] = joint1_l.q.x();
        m_mujoco.d->qpos[26] = joint1_l.q.y();
        m_mujoco.d->qpos[27] = joint1_l.q.z();
        m_mujoco.d->qpos[28] = joint2_l.theta;

        //右SRAs臂位置和姿态
        m_mujoco.d->qpos[7] =  arms[3].m_Desire.posd[0];
        m_mujoco.d->qpos[8] =  arms[3].m_Desire.posd[1];
        m_mujoco.d->qpos[9] =  arms[3].m_Desire.posd[2];

        m_mujoco.d->qpos[10] =  arms[3].m_Desire.posd[3];
        m_mujoco.d->qpos[11] =  arms[3].m_Desire.posd[4];
        m_mujoco.d->qpos[12] =  arms[3].m_Desire.posd[5];


        //左SRAs臂位置和姿态
        m_mujoco.d->qpos[13] =  arms[0].m_Desire.posd[0];
        m_mujoco.d->qpos[14] =  arms[0].m_Desire.posd[1];
        m_mujoco.d->qpos[15] =  arms[0].m_Desire.posd[2];

        m_mujoco.d->qpos[16] =  arms[0].m_Desire.posd[3];
        m_mujoco.d->qpos[17] =  arms[0].m_Desire.posd[4];
        m_mujoco.d->qpos[18] =  arms[0].m_Desire.posd[5];

        /*************************手末端位置实时获取********************************************************************/
        //        Eigen::Vector3d TipToOrigin_P, TipToWorld_P, OriginToWorld_P;
        //        Eigen::Matrix3d OriginToWorld_R, WorldToOrigin_R;
        //        Eigen::Matrix4d OriginToWorld_T, WorldToOrigin_T;

        //        Eigen::Vector4d TipToOrigin_P0,TipToWorld_P0;  //right  limb
        //        Eigen::Vector4d TipToOrigin_P1,TipToWorld_P1;  //left  limb

        //        OriginToWorld_P << m_mujoco.d->site_xpos[0], m_mujoco.d->site_xpos[1],m_mujoco.d->site_xpos[2];
        //        OriginToWorld_R << m_mujoco.d->site_xmat[0], m_mujoco.d->site_xmat[1], m_mujoco.d->site_xmat[2],
        //                m_mujoco.d->site_xmat[3], m_mujoco.d->site_xmat[4], m_mujoco.d->site_xmat[5],
        //                m_mujoco.d->site_xmat[6], m_mujoco.d->site_xmat[7], m_mujoco.d->site_xmat[8];
        //        OriginToWorld_T = PRtoT(OriginToWorld_P,OriginToWorld_R);
        //        WorldToOrigin_T = inverse(OriginToWorld_T);

        //        TipToWorld_P0 << m_mujoco.d->site_xpos[3],m_mujoco.d->site_xpos[4],m_mujoco.d->site_xpos[5],1.0;
        //        TipToOrigin_P0 = WorldToOrigin_T * TipToWorld_P0;
        //        Hand_End_P<< TipToOrigin_P0[0],TipToOrigin_P0[1],TipToOrigin_P0[2];
        //        Hand_End_V<< m_mujoco.d->sensordata[12], m_mujoco.d->sensordata[13], m_mujoco.d->sensordata[14];

        //        TipToWorld_P1 << m_mujoco.d->site_xpos[6],m_mujoco.d->site_xpos[7],m_mujoco.d->site_xpos[8],1.0;
        //        TipToOrigin_P1 = WorldToOrigin_T * TipToWorld_P1;
        //        Hand_End_P1<< TipToOrigin_P1[0],TipToOrigin_P1[1],TipToOrigin_P1[2];
        //        Hand_End_V1<< m_mujoco.d->sensordata[15], m_mujoco.d->sensordata[16], m_mujoco.d->sensordata[17];

        //        cout<<"TipToOrigin_P0:"<<Hand_End_P[2]<<endl;
        //        cout<<"TipToOrigin_P1:"<<m_mujoco.d->sensordata[2]<<endl;
        //        cout<<"TipToOrigin_p1:"<<m_mujoco.d->sensordata[5]<<endl;
        //        cout<<"TipToOrigin_p2:"<<m_mujoco.d->sensordata[11]<<endl;
        vrep.setHumanstate(cam[0].q,joint1_r.q,cam[2].q,joint2_r.theta,joint1_l.q,cam[4].q,joint2_l.theta);
        curvedata[0] =   m_support.hand_vz_mean_r;
        curvedata[1] =   m_support.hand_vz_mean_r;
        curvedata[2] =   m_mujoco.d->ncon;
        vrep.addGraphData(curvedata,3,"GraphData1");
        //        m_ftsensor.forceControl(2);
        //       cout <<"F2:"<<m_ftsensor.ft3<<endl;
    }
}
