#include "vrepsim.h"
#define PI      3.1416f
#include<iostream>
#include <Eigen/Geometry>

using namespace std;
vrepsim::vrepsim()
{
    m_Dofs = 3;
    //m_JointHandles = new int[m_Dofs];
    m_ClientID = -1;
}

bool vrepsim::connect(const char* ip)
{
    //printf("ip:%s\n",ip);
    bool ret = true;
    m_ClientID = simxStart((simxChar*)ip, 19997, true, true, 2000, 5);
//    rt_printf("m_ClientID:%d\n",m_ClientID);
    if (m_ClientID != -1)
    {
        //printf("Connected to CoppeliaSim remote API server\n");
        simxStartSimulation(m_ClientID, simx_opmode_blocking);
        //printf("simulation started\n");
    }
    else
    {
        ret = false;
    }

//    for (int i = 0; i < m_Dofs; i++)
//    {
//        char jname[64];
//        sprintf(jname, "joint%d", i + 1);
//        //printf("jname:%s\n", jname);
//        int ret = simxGetObjectHandle(m_ClientID, jname, &m_JointHandles[i], simx_opmode_blocking);
//        if (ret != simx_return_ok)
//            printf("Remote API function call returned with error code: %d\n", ret);
//    }

    return ret;
}
//*********************************************************************************************


void vrepsim::close()
{
    if (m_ClientID != -1)
    {
        simxStopSimulation(m_ClientID, simx_opmode_blocking);
        // Now close the connection to CoppeliaSim:
        simxFinish(m_ClientID);
        printf("close the connection to CoppeliaSim\n");
    }
}

//void vrepsim::setJointPositions(const VectorXd jointpos)
//{
//    float jp[m_Dofs];
//    for (int i = 0; i < jointpos.size(); i++)   jp[i] = jointpos[i];
//    simxSetStringSignal(m_ClientID,"JointPos",(unsigned char*)jp,4*m_Dofs,simx_opmode_oneshot);
//}

void vrepsim::setJointPositions(const VectorXd jointpos_d,const VectorXd jointpos_r,char* datename)
{
//    rt_printf("qqqqq\n");
    int count = jointpos_d.size()*2;
    float data[count];
    for (int i = 0; i < jointpos_d.size(); i++)
    {
        data[2*i] = jointpos_d[i];
        data[2*i+1] = jointpos_r[i];
    }
    int n = simxSetStringSignal(m_ClientID,datename,(unsigned char*)data,4*count,simx_opmode_oneshot);
//    rt_printf("n=:%d\n",n);
}

void vrepsim::setObjectPositions(const VectorXd objectpos_1,const VectorXd objectpos_2)
{
    int count = objectpos_1.size()*2;
    float data[count];
    for (int i = 0; i < objectpos_1.size(); i++)
    {
        data[i] = objectpos_1[i];
        data[i+3] = objectpos_2[i];
    }
    simxSetStringSignal(m_ClientID,"ObjectPosition",(unsigned char*)data,4*count,simx_opmode_oneshot);
}

void vrepsim::setHumanstate(Quaternionf Q_l0, Quaternionf Q_l1, Quaternionf Q_l2, double theta1, Quaternionf Q_l3, Quaternionf Q_l4,double theta2)
{
    int count = 22;
    float data[count];

    data[0] = Q_l0.x();
    data[1] = Q_l0.y();
    data[2] = Q_l0.z();
    data[3] = Q_l0.w();

    data[4] = Q_l1.x();
    data[5] = Q_l1.y();
    data[6] = Q_l1.z();
    data[7] = Q_l1.w();

    data[8] = Q_l2.x();
    data[9] = Q_l2.y();
    data[10] = Q_l2.z();
    data[11] = Q_l2.w();

    data[12] = theta1;

    data[13] =  Q_l3.x();
    data[14] = Q_l3.y();
    data[15] = Q_l3.z();
    data[16] = Q_l3.w();

    data[17] = Q_l4.x();
    data[18] = Q_l4.y();
    data[19] = Q_l4.z();
    data[20] = Q_l4.w();

    data[21] = theta2;
    simxSetStringSignal(m_ClientID,"Linkpose_r",(unsigned char*)data,4*count,simx_opmode_oneshot);
}

void vrepsim::setGraphParam(int curves, int points)
{
    int data[] = {curves,points};
    simxSetStringSignal(m_ClientID,"GraphParam",(unsigned char*)data,8,simx_opmode_oneshot_wait);
}

void vrepsim::addGraphData(float* data,int count,char* datename)
{
    simxSetStringSignal(m_ClientID,datename,(unsigned char*)data,4*count,simx_opmode_oneshot);
}


