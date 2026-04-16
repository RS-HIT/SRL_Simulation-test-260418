#include "vrepsimb0.h"
#define PI      3.1416f

vrepsimB0::vrepsimB0()
{
    //m_ClientID = -1;
    m_pClient = NULL;
}

bool vrepsimB0::connect(const char* ip)
{
    printf("ip:%s\n",ip);
    bool ret = true;
    char b0ip[64];
    sprintf(b0ip, "B0_RESOLVER=tcp://%s:22000",ip);
    putenv( b0ip );
    std::cout << "B0_RESOLVER = " << getenv("B0_RESOLVER") << std::endl;

    m_pClient = new b0RemoteApi("b0RemoteApi_c++Client","b0RemoteApi");

    if (m_pClient != NULL)
    {
        printf("Connected to CoppeliaSim remote API server\n");
        m_pClient->simxStartSimulation(m_pClient->simxDefaultPublisher());
        printf("simulation started\n");
    }
    else
    {
        ret = false;
    }

    for(int i=0;i<DOFS;i++)
    {
        char name[64];
        sprintf(name, "LBR_iiwa_7_R800_joint%d", i + 1);
        std::vector<msgpack::object>* reply=m_pClient->simxGetObjectHandle(name,m_pClient->simxServiceCall());
        m_JointHandles[i]=b0RemoteApi::readInt(reply,1);
    }

    return ret;
}
//*********************************************************************************************


void vrepsimB0::close()
{
    if (m_pClient != NULL)
    {
        m_pClient->simxStopSimulation(m_pClient->simxDefaultPublisher());
        printf("close the connection to CoppeliaSim\n");
    }
}

void vrepsimB0::setJointPositions(const float jointpos[DOFS])
{
    for (int i = 0; i < DOFS; i++)
    {
        m_pClient->simxSetJointPosition(m_JointHandles[i], jointpos[i],m_pClient->simxDefaultPublisher());
    }
}

void vrepsimB0::setJointTargetPositions(const float jointpos[DOFS])
{
    for (int i = 0; i < DOFS; i++)
    {
        m_pClient->simxSetJointTargetPosition(m_JointHandles[i], jointpos[i],m_pClient->simxDefaultPublisher());
    }
}

void vrepsimB0::getJointPositions_CB(std::vector<msgpack::object>* msg)
{
//    std::cout << "getJointPositions_CB." << std::endl;
//    std::string img(b0RemoteApi::readFloat(msg,1));
}

void vrepsimB0::getJointPositions(float jointpos[DOFS])
{
    m_pClient->simxSpinOnce();
    for (int i = 0; i < DOFS; i++)
    {
        m_pClient->simxGetJointPosition(m_JointHandles[i], m_pClient->simxServiceCall());
    }
}


void vrepsimB0::addGraphData(float *data,int count)
{
    //m_pClient->simxSetFloatSignal("GraphData",data,m_pClient->simxDefaultPublisher());
    m_pClient->simxSetStringSignal("GraphData",(char*)data,4*count,m_pClient->simxDefaultPublisher());
}


