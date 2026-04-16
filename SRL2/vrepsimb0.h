#ifndef VREPSIMB0_H
#define VREPSIMB0_H

#include "globalDef.h"
#include "b0RemoteApi.h"

class vrepsimB0
{
public:
    vrepsimB0();
    bool connect(const char* ip);
    void close();
    void setJointPositions(const float jointpos[DOFS]);
    void setJointTargetPositions(const float jointpos[DOFS]);
    void getJointPositions(float jointpos[DOFS]);
    void addGraphData(float *data,int count);
private :
    b0RemoteApi*     m_pClient;
    int m_JointHandles[DOFS];
    void getJointPositions_CB(std::vector<msgpack::object>* msg);

};
#endif // VREPSIMB0_H
