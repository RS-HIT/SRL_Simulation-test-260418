#ifndef VREPSIM_H
#define VREPSIM_H
#include "globalDef.h"
#include <Eigen/StdVector>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
using namespace Eigen;
#include <stdio.h>
extern "C" {
#include "extApi.h"
}

class vrepsim
{
public:
    vrepsim();
    bool connect(const char* ip);
    void close();
    void setJointPositions(const VectorXd jointpos_d,const VectorXd jointpos_r,char* datename);
    void setObjectPositions(const VectorXd objectpos_1,const VectorXd objectpos_2);
    void addGraphData(float* data,int count,char* datename);
    void setGraphParam(int curves, int points);
    void setHumanstate(Quaternionf Q_l0,Quaternionf Q_l1, Quaternionf Q_l2,double theta1, Quaternionf Q_l3, Quaternionf Q_l4,double theta2);
private :
    int     m_Dofs;
    simxInt m_ClientID;
};

#endif
