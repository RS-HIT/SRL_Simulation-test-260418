#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "globalDef.h"
#include "ik.h"


class kinematics
{
public:
    kinematics();
    ~kinematics();
    bool loadFromFile(const char* fname);
    bool setJointPositions(const float jointpos[]);
    bool setJointPosition(int idx, const float jointpos);
    bool setTipTransform(const C7Vector transform);
    bool setTargetTransform(const C7Vector transform);

    bool getJointPositions(float jointpos[]);
    bool getJointPosition(int idx, float* jointpos);
    bool getTipTransform(C7Vector* transform);
    bool getTargetTransform(C7Vector* transform);
private:
    int m_DOFS;
    int* m_MotorHandles;
    C7Vector m_TipTransform;
    C7Vector m_TargetTransform;
    int     m_TipHandle;
    int     m_TargetHandle;
};

#endif // KINEMATICS_H
