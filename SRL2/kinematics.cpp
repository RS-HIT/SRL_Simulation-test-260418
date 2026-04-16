#include "kinematics.h"
#include <stdio.h>
#define PI  3.1416f
kinematics::kinematics()
{

}

kinematics::~kinematics()
{
    delete m_MotorHandles;
}

bool kinematics::setJointPositions(const float jointpos[])
{
    for (int i = 0; i < m_DOFS; i++)
    {
        if(!ikSetJointPosition(m_MotorHandles[i], jointpos[i])) return false;
    }
    C7Vector transform;
    getTipTransform(&transform);
    setTargetTransform(transform);
    return true;
}

bool kinematics::setJointPosition(int idx, const float jointpos)
{
    return ikSetJointPosition(m_MotorHandles[idx], jointpos);
}

bool kinematics::setTipTransform(const C7Vector transform)
{
    return ikSetObjectTransformation(m_TipHandle, -1, &transform);
}

bool kinematics::setTargetTransform(const C7Vector transform)
{
    ikSetObjectTransformation(m_TargetHandle, -1, &transform);
    return ikHandleIkGroup(ik_handle_all);
}

bool kinematics::getJointPositions(float jointpos[])
{
    for (int i = 0; i < m_DOFS; i++)
    {
        if(!ikGetJointPosition(m_MotorHandles[i], &jointpos[i])) return false;
    }
    return true;
}

bool kinematics::getJointPosition(int idx, float* jointpos)
{
    return ikGetJointPosition(m_MotorHandles[idx], jointpos);
}

bool kinematics::getTipTransform(C7Vector* transform)
{
    return ikGetObjectTransformation(m_TipHandle, -1, transform);
}

bool kinematics::getTargetTransform(C7Vector* transform)
{
    return ikGetObjectTransformation(m_TargetHandle, -1, transform);
}

bool kinematics::loadFromFile(const char* fname)
{
    // Read the exported kinematic file:
    FILE* file = NULL;
    file = fopen(fname, "rb");
    if (file)
    {
        fseek(file, 0, SEEK_END);
        unsigned long fl = ftell(file);
        int dataLength = (int)fl;
        fseek(file, 0, SEEK_SET);
        std::vector<unsigned char> data;
        data.resize(dataLength);
        fread((char*)&data[0], dataLength, 1, file);
        fclose(file);

        // Initialize the environment and import the kinematic data:
        ikCreateEnvironment();
        ikLoad(&data[0], dataLength);

        printf("ik calculation test for LBR_iiwa_7_R800, model file %s \n", fname);

        m_DOFS = 7;
        m_MotorHandles = new int[m_DOFS];
        // Get some handles from the robot model:
        for (int i = 0; i < m_DOFS; i++)
        {
            char name[64];
            sprintf(name, "LBR_iiwa_7_R800_joint%d", i + 1);
            bool ret = ikGetObjectHandle(name, &m_MotorHandles[i]);
            printf("joint%d handle=%d\n", i, m_MotorHandles[i]);
        }

        //int targetHandle;
        ikGetObjectHandle("target", &m_TargetHandle);
        printf("targetHandle=%d\n", m_TargetHandle);
        //int tipHandle;
        ikGetObjectHandle("tip", &m_TipHandle);
        printf("tipHandle=%d\n", m_TipHandle);


        //int baseHandle;
        //ikGetObjectHandle("LBR_iiwa_7_R800", &baseHandle);
        //printf("baseHandle=%d\n", baseHandle);

//        simReal ps[] = { 0,-30.0f * PI / 180.0f,0.0f,-60.0f * PI / 180.0f,0.0f,-30.0f * PI / 180.0f,0.0f };
//        setJointPositions(ps);
//        C7Vector tipTransf;
//        ikGetObjectTransformation(m_TipHandle, -1, &tipTransf);
//        ikSetObjectTransformation(m_TargetHandle, -1, &tipTransf);

//        // Get the initial target dummy transformation, of the robot:
//        C7Vector initTargetTransf;
//        ikGetObjectTransformation(m_TargetHandle, -1, &initTargetTransf);
        return true;
    }
    return false;
}
