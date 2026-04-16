#ifndef VREPSIM_H
#define VREPSIM_H
//*********************************************************************************************
//Vrep Releated

extern "C" {
#include "extApi.h"
//#include "extApiPlatform.h"
//#include "simConst.h"
//#include "simLib.h"
//#include "simTypes.h"
}
#include <stdio.h>
simxInt clientID;
//simxInt JointHandles[NUMBER_OF_DOFS];

//void Vrep_setJointPosition(simxInt idx, simReal pos)
//{
//    simxInt ret = simxSetJointPosition(clientID, JointHandles[idx], pos, simx_opmode_oneshot); // Try to retrieve the streamed data
//}

//void Vrep_setJointPositions(simReal pos[7])
//{
//    for (int i = 0; i < NUMBER_OF_DOFS; i++)
//    {
//        Vrep_setJointPosition(i, pos[i]);
//    }
//    extApi_sleepMs(CYCLE_TIME_IN_SECONDS * 1000);
//}

void Vrep_connect()
{
    clientID = simxStart((simxChar*)"192.168.10.128", 19997, true, true, 2000, 5);
    if (clientID != -1)
    {
        printf("Connected to CoppeliaSim remote API server\n");
    }
//    int objectCount;
//    int* objectHandles;
//    for (int i = 0; i < NUMBER_OF_DOFS; i++)
//    {
//        char jname[16];
//        sprintf(jname, "LBR_iiwa_7_R800_joint%d", i + 1);
//        int ret = simxGetObjectHandle(clientID, jname, &JointHandles[i], simx_opmode_blocking);
//        if (ret == simx_return_ok)
//            printf("vrep joint%d handle=%d\n", i, JointHandles[i]);
//        else
//            printf("Remote API function call returned with error code: %d\n", ret);
//    }
    simxStartSimulation(clientID, simx_opmode_blocking);
}
//*********************************************************************************************


void Vrep_close()
{
    simxStopSimulation(clientID, simx_opmode_blocking);
    // Now close the connection to CoppeliaSim:
    simxFinish(clientID);
    printf("close the connection to CoppeliaSim\n");
}
//
#endif // VREP_H
