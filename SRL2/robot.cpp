#include "robot.h"
#include <unistd.h>
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
#include "magneticencoder.h"
#include "Dynamixel.h"
#include "mitmotor.h"
#include "rt485.h"
//ZqH
#define FILTER_A 0.2
#define i1 4.15392189
#define i2 4.15392189
#define i3 2.043649491
using namespace robot;
extern float gSpeed;
extern char cmd;
extern Eigen::Vector3d Hand_End_P;
extern ftSensor m_ftsensor;
extern int m_task_state;
extern int state_support;
enum task_state_support
{
    none = 0,
    support = 1,
    force = 2,
    back = 3,
    gblance = 4

};
void robotArm::rtCtrlProc(void* arg)
{
    robotArm* pthis = (robotArm*)arg;
    //    char*  a   =(robotArm*)arg;
    unsigned long ov;
    float rtPeroid = pthis->m_TStep*1000000000;
    rt_task_set_periodic(NULL, TM_NOW, rtPeroid);
    while(!pthis->m_StopCtrl)
    {
        pthis->ctrlStateMachine();
        rt_task_wait_period(&ov);
    }
}
//void robotArm::rtCtrlProc(robotArm* arm)
//{
////    robotArm* pthis = (robotArm*)arg;
//    unsigned long ov;
//    float rtPeroid = arm->m_TStep*1000000000;
//    rt_task_set_periodic(NULL, TM_NOW, rtPeroid);
//    while(!arm->m_StopCtrl)
//    {
//        arm->ctrlStateMachine();
//        rt_task_wait_period(&ov);
//    }
//}

struct
{
    Eigen::Vector3f    p;
    Eigen::Vector3f    pry;
    Eigen::Quaternionf q;
    Eigen::Vector3f    v;
    Eigen::Vector3f  a;
}cam;

//void robotArm::VrepGraphAddData(float curvedata[])
//{
//    static int pcount = 0;
//    pcount++;
//    for(int i=0;i<graph_curves;i++)
//    {
//        graphbuf[(pcount-1)*graph_curves+i] = curvedata[i];
//    }
//    if(pcount==graph_points)
//    {
//        pcount = 0;
//        m_vrep.addGraphData(graphbuf,graph_points*graph_curves,curvename);
//    }
//}

double robotArm::round_to_lmt(double v, double lmt_up ,double lmt_down)
{
    if(v >= lmt_up) v = lmt_up;
    if (v <= lmt_down) v = lmt_down;
    return v;
}
void robotArm::jointToMotor(VectorXd jpos,VectorXd jvel,VectorXd jtor,VectorXd &mpos,VectorXd &mvel,VectorXd &mtor)
{
    mpos[0] = i1 * jpos[0];
    mpos[1] = i2 * jpos[1] + i3 * jpos[2];
    mpos[2] = i2 * jpos[1] - i3* jpos[2];
    mpos[3] =  jpos[3];
    mpos[4] = -jpos[4];
    mpos[5] = -jpos[5];
    mpos[6] = -jpos[6];
    mvel[0] = i1* jvel[0];
    mvel[1] = i2* jvel[1] + i3* jvel[2];
    mvel[2] = i2 * jvel[1] - i3 * jvel[2];
    mvel[3] = -mvel[3];
    mvel[4] = -mvel[4];
    mvel[5] = -mvel[5];
    mvel[6] = -mvel[6];
    mtor[0] = jtor[0] / i1;
    mtor[1] = (i3 * jtor[1] + i2 * jtor[2]) / (2*i2*i3);
    mtor[2] = (i3 * jtor[1] - i2 * jtor[2]) / (2*i2*i3);
    mtor[3] = -mtor[3];
    mtor[4] = -mtor[4];
    mtor[5] = -mtor[5];
    mtor[6] = -mtor[6];
}
void robotArm::jointToMotor(VectorXd jpos,VectorXd &mpos)
{
    mpos[0] = i1 * jpos[0];
    mpos[1] = i2 * jpos[1] + i3 * jpos[2];
    mpos[2] = i2 * jpos[1] - i3* jpos[2];
}
void robotArm::motorToJoint(VectorXd mpos,VectorXd mvel,VectorXd mtor,VectorXd &jpos,VectorXd &jvel,VectorXd &jtor)
{
    jpos[0] = mpos[0] / i1;
    jpos[1] = (mpos[1] + mpos[2]) /  (2*i2);
    jpos[2] = (mpos[1] - mpos[2]) /  (2*i3);
    jvel[0] = mvel[0] / i1;
    jvel[1] = (mvel[1] + mvel[2]) / (2*i2);
    jvel[2] = (mvel[1] - mvel[2]) / (2*i3);
    jtor[0] = i1 * mtor[0];
    jtor[1] = i2 * mtor[1] + i2 * mtor[2];
    jtor[2] = i3 * mtor[1] - i3 * mtor[2];
}
void robotArm::motorToJoint(VectorXd mpos,VectorXd &jpos)
{
    jpos[0] = mpos[0] / i1;
    jpos[1] = (mpos[1] + mpos[2]) /  (2*i2);
    jpos[2] = (mpos[1] - mpos[2]) /  (2*i3);
}
//更改
void robotArm::jointMove(VectorXd pos,VectorXd vel,VectorXd tor,int type)
{
    VectorXd mpos(m_Dofs+1), mvel(m_Dofs+1), mtor(m_Dofs+1);
    jointToMotor(pos,vel,tor,mpos,mvel,mtor);
    sendMotorCmds(mpos,mvel,mtor,type);
}
void robotArm::jointMove_t(VectorXd tor)
{
    VectorXd mpos(m_Dofs+1), mvel(m_Dofs+1), mtor(m_Dofs+1);
    VectorXd pos(m_Dofs+1),vel(m_Dofs+1);
    pos = VectorXd::Zero(tor.size());
    vel = VectorXd::Zero(tor.size());
    jointToMotor(pos,vel,tor,mpos,mvel,mtor);
    float pr,vr,tr;
    for(int i=0;i<m_mitmotor_num;i++)
    {
        m_canComm->motors[i]->sendCMD_t(mtor[i]);
        m_canComm->motors[i]->getState(pr,vr,tr);
        motorpos_r[i] = pr;
        motorvel_r[i] = vr;
        motortor_r[i] = tr;
    }
}
long num   = 0;
//更改
void robotArm::sendMotorCmds(VectorXd pos,VectorXd vel,VectorXd tff,int type)
{
    float pd,vd,td;
    float pr,vr,tr;
    if(type == 1)
    {
        float fricStatic = 0.1f;
        for(int i=0;i<m_mitmotor_num;i++)
        {
            pd = pos[i] - motorpos_ini[i];
            vd = vel[i];
            td = tff[i];
            //        cout<<"td:"<<td<<endl;
            //if(firsttime)vd = 0;
            float t_fric = 0;
            if(vd>0.05f) t_fric = fricStatic;
            else if(vd<-0.05f) t_fric = -fricStatic;
            //            td = 0;//td + t_fric ;
            bool flag = fabs(pos[i])>= m_canComm ->motors[i]->cmdpos_limit;
            if(flag)
            {
                cout<<"motor:"<<  i+1 <<"over cmdpos_limit"<<endl;
            }
            else
            {
                m_canComm ->motors[i]->sendCMD(pd,vd,td,type);
                //               rt_printf("motor:%d pd=%0.3f,vd=%0.3f,td=%0.3f\n",i,pd,vd,td);
            }
            m_canComm ->motors[i]->getState(pr,vr,tr);
            motorpos_r[i] = pr;
            motorvel_r[i] = vr;
            motortor_r[i] = tr;
        }
        RTIME time_last;
        if(datename == "JointPosition1"|| datename == "JointPosition4")
        {
            for(int i=0;i<3;i++)
            {
                pd = pos[i+3];
                vd = vel[i+3];
                td = tff[i+3];
                time_last = rt_timer_read();
                m_485Comm.dynamixels[i]->setPos(pd*180.0/PI);
            }
        }
    }
    else if(type == 2)
    {
        int v=0;
    }
    else if(type == 3)
    {
        //        float pr,vr,tr;
        for(int i=0;i<m_mitmotor_num;i++)
        {
            pd = pos[i];
            vd = vel[i];
            td = tff[i];
            m_canComm ->motors[i]->protect(type);
            m_canComm ->motors[i]->sendCMD(pd,vd,td,type);
            m_canComm->motors[i]->getState(pr,vr,tr);
            motorpos_r[i] = pr;
            motorvel_r[i] = vr;
            motortor_r[i] = tr;
        }
    }
}
void robotArm::fetchMotorStates(VectorXd &p,VectorXd &v,VectorXd &t)
{
    VectorXd mp(m_mitmotor_num),mv(m_mitmotor_num),mt(m_mitmotor_num);
    float pr,vr,tr;
    for(int i=0;i<m_mitmotor_num;i++)
    {
        m_canComm ->motors[i]->fetchState(pr,vr,tr);
        //        m_canComm ->encoders[i]->
        mp[i] = pr + motorpos_ini[i];
        //        cout<<motorpos_ini[0]*180/PI<<endl;
        //rt_printf("m_canComm ->motors[i]->m_param.dir:%d\n",m_canComm ->motors[i]->m_param.dir);
        mv[i] = vr;
        mt[i] = tr;
    }
    //rt_printf("loopcount:%d mp:[%0.3f,%0.3f,%0.3f]\n",ctrlloopcount,mp[0],mp[1],mp[2]);
    motorToJoint(mp,mv,mt,p,v,t);
}
void robotArm::getMotorStates(VectorXd &p,VectorXd &v,VectorXd &t)
{
    VectorXd mp(m_mitmotor_num),mv(m_mitmotor_num),mt(m_mitmotor_num);
    for(int i=0;i<m_mitmotor_num;i++)
    {
        float pr,vr,tr;
        m_canComm ->motors[i]->getState(pr,vr,tr);
        mp[i] = pr + motorpos_ini[i];
        mv[i] = vr;
        mt[i] = tr;
    }
    motorToJoint(mp,mv,mt,p,v,t);
}
void robotArm::calcPVAfromP(VectorXd pos, MOTION_CMD& mc,bool firsttime)
{
    mc.posd = pos;
    if(!firsttime)
    {
        mc.veld = (mc.posd - mc.posd_last)/m_TStep;
        mc.accd = (mc.veld - mc.veld_last)/m_TStep;
    }
    mc.posd_last = mc.posd;
    mc.veld_last = mc.veld;
}
void robotArm::calcPVAfromP(VectorXd pos, MOTION_STATUS& mc,bool firsttime)
{
    mc.posr = pos;
    if(!firsttime)
    {
        mc.velr_c = (mc.posr - mc.posr_last)/m_TStep;
        mc.accr_c = (mc.velr_c - mc.velr_c_last)/m_TStep;
    }
    mc.posr_last = mc.posr;
    mc.velr_c_last = mc.velr_c;
}
void robotArm::calcPVAfromP(VectorXd pos, Diff_va& mc,bool firsttime,float n_TStep)
{
    mc.pos = pos;
    for(int i=0;i<3;i++)
    {
        if(!firsttime && mc.pos[i] != mc.pos_last[i])
        {
            mc.vel[i] = (mc.pos[i] - mc.pos_last[i])/m_TStep/n_TStep;
            mc.acc[i] = (mc.vel[i] - mc.vel_last[i])/m_TStep/n_TStep;
        }
    }
    mc.pos_last = mc.pos;
    mc.vel_last = mc.vel;
}
VectorXd robotArm::Filter(VectorXd value,struct FITER_STATUS *fiter) {
    fiter->value = value;
    fiter->value = (1.0-FILTER_A) * fiter->value_last + FILTER_A * fiter->value;
    fiter->value_last = fiter->value;
    return  fiter->value;
}
void robotArm::ctrlStateMachine()
{
    Eigen::Vector3d camerr;
    Eigen::Vector3d accerr;
    Eigen::Vector3d vccerr;
    //state machine
    ctrlloopcount++;
    ctrltime = ctrlloopcount * m_TStep;
    if(m_MotorsEnabled)
    {
        int error_count = 0;
        int* back;
        for(int i=0;i<3;i++)
        {
            back = m_canComm ->motors[i]->protect(1);
            bool flag = back[0] || back[1] || back[2];
            if(flag)
            {
                m_SM.action == SM_ACTION::STOP_MOTION;
                error_count++;
            }
        }
        if(error_count>0)
        {
            for(int i=0;i<m_mitmotor_num;i++)
            {
                m_canComm ->motors[i]->enableMotor(false);
            }
            if(datename == "JointPosition1")
            {
                cout<<"left limb error"<<endl;
            }
            if(datename == "JointPosition4")
            {
                cout<<"right limb error"<<endl;
            }
        }
    }
    if(m_SM.action == SM_ACTION::ENABLE_MOTORS)
    {   rt_printf("zqhzqhzqh\n");
        rt_printf("SM_ACTION::ENABLE_MOTORS\n");
        rt_printf("m_SM.action:%d,m_SM.state:%d,m_SM.motiontype:%d\n",m_SM.action,m_SM.state,m_SM.motiontype);
        m_SM.action = SM_ACTION::NONE;
        m_MotorsEnabled = true;
        m_Desire.posd = m_Real.posr;
        m_dynamics.setJointPositions(m_Desire.posd);
        //         BYTE data_init[8];
        for(int i=0;i<m_mitmotor_num;i++)
        {
            m_canComm ->motors[i]->enableMotor(true);
            //            m_canComm->motors[i]->pack_cmd( m_canComm->motors[i]->m_cmd.pcmd,0,100,2,0,data_init);
            //            m_canComm->canTx(i+1,8,data_init);
        }
        if(datename == "JointPosition1"|| datename == "JointPosition4")
        {
            for(int i=0;i<m_dynamixel_num;i++)
            {
                m_485Comm.dynamixels[i]->enableDynamixel(1);
            }
        }
        return;
    }
    if(m_SM.action == SM_ACTION::DISABLE_MOTORS)
    {
        rt_printf("SM_ACTION::DISABLE_MOTORS\n");
        m_SM.action = SM_ACTION::NONE;
        m_MotorsEnabled = false;
        for(int i=0;i<m_mitmotor_num;i++)
        {
            m_canComm ->motors[i]->enableMotor(false);
        }
        if(datename == "JointPosition1"||datename == "JointPosition4")
        {
            for(int i=0;i<m_dynamixel_num;i++)
            {
                m_485Comm.dynamixels[i]->enableDynamixel(0);
            }
        }
        return;
    }
    if(m_SM.action == SM_ACTION::BEGIN_MOTION && m_SM.state == SM_STATE::STOPPED)
    {
        rt_printf("m_SM.action:%d,m_SM.state:%d,m_SM.motiontype:%d\n",m_SM.action,m_SM.state,m_SM.motiontype);
        if(m_SM.motiontype == MOTION_TYPE::JOINT_RML)
        {
            m_RMLMotion.idx = 0;
            m_RMLMotion.substep = 1;
            m_RMLMotion.onoff = true;
            m_RMLMotion.firsttime = true;
            //            rt_printf("SM_ACTION::DISABLE_MOTORS\n");
        }
        else if(m_SM.motiontype == MOTION_TYPE::BIRDHEAD)
        {
            m_BirdHeadMotion.onoff = true;
            m_BirdHeadMotion.firsttime = true;
        }
        else if(m_SM.motiontype == MOTION_TYPE::BIRDHEAD_Rod)
        {
            m_BirdHeadMotion_Rod.onoff = true;
            m_BirdHeadMotion_Rod.firsttime = true;
        }
        else if(m_SM.motiontype == MOTION_TYPE::BIRDHEAD_Planing)
        {
            m_BirdHeadMotion_Planing.onoff = true;
            m_BirdHeadMotion_Planing.firsttime = true;
        }
        else if(m_SM.motiontype == MOTION_TYPE::G_Blance)
        {
            m_G_Blance.onoff = true;
            m_G_Blance.firsttime = true;
        }
        else if(m_SM.motiontype == MOTION_TYPE::JOINT_Planing)
        {
            m_Joint_Planing.onoff = true;
            m_Joint_Planing.firsttime1 = true;
            m_Joint_Planing.firsttime2 = true;
        }
        else if(m_SM.motiontype == MOTION_TYPE::GRIPPER_Open)
        {
            m_Gripper_Open.onoff = true;
            m_Gripper_Open.firsttime = true;
        }
        else if(m_SM.motiontype == MOTION_TYPE::GRIPPER_Close)
        {
            m_Gripper_Close.onoff = true;
            m_Gripper_Close.firsttime = true;
        }
        else if(m_SM.motiontype == MOTION_TYPE::JOINT_Error)
        {
            m_Joint_Error.onoff = true;
            m_Joint_Error.firsttime = true;
        }
        else if(m_SM.motiontype == MOTION_TYPE::Identify)
        {
            m_Identify.onoff = true;
            m_Identify.firsttime = true;
        }
        else if(m_SM.motiontype == MOTION_TYPE::SUPPORT)
        {
            m_Support.onoff = true;
            m_Support.firsttime = true;
        }
        //***************Hongwei***************
        else if(m_SM.motiontype == MOTION_TYPE::G_Blance_Xuangua)
        {
            m_Xuangua.onoff = true;
            m_Xuangua.firsttime = true;
        }
        m_SM.action = SM_ACTION::NONE;
        m_SM.state = SM_STATE::RUNNING;
        //m_dynamics.setJointPositions(m_Real.posr);
    }
    if(m_SM.action == SM_ACTION::STOP_MOTION && m_SM.state == SM_STATE::RUNNING)
    {
        m_SM.action = SM_ACTION::NONE;
        m_SM.state = SM_STATE::STOPPED;
        m_RMLMotion.onoff = false;
        m_BirdHeadMotion.onoff = false;
        m_BirdHeadMotion_Rod.onoff = false;
        m_BirdHeadMotion_Planing.onoff = false;
        m_G_Blance.onoff =false;
        m_Joint_Planing.onoff = false;
        m_Gripper_Open.onoff = false;
        m_Gripper_Close.onoff = false;
        m_Joint_Error.onoff = false;
        m_Identify.onoff = false;
        m_Support.onoff = false;
        //****************jing****************
        m_Xuangua.onoff = false;
        rt_printf("motion stopped,m_SM.action:%d,m_SM.state:%d\n",m_SM.action,m_SM.state);
    }
    if(m_SM.state == SM_STATE::RUNNING)
    {
        //        rt_printf("enter the SM_STATE::RUNNING\n");
        //RML MOTION
        if(m_RMLMotion.onoff == true)
        {
            //            rt_printf("enter the rml\n");
            double theta;
            Math::Vector3d axs;
            if(m_RMLMotion.substep == 1)
            {
                rt_printf("rmlmotion pathpoint[%d:%d] \n",m_RMLMotion.ps.size(),m_RMLMotion.idx);
                if(m_RMLMotion.idx == m_RMLMotion.ps.size())
                {
                    m_RMLMotion.substep = 4;
                }
                else
                {
                    m_RMLMotion.jointPos0 = VectorXd(m_Dofs);
                    m_dynamics.getJointPositions(m_RMLMotion.jointPos0);
                    m_dynamics.getEndPose(m_RMLMotion.pos0,m_RMLMotion.mat0);
                    m_RMLMotion.substep = 2;
                    //collect date
                    for(int i=0;i<3;i++)
                    {
                        joint_date[i].mpos_r.clear();
                        joint_date[i].mvel_r.clear();
                        joint_date[i].macc_r.clear();
                        joint_date[i].mtor_r.clear();
                        joint_date[i].mpos_d.clear();
                        joint_date[i].mvel_d.clear();
                        joint_date[i].macc_d.clear();
                        joint_date[i].mtor_d.clear();
                        joint_date[i].jpos_r.clear();
                        joint_date[i].jvel_r.clear();
                        joint_date[i].jacc_r.clear();
                        joint_date[i].jtor_r.clear();
                        joint_date[i].jpos_d.clear();
                        joint_date[i].jvel_d.clear();
                        joint_date[i].jacc_d.clear();
                        joint_date[i].jtor_d.clear();
                        joint_date[i].cjpos.clear();
                        joint_date[i].cjvel.clear();
                        joint_date[i].cjacc.clear();
                        joint_date[i].cjtor.clear();
                    }
                    t.clear();
                    float pr,vr,tr;
                    for(int i=0;i<3;i++)
                    {
                        m_canComm ->canTx(i+7,0,NULL);
                        Jc_Real.pos_last[i] = m_canComm->encoders[i]->pos;
                        Jc_Real.vel_last[i] = 0;
                        m_canComm ->motors[i]->getState(pr,vr,tr);
                        Mc_Real.pos_last[i] = pr - m_canComm->motors[i]->m_param.zeroPos;
                        Mc_Real.vel_last[i] = vr;
                    }
                    m_RMLMotion.t=0;
                }
            }
            else if(m_RMLMotion.substep == 2)
            {
                //rt_printf("rmlmotion pathpoint[%d:%d] \n",m_RMLMotion.ps.size(),m_RMLMotion.idx);
                //loop 0 for each path rmlpoint
                RMLMotionPoint rmlPoint = m_RMLMotion.ps[m_RMLMotion.idx];
                VectorXd tp = rmlPoint.pos;
                //                VectorXd cp = VectorXd::Zero(tp.size());
                if(rmlPoint.type == JOINT_RML)
                {
                    VectorXd cp(tp.size());
                    VectorXd cp_arm(m_Dofs);
                    m_dynamics.getJointPositions(cp_arm);
                    //                    m_485Comm.dynamixels[3]->getPos(pos_gripper);
                    //                    VectorXd cp(cp_arm[0],cp_arm[1],cp_arm[2],cp_arm[3],cp_arm[4],cp_arm[5],0.5);
                    cp<< cp_arm[0],cp_arm[1],cp_arm[2],cp_arm[3],cp_arm[4],cp_arm[5],gripperpos_last;
                    VectorXd tp = rmlPoint.pos;
                    m_motionPlan.init(tp.size(), m_TStep);
                    m_motionPlan.rmlPos(cp,tp,rmlPoint.vel,rmlPoint.acc);
                    //                    cout << "cp111:" << cp.transpose() << "tp:" << tp.transpose() << endl;
                }
                else if(rmlPoint.type == CARTESIAN_RML_6D)
                {
                    Math::Matrix3d mat,mat0,mat0_t,mat1;
                    Math::Vector3d pos;
                    VectorXd tp;
                    VectorXd cp = VectorXd::Zero(4);
                    m_dynamics.getEndPose(pos,mat0);
                    mat1 = m_dynamics.rpyToMatrix({rmlPoint.pos[3], rmlPoint.pos[4], rmlPoint.pos[5]});
                    //                    mat =mat1*mat0_t;
                    mat =mat0.transpose()*mat1;
                    //                      mat =mat0_t*mat1;
                    theta = acos((mat(0,0)+mat(1,1)+mat(2,2)-1)/2);
                    double axs_x = (mat(2,1)-mat(1,2))/2*sin(theta);
                    double axs_y = (mat(0,2)-mat(2,0))/2*sin(theta);
                    double axs_z = (mat(1,0)-mat(0,1))/2*sin(theta);
                    double axs_norm = sqrt(pow(axs_x,2)+pow(axs_y,2)+pow(axs_z,2));
                    axs << axs_x/axs_norm,axs_y/axs_norm,axs_z/axs_norm;
                    cp = Math::Vector4d(pos[0],pos[1],pos[2],0);
                    tp = Math::Vector4d(rmlPoint.pos[0],rmlPoint.pos[1],rmlPoint.pos[2],theta);
                    //                      tp<<rmlPoint.pos[0],rmlPoint.pos[1],rmlPoint.pos[2],theta;
                    m_motionPlan.init(tp.size(), m_TStep);
                    m_motionPlan.rmlPos(cp,tp,rmlPoint.vel,rmlPoint.acc);
                }
                m_RMLMotion.substep = 3;
            }
            else if(m_RMLMotion.substep == 3)
            {
                //rt_printf("rmlmotion pathpoint[%d:%d] \n",m_RMLMotion.ps.size(),m_RMLMotion.idx);
                RMLMotionPoint rmlPoint = m_RMLMotion.ps[m_RMLMotion.idx];
                VectorXd p,v,a;
                VectorXd q(m_Dofs +1);
                VectorXd vel(m_Dofs +1);
                VectorXd tor(m_Dofs +1);
                Math::Vector3d pos;
                Math::Matrix3d mat;
                Math::Matrix3d mat_c;
                VectorXd q_inv(m_Dofs);
                //                VectorXd q_update(m_Dofs);
                if(rmlPoint.type == JOINT_RML)
                {
                    p.resize(m_Dofs+1);
                    v.resize(m_Dofs+1);
                    a.resize(m_Dofs+1);
                }
                else if(rmlPoint.type == CARTESIAN_RML_6D)
                {
                    p.resize(6);
                    v.resize(6);
                    a.resize(6);
                }
                if(!m_motionPlan.rmlStep(p,v,a))
                {
                    if(rmlPoint.type == JOINT_RML)
                    {
                        q = p;
                        //
                    }
                    else if(rmlPoint.type == CARTESIAN_RML_6D)
                    {
                        theta = p[3];
                        mat_c(0,0)=pow(axs[0],2)*(1-cos(theta)) + cos(theta);
                        mat_c(0,1)=axs[0]*axs[1]*(1-cos(theta)) - axs[2]*sin(theta);
                        mat_c(0,2)=axs[0]*axs[2]*(1-cos(theta)) + axs[1]*sin(theta);
                        mat_c(1,0)=axs[0]*axs[1]*(1-cos(theta)) + axs[2]*sin(theta);
                        mat_c(1,1)=pow(axs[1],2)*(1-cos(theta)) + cos(theta);
                        mat_c(1,2)=axs[1]*axs[2]*(1-cos(theta)) - axs[0]*sin(theta);
                        mat_c(2,0)=axs[0]*axs[2]*(1-cos(theta)) - axs[1]*sin(theta);
                        mat_c(2,1)=axs[1]*axs[2]*(1-cos(theta)) + axs[0]*sin(theta);
                        mat_c(2,2)=pow(axs[2],2)*(1-cos(theta)) + cos(theta);
                        pos = Math::Vector3d(p[0],p[1],p[2]);
                        //                        mat =mat_c*m_RMLMotion.mat0;
                        mat = m_RMLMotion.mat0*mat_c;
                        //                        mat =m_RMLMotion.mat0*mat_c;
                        if(theta ==0)
                        {
                            mat = m_RMLMotion.mat0;
                        }
                        m_dynamics.inverseKinematics(pos,mat,q_inv);
                        q<<q_inv[0],q_inv[1],q_inv[2],q_inv[3],q_inv[4],q_inv[5],gripperpos_last;
                    }
                    q_inv << q[0],q[1],q[2],q[3],q[4],q[5];
                    //                    for(int i=0;i<6;i++)
                    //                    {
                    //                        if((q_inv[i]- m_RMLMotion.q_inv_last[i])>=10*PI/180)
                    //                        {
                    //                            cout <<"inv over limit"<<endl;
                    //                            q_inv[i] = m_RMLMotion.q_inv_last[i];
                    //                        }
                    //                        m_RMLMotion.q_inv_last[i] = q_inv[i];
                    //                    }
                    VectorNd limit_up = VectorNd::Zero(6);
                    VectorNd limit_down = VectorNd::Zero(6);
                    if(datename == "JointPosition1")
                    {
                        limit_up   << -j1_limit1,j2_limit2,j3_limit2,j4_limit2,j5_limit2,j6_limit2;
                        limit_down   << -j1_limit2,j2_limit1,j3_limit1,j4_limit1,j5_limit1,j6_limit1;
                    }
                    if(datename == "JointPosition4")
                    {
                        limit_up   << j1_limit2,j2_limit2,j3_limit2,j4_limit2,j5_limit2,j6_limit2;
                        limit_down   << j1_limit1,j2_limit1,j3_limit1,j4_limit1,j5_limit1,j6_limit1;
                    }
                    for(int i=0;i<6;i++)
                    {
                        q_inv[i] = round_to_lmt(q_inv[i],limit_up[i],limit_down[i]);
                    }
                    calcPVAfromP(q_inv,m_Desire,m_RMLMotion.firsttime);
                    //                    cout<< "q:\n"<<q[6] <<endl;
                    m_RMLMotion.firsttime = false;
                    m_dynamics.setJointPositions(q_inv);
                    gripperpos_last = q[6];
                    //让动力学只计算重力项
                    VectorXd vel_i(6);
                    VectorXd acc_i(6);
                    vel_i<<0,0,0,0,0,0;
                    vel_i<<0,0,0,0,0,0;
                    m_dynamics.inverseDynamics(m_Desire.posd,vel_i,acc_i,m_Desire.tord);
                    q <<m_Desire.posd[0],m_Desire.posd[1],m_Desire.posd[2],m_Desire.posd[3],m_Desire.posd[4],m_Desire.posd[5],q[6];
                    vel <<m_Desire.veld[0],m_Desire.veld[1],m_Desire.veld[2],m_Desire.veld[3],m_Desire.veld[4],m_Desire.veld[5],0;
                    tor <<m_Desire.tord[0],m_Desire.tord[1],m_Desire.tord[2],m_Desire.tord[3],m_Desire.tord[4],m_Desire.tord[5],0;
                    jointMove(q,vel,tor,POSMODE);
                    m_RMLMotion.t = m_RMLMotion.t + m_TStep;
                    jointToMotor(q,vel,tor,mpos_d,mvel_d,mtor_d);
                    for(int i=0;i<m_mitmotor_num;i++)
                    {
                        mpos_d[i] = mpos_d[i] - motorpos_ini[i];
                    }
                    //                    for(int i=0;i<3;i++)
                    //                    {
                    //                        m_canComm->canTx(i+0x607,0,NULL);
                    //                        epos[i] = (m_canComm->encoders[i]->pos)*PI/180;
                    //                    }
                    //epos_f =Filter(epos,&fiter);
                    calcPVAfromP(epos,Jc_Real,false,1);
                    for(int i=0;i<3;i++)
                    {
                        joint_date[i].mpos_r.push_back(motorpos_r[i]);
                        joint_date[i].mvel_r.push_back(motorvel_r[i]);
                        joint_date[i].macc_r.push_back(Mc_Real.acc[i]);
                        joint_date[i].mtor_r.push_back(motortor_r[i]);
                        joint_date[i].mpos_d.push_back(mpos_d[i]);
                        joint_date[i].mvel_d.push_back(mvel_d[i]);
                        joint_date[i].mtor_d.push_back(mtor_d[i]);  //动力学方程得关节期望力矩,jointToMotor得电机期望力矩
                        joint_date[i].jpos_r.push_back(Jc_Real.pos[i]);
                        joint_date[i].jvel_r.push_back(Jc_Real.vel[i]);
                        joint_date[i].jacc_r.push_back(Jc_Real.acc[i]);
                        joint_date[i].jpos_d.push_back(m_Desire.posd[i]);
                        //                    joint_data[i].jvel_d.push_back(m_Desire.veld[i]);
                        joint_date[i].jvel_d.push_back(m_Desire.accd[i]);
                        joint_date[i].jacc_d.push_back(m_Desire.accd[i]);
                        joint_date[i].jtor_d.push_back(m_Desire.tord[i]);
                        joint_date[i].cjpos.push_back(m_Real.posr[i]);
                        joint_date[i].cjvel.push_back(m_Real.velr[i]);
                        joint_date[i].cjtor.push_back(m_Real.torr[i]);
                    }
                    joint_date[0].macc_d.push_back(i1* joint_date[0].jacc_d.back());
                    joint_date[1].macc_d.push_back(i2* joint_date[1].jacc_d.back() + i3* joint_date[1].jacc_d.back());
                    joint_date[2].macc_d.push_back(-(i2 * joint_date[2].jacc_d.back() - i3 * joint_date[2].jacc_d.back()));
                    joint_date[0].jtor_r.push_back(i1 * motortor_r[0]);
                    joint_date[1].jtor_r.push_back(i2 * motortor_r[1] + i2 * motortor_r[2]);
                    joint_date[2].jtor_r.push_back(i3 * motortor_r[1] - i3 * motortor_r[2]);
                    joint_date[0].cjacc.push_back(joint_date[0].macc_r.back() / i1);
                    joint_date[1].cjacc.push_back((joint_date[1].macc_r.back() + joint_date[2].macc_r.back()) / (2*i2));
                    joint_date[2].cjacc.push_back((joint_date[1].macc_r.back() - joint_date[2].macc_r.back()) / (2*i3));
                    t.push_back(m_RMLMotion.t);
                }
                else
                {
                    m_RMLMotion.idx++;
                    m_RMLMotion.substep = 1;
                }
            }
            else if(m_RMLMotion.substep == 4)
            {
                m_RMLMotion.substep = 0;
                m_RMLMotion.onoff = false;
                m_SM.state = SM_STATE::STOPPED;
                rt_printf("rmlmotion finished\n");
            }
        }
        //BirdHead MOTION
        if(m_BirdHeadMotion.onoff == true)
        {
            if(m_BirdHeadMotion.firsttime)
            {
                Eigen::Vector3f    cam_p;
                Eigen::Vector3f    cam_pry;
                Eigen::Quaternionf cam_q;
                Eigen::Vector3f    cam_v;
                Eigen::Vector3f    cam_a;
            }
            else
            {

            }
        }
        //BirdHead_rod MOTION
        if(m_BirdHeadMotion_Rod.onoff == true)
        {
            if(m_BirdHeadMotion_Rod.firsttime)
            {

            }
            else
            {

            }
        }
        //BirdHead_planing MOTION
        if(m_BirdHeadMotion_Planing.onoff == true)
        {
            if(m_BirdHeadMotion_Planing.firsttime)
            {
                rt_printf("进入第一次规划程序\n");
                m_BirdHeadMotion_Planing.firsttime =false;
                Math::Vector3d pos_end,rpy_end;
                m_dynamics.getEndPose(pos_end,rpy_end);
                m_BirdHeadMotion_Planing.rob_p0 = pos_end;
                m_BirdHeadMotion_Planing.rob_o0 =rpy_end;
                m_BirdHeadMotion_Planing.rob_pd = pos_end;
                m_BirdHeadMotion_Planing.x =0;
                m_BirdHeadMotion_Planing.theta =0;
            }
            else
            {
                //  BirdHead_planing for sin(x)
                float x_limit = 0.2;
                float z_limit = 0.05;
                float r_circle = 0.1;
                float w = 2*PI/x_limit;
                m_BirdHeadMotion_Planing.theta = m_BirdHeadMotion_Planing.theta + m_BirdHeadMotion_Planing.deta_theta;
                m_BirdHeadMotion_Planing.x = r_circle*sin(m_BirdHeadMotion_Planing.theta);
                m_BirdHeadMotion_Planing.z = r_circle*cos(m_BirdHeadMotion_Planing.theta) -r_circle;

                m_BirdHeadMotion_Planing.y1 = -r_circle*cos(m_BirdHeadMotion_Planing.theta) +r_circle;
                m_BirdHeadMotion_Planing.z1 = r_circle*sin(m_BirdHeadMotion_Planing.theta);

                Math::Vector3d pos_end(m_BirdHeadMotion_Planing.rob_pd);
                Math::Vector3d rpy_end(m_BirdHeadMotion_Planing.rob_o0);
                VectorXd q(m_Dofs);
                VectorXd pos(m_Dofs+1);
                VectorXd vel(m_Dofs+1);
                VectorXd tor(m_Dofs+1);
                m_dynamics.inverseKinematics(pos_end,rpy_end,q);
                m_dynamics.setJointPositions(q);
                calcPVAfromP(q,m_Desire,false);
                m_dynamics.inverseDynamics(m_Desire.posd,m_Desire.veld,m_Desire.accd,m_Desire.tord);
                pos <<m_Desire.posd[0],m_Desire.posd[1],m_Desire.posd[2],m_Desire.posd[3],m_Desire.posd[4],m_Desire.posd[5],gripperpos_last;
                vel <<m_Desire.veld[0],m_Desire.veld[1],m_Desire.veld[2],m_Desire.veld[3],m_Desire.veld[4],m_Desire.veld[5],0;
                tor <<m_Desire.tord[0],m_Desire.tord[1],m_Desire.tord[2],m_Desire.tord[3],m_Desire.tord[4],m_Desire.tord[5],0;
                jointMove(pos,vel,tor,POSMODE);
                if(datename == "JointPosition1")
                {
                    m_BirdHeadMotion_Planing.rob_pd[1]  = m_BirdHeadMotion_Planing.rob_p0[1] + m_BirdHeadMotion_Planing.y1 ;
                    m_BirdHeadMotion_Planing.rob_pd[2]  = m_BirdHeadMotion_Planing.rob_p0[2] - m_BirdHeadMotion_Planing.z1;
                }
                if(datename == "JointPosition4")
                {
                    m_BirdHeadMotion_Planing.rob_pd[0]  = m_BirdHeadMotion_Planing.rob_p0[0] + m_BirdHeadMotion_Planing.x ;
                    m_BirdHeadMotion_Planing.rob_pd[2]  = m_BirdHeadMotion_Planing.rob_p0[2] + m_BirdHeadMotion_Planing.z;
                }
            }
        }
        //g_blance MOTION
        if(m_G_Blance.onoff == true)
        {
            if(m_G_Blance.firsttime)
            {
                //模式提示
                if (m_task_state==0) //辅助搬运
                {
                    cout<<"辅助搬运力控"<<endl;
                }
                else if(m_task_state==1) //过顶支撑
                {
                    cout<<"过顶支撑力控"<<endl;
                }
                else if(m_task_state==2) //销孔装配
                {
                    cout<<"销孔装配力控"<<endl;
                }
                else if(m_task_state==3) //悬挂作业
                {
                    cout<<"悬挂作业力控"<<endl;
                }
                else if(m_task_state==4) //焊接开关
                {
                    cout<<"焊接开关力控"<<endl;
                }
                VectorXd jp(m_Dofs);
                m_dynamics.getJointPositions(jp);
                m_Desire.posd.tail(3) << jp[3], jp[4], jp[5];//For xianlan
                m_G_Blance.firsttime =false;
            }
            else
            {
                m_Desire.posd.head(3) = m_Real.posr.head(3);
                /*************************关节角度保护*******************************/
                VectorXd limit_up = VectorXd::Zero(6);
                VectorXd limit_down = VectorXd::Zero(6);
                BYTE data[8];
                if(datename == "JointPosition1")
                {
                    limit_up   << -j1_limit1,j2_limit2,j3_limit2,j4_limit2,j5_limit2,j6_limit2;
                    limit_down   << -j1_limit2,j2_limit1,j3_limit1,j4_limit1,j5_limit1,j6_limit1;
                    for(int i=0;i<m_mitmotor_num;i++)
                    {
                        if(m_Desire.posd[i]>= limit_up[i] || m_Desire.posd[i]<= limit_down[i] )
                        {
                            cout<<"joint"<< i+1 <<"over joint limit"<<endl;
                            for(int i=0;i<3;i++)
                            {
                                m_canComm ->motors[i]->enableMotor(false);
                            }
                        }
                    }
                }
                if(datename == "JointPosition4")
                {
                    limit_up   << j1_limit2,j2_limit2,j3_limit2,j4_limit2,j5_limit2,j6_limit2;
                    limit_down   << j1_limit1,j2_limit1,j3_limit1,j4_limit1,j5_limit1,j6_limit1;
                    for(int i=0;i<m_mitmotor_num;i++)
                    {
                        if(m_Desire.posd[i]>= limit_up[i] || m_Desire.posd[i]<= limit_down[i] )
                        {
                            cout<<"joint"<< i+1 <<"over joint limit"<<endl;
                            for(int i=0;i<3;i++)
                            {
                                m_canComm ->motors[i]->enableMotor(false);
                            }
                        }
                    }
                }
                m_dynamics.setJointPositions(m_Desire.posd);
                m_G_Blance.posr << m_Real.posr[0],m_Real.posr[1],m_Real.posr[2], 0.0, 0.0, 0.0;
                m_G_Blance.torr_G  << 0.0, cos(m_Real.posr[1])*2.5+cos(m_Real.posr[1]+m_Real.posr[2])*0.9, cos(m_Real.posr[1]+m_Real.posr[2])*0.9,0.0,0.0,0.0;

                m_G_Blance.torr[1] = /*m_Real.torr_coupFric[1]+*/m_G_Blance.torr_G[1];
                m_G_Blance.torr[2] = /*m_Real.torr_coupFric[2]+*/m_G_Blance.torr_G[2];
                VectorXd pos_j(7);
                VectorXd vel_j(7);
                VectorXd tor_j(7);
                pos_j<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                vel_j<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                tor_j<<m_G_Blance.torr[0],m_G_Blance.torr[1],m_G_Blance.torr[2],0.0 ,0.0 ,0.0, 0.0;
                jointMove(pos_j,vel_j,tor_j,TORMODE);

                if(cmd=='l')
                {
                    rt_printf("leave the  gblance mode\n");
                    m_SM.action = SM_ACTION::NONE;
                    m_SM.state = SM_STATE::STOPPED;
                    m_G_Blance.onoff = false;
                }

            }
        }

        //joint_planing MOTION
        if(m_Joint_Planing.onoff == true)
        {
            if(m_Joint_Planing.firsttime1)
            {
                m_Joint_Planing.t =0;
                for(int i=0;i<3;i++)
                {
                    joint_date[i].mpos_r.clear();
                    joint_date[i].mvel_r.clear();
                    joint_date[i].macc_r.clear();
                    joint_date[i].mtor_r.clear();
                    joint_date[i].mpos_d.clear();
                    joint_date[i].mvel_d.clear();
                    joint_date[i].macc_d.clear();
                    joint_date[i].mtor_d.clear();
                    joint_date[i].jpos_r.clear();
                    joint_date[i].jvel_r.clear();
                    joint_date[i].jacc_r.clear();
                    joint_date[i].jtor_r.clear();
                    joint_date[i].jpos_d.clear();
                    joint_date[i].jvel_d.clear();
                    joint_date[i].jacc_d.clear();
                    joint_date[i].jtor_d.clear();
                    joint_date[i].cjpos.clear();
                    joint_date[i].cjvel.clear();
                    joint_date[i].cjacc.clear();
                    joint_date[i].cjtor.clear();
                }
                t.clear();
                int N_maf = 100;
                bool flag_maf = 0;
                float pr,vr,tr;
                for(int i=0;i<3;i++)
                {
                    m_canComm ->canTx(i+607,0,NULL);
                    Jc_Real.pos_last[i] = (m_canComm->encoders[i]->pos)*PI/180;
                    Jc_Real.vel_last[i] = 0;
                    m_canComm ->motors[i]->getState(pr,vr,tr);
                    Mc_Real.pos_last[i] = pr - m_canComm->motors[i]->m_param.zeroPos;
                    Mc_Real.vel_last[i] = vr;
                }
                rt_printf("进入第一次整体辨识程序\n");
                m_Joint_Planing.firsttime1 =false;
                //                if(p[2]-(-PI / 4.0f)==0)
                //                {
                //                    rt_printf("进入第一次整体辨识程序\n");
                //                    m_Joint_Planing.firsttime1 =false;
                //                }
            }
            else
            {
                //                rt_printf("进入整体辨识主程序\n");
                VectorXd q(m_Dofs);
                VectorXd pos(m_Dofs+1);
                VectorXd vel(m_Dofs+1);
                VectorXd tor(m_Dofs+1);
                float theta[3];
                double a0 = -PI/10;
                double a1 = PI/10;
                double a2 = -PI/10;
                double b0 = -PI/10;
                double b1 = PI/10;
                double b2 = -PI/10;
                float n = 2;
                //                double w_base0 = 0.9;
                //                double w_base0 = 0.5;
                double w_base0 = 0.1;
                //                double w_base1 = 0.9;
                //                double w_base1 = 0.5;
                double w_base1 = 0.1;
                //                double w_base2 = 1.8;
                //                double w_base2 = 1.0;
                double w_base2 = 0.2;
                double w_j0 = 2*PI*w_base0;
                double w_j1 = 2*PI*w_base1;
                double w_j2 = 2*PI*w_base2;
                double T_j0 = 1/n/w_base0;
                double T_j1 = 1/n/w_base1;
                double T_j2 = 1/n/w_base2;
                theta[0]  = a0/w_j0*sin(n*w_j0* m_Joint_Planing.t)-b0/w_j0*cos(n*w_j0* m_Joint_Planing.t)+b0/w_j0;
                theta[1]  = a1/w_j1*sin(n*w_j1* m_Joint_Planing.t)-b1/w_j1*cos(n*w_j1* m_Joint_Planing.t)+b1/w_j1+PI/20;
                theta[2]  = a2/w_j2*sin(n*w_j2* m_Joint_Planing.t)-b2/w_j2*cos(n*w_j2* m_Joint_Planing.t)+b2/w_j2-PI/4;
                //                theta[0]  = -b0/w_j0*cos(0.5*w_j0* m_Joint_Planing.t)+b0/w_j0;
                //                theta[1]  = -b1/w_j1*cos(0.5*w_j1* m_Joint_Planing.t)+b1/w_j1+PI/20;
                //                theta[2]  = -b2/w_j2*cos(0.5*w_j2* m_Joint_Planing.t)+b2/w_j2-PI/4;
                m_Joint_Planing.t_out =  m_Joint_Planing.t;
                m_Joint_Planing.t = m_Joint_Planing.t + m_TStep;
                //                cout << "最大速度："<<a0/w_j0* w_j0<<endl;
                q <<theta[0],theta[1],theta[2],0.0,0.0,0.0;
                //                q <<theta[0],0.0,0.0,0.0,0.0,0.0;
                m_dynamics.setJointPositions(q);
                calcPVAfromP(q,m_Desire,false);
                m_dynamics.inverseDynamics(m_Desire.posd,m_Desire.veld,m_Desire.accd,m_Desire.tord);
                pos <<m_Desire.posd[0],m_Desire.posd[1],m_Desire.posd[2],m_Desire.posd[3],m_Desire.posd[4],m_Desire.posd[5],gripperpos_last;
                vel <<m_Desire.veld[0],m_Desire.veld[1],m_Desire.veld[2],m_Desire.veld[3],m_Desire.veld[4],m_Desire.veld[5],0;
                tor <<m_Desire.tord[0],m_Desire.tord[1],m_Desire.tord[2],m_Desire.tord[3],m_Desire.tord[4],m_Desire.tord[5],0;
                jointMove(pos,vel,tor,POSMODE);
                if(m_Joint_Planing.t > 6*T_j0)
                {
                    //                            m_Joint_Planing.jointerr_flag = 1;
                    m_Joint_Planing.t =0;
                    m_SM.action = SM_ACTION::NONE;
                    m_SM.state = SM_STATE::STOPPED;
                    m_Joint_Planing.onoff = false;
                }
                VectorXd    mtor_r_f;
                VectorXd    epos_f;
                VectorXd    mpos_d;
                mpos_d = VectorXd::Zero(m_Dofs+1);
                VectorXd    mvel_d;
                mvel_d = VectorXd::Zero(m_Dofs+1);
                VectorXd    mtor_d;
                mtor_d = VectorXd::Zero(m_Dofs+1);
                if(ctrlloopcount % 1== 0)
                {
                    for(int i=0;i<3;i++)
                    {
                        epos[i] = (m_canComm->encoders[i]->pos)*PI/180;
                    }
                    //epos_f =Filter(epos,&fiter);
                    calcPVAfromP(epos,Jc_Real,false,1);
                }
                //mtor_r_f = Filter(motortor_r,&fiter);
                calcPVAfromP(motorpos_r,Mc_Real,false,1);
                jointToMotor(pos,vel,tor,mpos_d,mvel_d,mtor_d);
                for(int i=0;i<3;i++)
                {
                    joint_date[i].mpos_r.push_back(motorpos_r[i]);
                    //                    joint_date[i].mpos_r.push_back(motorvel_r[i]);
                    joint_date[i].mvel_r.push_back(motorvel_r[i]);
                    //                    joint_date[i].mvel_r.push_back(Mc_Real.vel[i]);
                    joint_date[i].macc_r.push_back(Mc_Real.acc[i]);
                    joint_date[i].mtor_r.push_back(motortor_r[i]);
                    joint_date[i].mpos_d.push_back(mpos_d[i]);
                    joint_date[i].mvel_d.push_back(mvel_d[i]);
                    joint_date[i].mtor_d.push_back(mtor_d[i]);  //动力学方程得关节期望力矩,jointToMotor得电机期望力矩
                    //                    joint_date[i].epos.push_back(m_canComm ->encoders[i]->pos);
                    joint_date[i].jpos_r.push_back(Jc_Real.pos[i]);
                    joint_date[i].jvel_r.push_back(Jc_Real.vel[i]);
                    joint_date[i].jacc_r.push_back(Jc_Real.acc[i]);
                    joint_date[i].jpos_d.push_back(m_Desire.posd[i]);
                    joint_date[i].jvel_d.push_back(m_Desire.veld[i]);
                    joint_date[i].jacc_d.push_back(m_Desire.accd[i]);
                    joint_date[i].jtor_d.push_back(m_Desire.tord[i]);
                    joint_date[i].cjpos.push_back(m_Real.posr[i]);
                    joint_date[i].cjvel.push_back(m_Real.velr[i]);
                    joint_date[i].cjtor.push_back(m_Real.torr[i]);
                }
                joint_date[0].macc_d.push_back(i1* joint_date[0].jacc_d.back());
                joint_date[1].macc_d.push_back(i2* joint_date[1].jacc_d.back() + i3* joint_date[1].jacc_d.back());
                joint_date[2].macc_d.push_back(-(i2 * joint_date[2].jacc_d.back() - i3 * joint_date[2].jacc_d.back()));
                joint_date[0].jtor_r.push_back(i1 * motortor_r[0]);
                joint_date[1].jtor_r.push_back(i2 * motortor_r[1] + i2 * motortor_r[2]);
                joint_date[2].jtor_r.push_back(i3 * motortor_r[1] - i3 * motortor_r[2]);
                joint_date[0].cjacc.push_back(joint_date[0].macc_r.back() / i1);
                joint_date[1].cjacc.push_back((joint_date[1].macc_r.back() + joint_date[2].macc_r.back()) / (2*i2));
                joint_date[2].cjacc.push_back((joint_date[1].macc_r.back() - joint_date[2].macc_r.back()) / (2*i3));
                t.push_back(m_Joint_Planing.t_out);
            }
        }
        //gripper open
        if(m_Gripper_Open.onoff == true)
        {
            //            m_ftsensor.forceControl();
            //            cout <<"ft(3):"<< m_ftsensor.ft(2) <<endl;
        }
        //gripper close
        if(m_Gripper_Close.onoff == true)
        {
        }
        //JOINT_Error test
        if(m_Joint_Error.onoff == true)
        {
            if(m_Joint_Error.firsttime)
            {
                rt_printf("进入第一次关节误差测试程序\n");
                m_Joint_Error.firsttime =false;
                m_Joint_Error.t =0;
                for(int i=0;i<3;i++)
                {
                    joint_date[i].mpos_r.clear();
                    joint_date[i].mvel_r.clear();
                    joint_date[i].macc_r.clear();
                    joint_date[i].mtor_r.clear();
                    joint_date[i].mpos_d.clear();
                    joint_date[i].mvel_d.clear();
                    joint_date[i].macc_d.clear();
                    joint_date[i].mtor_d.clear();
                    joint_date[i].jpos_r.clear();
                    joint_date[i].jvel_r.clear();
                    joint_date[i].jacc_r.clear();
                    joint_date[i].jtor_r.clear();
                    joint_date[i].jpos_d.clear();
                    joint_date[i].jvel_d.clear();
                    joint_date[i].jacc_d.clear();
                    joint_date[i].jtor_d.clear();
                    joint_date[i].cjpos.clear();
                    joint_date[i].cjvel.clear();
                    joint_date[i].cjacc.clear();
                    joint_date[i].cjtor.clear();
                }
                t.clear();
                float pr,vr,tr;
                for(int i=0;i<3;i++)
                {
                    m_canComm ->canTx(i+607,0,NULL);
                    Jc_Real.pos_last[i] = (m_canComm->encoders[i]->pos)*PI/180;
                    Jc_Real.vel_last[i] = 0;
                    m_canComm ->motors[i]->getState(pr,vr,tr);
                    Mc_Real.pos_last[i] = pr - m_canComm->motors[i]->m_param.zeroPos;
                    Mc_Real.vel_last[i] = vr;
                }
                r_ctrl_rod.ev_last << motortor_r;
            }
            else
            {
                //                 rt_printf("进入关节误差测试主程序\n");
                VectorXd q(m_Dofs);
                VectorXd pos(m_Dofs+1);
                VectorXd vel(m_Dofs+1);
                VectorXd tor(m_Dofs+1);
                float theta[3];
                double A_lim_j0 = PI/3;
                double A_lim_j1 = PI/4;
                double A_lim_j2 = 5*PI/12;
                //                                                double w_base0 = 0.9;
                //                                double w_base0 = 0.5;
                double w_base0 = 0.1;
                //                double w_base1 = 0.9;
                //                                                double w_base1 = 0.5;
                double w_base1 = 0.1;
                //                double w_base2 = 1.8;
                //                                 double w_base2 = 1.0;
                //                double w_base2 = 0.2;
                double w_base2 =0.9;
                double w_j0 = 2*PI*w_base0;
                double w_j1 = 2*PI*w_base1;
                double w_j2 = 2*PI*w_base2;
                double T_j0 = 1/w_base0;
                double T_j1 = 1/w_base1;
                double T_j2 = 1/w_base2;
                theta[0]  = A_lim_j0*cos(w_j0* m_Joint_Error.t)-A_lim_j0;
                theta[1]  = -(A_lim_j1*cos(w_j1* m_Joint_Error.t)-A_lim_j1);
                theta[2]  = A_lim_j2*cos(w_j2* m_Joint_Error.t)-A_lim_j2;
                m_Joint_Error.t_out =  m_Joint_Error.t;
                m_Joint_Error.t = m_Joint_Error.t + m_TStep;
                if( m_Joint_Error.jointerr_flag ==1)
                {
                    //                    cout << T_j0 <<endl;
                    cout << "最大速度："<<A_lim_j0* w_j0<<endl;
                    q <<theta[0] ,0.0,0.0,0.0,0.0,0.0;
                    m_dynamics.setJointPositions(q);
                    calcPVAfromP(q,m_Desire,false);
                    m_dynamics.inverseDynamics(m_Desire.posd,m_Desire.veld,m_Desire.accd,m_Desire.tord);
                    pos <<m_Desire.posd[0],m_Desire.posd[1],m_Desire.posd[2],m_Desire.posd[3],m_Desire.posd[4],m_Desire.posd[5],gripperpos_last;
                    vel <<m_Desire.veld[0],m_Desire.veld[1],m_Desire.veld[2],m_Desire.veld[3],m_Desire.veld[4],m_Desire.veld[5],0;
                    tor <<m_Desire.tord[0],m_Desire.tord[1],m_Desire.tord[2],m_Desire.tord[3],m_Desire.tord[4],m_Desire.tord[5],0;
                    jointMove(pos,vel,tor,POSMODE);
                    if(m_Joint_Error.t > 3*T_j0)
                    {
                        //                            m_Joint_Error.jointerr_flag = 1;
                        m_Joint_Error.t =0;
                        m_SM.action = SM_ACTION::NONE;
                        m_SM.state = SM_STATE::STOPPED;
                        m_Joint_Error.onoff = false;
                    }
                }
                if( m_Joint_Error.jointerr_flag ==1)
                {
                    //                    cout << T_j1 <<endl;
                    cout << "最大速度："<<A_lim_j1* w_j1<<endl;
                    q <<0.0,theta[1],0.0,0.0,0.0,0.0;
                    m_dynamics.setJointPositions(q);
                    calcPVAfromP(q,m_Desire,false);
                    m_dynamics.inverseDynamics(m_Desire.posd,m_Desire.veld,m_Desire.accd,m_Desire.tord);
                    pos <<m_Desire.posd[0],m_Desire.posd[1],m_Desire.posd[2],m_Desire.posd[3],m_Desire.posd[4],m_Desire.posd[5],gripperpos_last;
                    vel <<m_Desire.veld[0],m_Desire.veld[1],m_Desire.veld[2],m_Desire.veld[3],m_Desire.veld[4],m_Desire.veld[5],0;
                    tor <<m_Desire.tord[0],m_Desire.tord[1],m_Desire.tord[2],m_Desire.tord[3],m_Desire.tord[4],m_Desire.tord[5],0;
                    jointMove(pos,vel,tor,POSMODE);
                    if(m_Joint_Error.t > 3*T_j1)
                    {
                        //                            m_Joint_Error.jointerr_flag = 2;
                        m_Joint_Error.t =0;
                        m_SM.action = SM_ACTION::NONE;
                        m_SM.state = SM_STATE::STOPPED;
                        m_Joint_Error.onoff = false;
                    }
                }
                if( m_Joint_Error.jointerr_flag ==0)
                {
                    //                    cout << T_j2 <<endl;
                    cout << "最大速度："<<A_lim_j2* w_j2<<endl;
                    q <<0.0,0.0,theta[2],0.0,0.0,0.0;
                    m_dynamics.setJointPositions(q);
                    calcPVAfromP(q,m_Desire,false);
                    m_dynamics.inverseDynamics(m_Desire.posd,m_Desire.veld,m_Desire.accd,m_Desire.tord);
                    pos <<m_Desire.posd[0],m_Desire.posd[1],m_Desire.posd[2],m_Desire.posd[3],m_Desire.posd[4],m_Desire.posd[5],gripperpos_last;
                    vel <<m_Desire.veld[0],m_Desire.veld[1],m_Desire.veld[2],m_Desire.veld[3],m_Desire.veld[4],m_Desire.veld[5],0;
                    tor <<m_Desire.tord[0],m_Desire.tord[1],m_Desire.tord[2],m_Desire.tord[3],m_Desire.tord[4],m_Desire.tord[5],0;
                    jointMove(pos,vel,tor,POSMODE);
                    if(m_Joint_Error.t > 3*T_j2)
                    {
                        m_Joint_Error.jointerr_flag = 0;
                        m_Joint_Error.t =0;
                        m_SM.action = SM_ACTION::NONE;
                        m_SM.state = SM_STATE::STOPPED;
                        m_Joint_Error.onoff = false;
                    }
                }
                //                VectorXd    mtor_r_f;
                //                VectorXd    epos_f;
                VectorXd    mpos_d;
                mpos_d = VectorXd::Zero(m_Dofs+1);
                VectorXd    mvel_d;
                mvel_d = VectorXd::Zero(m_Dofs+1);
                VectorXd    mtor_d;
                mtor_d = VectorXd::Zero(m_Dofs+1);
                if(ctrlloopcount % 1== 0)
                {
                    for(int i=0;i<3;i++)
                    {
                        epos[i] = (m_canComm->encoders[i]->pos)*PI/180;
                    }
                    //epos_f =Filter(epos,&fiter);
                    calcPVAfromP(epos,Jc_Real,false,1);
                }
                r_ctrl_rod.ev << motortor_r;
                r_ctrl_rod.ev_f = (1.0-FILTER_A) * r_ctrl_rod.ev_last + FILTER_A *r_ctrl_rod.ev;
                r_ctrl_rod.ev_last =   r_ctrl_rod.ev_f ;
                //                mtor_r_f = Filter(i2 * motortor_r[1] + i2 * motortor_r[2],&fiter);
                Jc_Real.vel_f = Filter(Jc_Real.vel,&fiter);
                //                Jc_Real.acc_f = Filter(Jc_Real.acc,&fiter);
                calcPVAfromP(motorpos_r,Mc_Real,false,1);
                jointToMotor(pos,vel,tor,mpos_d,mvel_d,mtor_d);
                for(int i=0;i<3;i++)
                {
                    joint_date[i].mpos_r.push_back(motorpos_r[i]);
                    //                    joint_date[i].mpos_r.push_back(motorvel_r[i]);
                    joint_date[i].mvel_r.push_back(motorvel_r[i]);
                    //                    joint_date[i].mvel_r.push_back(Mc_Real.vel[i]);
                    joint_date[i].macc_r.push_back(Mc_Real.acc[i]);
                    joint_date[i].mtor_r.push_back(r_ctrl_rod.ev_f[i]);
                    joint_date[i].mpos_d.push_back(mpos_d[i]);
                    joint_date[i].mvel_d.push_back(mvel_d[i]);
                    joint_date[i].mtor_d.push_back(mtor_d[i]);  //动力学方程得关节期望力矩,jointToMotor得电机期望力矩
                    //                    joint_date[i].epos.push_back(m_canComm ->encoders[i]->pos);
                    joint_date[i].jpos_r.push_back(Jc_Real.pos[i]);
                    joint_date[i].jvel_r.push_back(Jc_Real.vel_f[i]);
                    joint_date[i].jacc_r.push_back(Jc_Real.acc_f[i]);
                    joint_date[i].jpos_d.push_back(m_Desire.posd[i]);
                    joint_date[i].jvel_d.push_back(m_Desire.veld[i]);
                    joint_date[i].jacc_d.push_back(m_Desire.accd[i]);
                    joint_date[i].jtor_d.push_back(m_Desire.tord[i]);
                    joint_date[i].cjpos.push_back(m_Real.posr[i]);
                    joint_date[i].cjvel.push_back(m_Real.velr[i]);
                    joint_date[i].cjtor.push_back(m_Real.torr[i]);
                }
                joint_date[0].macc_d.push_back(i1* joint_date[0].jacc_d.back());
                joint_date[1].macc_d.push_back(i2* joint_date[1].jacc_d.back() + i3* joint_date[1].jacc_d.back());
                joint_date[2].macc_d.push_back(-(i2 * joint_date[2].jacc_d.back() - i3 * joint_date[2].jacc_d.back()));
                joint_date[0].jtor_r.push_back(i1 *r_ctrl_rod.ev_f[0]);
                joint_date[1].jtor_r.push_back(i2 * r_ctrl_rod.ev_f[1] + i2 *r_ctrl_rod.ev_f[2]);
                joint_date[2].jtor_r.push_back(i3 * r_ctrl_rod.ev_f[1] - i3 *r_ctrl_rod.ev_f[2]);
                joint_date[0].cjacc.push_back(joint_date[0].macc_r.back() / i1);
                joint_date[1].cjacc.push_back((joint_date[1].macc_r.back() + joint_date[2].macc_r.back()) / (2*i2));
                joint_date[2].cjacc.push_back((joint_date[1].macc_r.back() - joint_date[2].macc_r.back()) / (2*i3));
                t.push_back(m_Joint_Error.t_out);
            }
        }
        //Identify test
        if(m_Identify.onoff == true)
        {
            if(m_Identify.firsttime)
            {
                rt_printf("进入第一次编码器测试程序\n");
                m_Identify.firsttime =false;
                m_Identify.t =0;
                for(int i=0;i<3;i++)
                {
                    joint_date[i].mpos_r.clear();
                    joint_date[i].mvel_r.clear();
                    joint_date[i].macc_r.clear();
                    joint_date[i].mtor_r.clear();
                    joint_date[i].mpos_d.clear();
                    joint_date[i].mvel_d.clear();
                    joint_date[i].macc_d.clear();
                    joint_date[i].mtor_d.clear();
                    joint_date[i].jpos_r.clear();
                    joint_date[i].jvel_r.clear();
                    joint_date[i].jacc_r.clear();
                    joint_date[i].jtor_r.clear();
                    joint_date[i].jpos_d.clear();
                    joint_date[i].jvel_d.clear();
                    joint_date[i].jacc_d.clear();
                    joint_date[i].jtor_d.clear();
                    joint_date[i].cjpos.clear();
                    joint_date[i].cjvel.clear();
                    joint_date[i].cjacc.clear();
                    joint_date[i].cjtor.clear();
                }
                t.clear();
                float pr,vr,tr;
                for(int i=0;i<3;i++)
                {
                    m_canComm ->canTx(i+7,0,NULL);
                    Jc_Real.pos_last[i] = m_canComm->encoders[i]->pos;
                    Jc_Real.vel_last[i] = 0;
                    m_canComm ->motors[i]->getState(pr,vr,tr);
                    Mc_Real.pos_last[i] = pr - m_canComm->motors[i]->m_param.zeroPos;
                    Mc_Real.vel_last[i] = vr;
                }
            }
            else
            {
            }
        }
        //force control -- Support
        if(m_Support.onoff == true)
        {
            if(m_Support.firsttime)
            {

                //模式提示
                if (m_task_state==0) //辅助搬运
                {
                    cout<<"辅助搬运力控"<<endl;
                }
                else if(m_task_state==1) //过顶支撑
                {
                    cout<<"zqh过顶支撑力控"<<endl;
                }
                else if(m_task_state==2) //销孔装配
                {
                    cout<<"销孔装配力控"<<endl;
                }
                else if(m_task_state==3) //悬挂作业
                {
                    cout<<"悬挂作业力控"<<endl;
                }
                else if(m_task_state==4) //焊接开关
                {
                    cout<<"焊接开关力控"<<endl;
                }
                VectorXd jp(m_Dofs);
                m_dynamics.getJointPositions(jp);
                m_Desire.posd.tail(3) << jp[3], jp[4], jp[5];//For xianlan
                m_Support.firsttime = false;
            }
            else
            {
                m_Desire.posd.head(3) = m_Real.posr.head(3);
                /*************************关节角度保护*******************************/
//                VectorXd limit_up = VectorXd::Zero(6);
//                VectorXd limit_down = VectorXd::Zero(6);
//                BYTE data[8];
//                if(datename == "JointPosition1")
//                {
//                    limit_up   << -j1_limit1,j2_limit2,j3_limit2,j4_limit2,j5_limit2,j6_limit2;
//                    limit_down   << -j1_limit2,j2_limit1,j3_limit1,j4_limit1,j5_limit1,j6_limit1;
//                    for(int i=0;i<m_mitmotor_num;i++)
//                    {
//                        if(m_Desire.posd[i]>= limit_up[i] || m_Desire.posd[i]<= limit_down[i] )
//                        {
//                            cout<<"joint"<< i+1 <<"over joint limit"<<endl;
//                            for(int i=0;i<3;i++)
//                            {
//                                m_canComm ->motors[i]->enableMotor(false);
//                            }
//                        }
//                    }
//                }
//                if(datename == "JointPosition4")
//                {
//                    limit_up   << j1_limit2,j2_limit2,j3_limit2,j4_limit2,j5_limit2,j6_limit2;
//                    limit_down   << j1_limit1,j2_limit1,j3_limit1,j4_limit1,j5_limit1,j6_limit1;
//                    for(int i=0;i<m_mitmotor_num;i++)
//                    {
//                        if(m_Desire.posd[i]>= limit_up[i] || m_Desire.posd[i]<= limit_down[i] )
//                        {
//                            cout<<"joint"<< i+1 <<"over joint limit"<<endl;
//                            for(int i=0;i<3;i++)
//                            {
//                                m_canComm ->motors[i]->enableMotor(false);
//                            }
//                        }
//                    }
//                }
                m_dynamics.setJointPositions(m_Desire.posd);
                m_Support.posr << m_Real.posr[0],m_Real.posr[1],m_Real.posr[2], 0.0, 0.0, 0.0;
                Math::Vector3d F;
                if (m_task_state==0) //辅助搬运
                {
                    //                     cout<<"辅助搬运力控"<<endl;
//                    F<< 0.0, 0.0, -10;
                     F<< 0.0, 0.0, -10;

                }
                else if(m_task_state==1) //过顶支撑
                {
                    //                      cout<<"过顶支撑力控"<<endl;
                    if(state_support == task_state_support::force)
                    {
                        m_Support.F = m_Support.F + 0.05;
                        F<< 0.0, 0.0, -m_Support.F;
                        if(m_Support.F >= 40.0)
                        {
                            m_Support.F = 40.0;
                        }
                     }
                    if(state_support == task_state_support::none)
                    {
                        m_Support.F = m_Support.F - 0.05;
                        F<< 0.0, 0.0, -m_Support.F;

                        if(m_Support.F <= 8.0)
                        {
                            m_Support.F = 8.0;
                            cmd = 'l';
                        }
                    }

                }
                else if(m_task_state==2) //销孔装配
                {
                    //                     cout<<"销孔装配力控"<<endl;
                    F<< 0.0, 0.0, -15.0;
                }
                else if(m_task_state==3) //悬挂作业
                {
                    //                     cout<<"悬挂作业力控"<<endl;
                    F<< 0.0, 0.0, -10.0;
                }
                else if(m_task_state==4) //焊接开关
                {
                    //                     cout<<"焊接开关力控"<<endl;
                    F<< 0.0, 0.0, -10.0;
                }
                Math::Vector3d pos;
                Math::Vector3d t_end;
                m_dynamics.getendpos_f(pos);
                t_end=pos.cross(F);
//                cout <<"F:"<<F[2]<<endl;
                Math::SpatialVector fext= SpatialVector(t_end[0],t_end[1],t_end[2],F[0],F[1],F[2]);
                m_Support.f_ext[3] = fext;
                //                 m_dynamics.inverseDynamics(m_Support.posr,m_Support.velr,m_Support.accr,m_Support.torr);  //重力平衡
                m_dynamics.inverseDynamics(m_Support.posr,m_Support.velr,m_Support.accr,m_Support.torr_inv, &m_Support.f_ext);  //力输出

                /****************dy_inv + J*********************/
                if(datename == "JointPosition1")
                {
                    m_Support.F_ext_end << 0,0,0,0,0,-30;
                }
                if(datename == "JointPosition4")
                {
                    m_Support.F_ext_end << 0,0,0,0,0,-30;
                }
                m_dynamics.getJacobi(m_Support.G);
                m_Support.tor_ext = m_Support.G.transpose()*m_Support.F_ext_end;
                for(int i=0;i<6;i++) m_Support.torr[i] -= m_Support.tor_ext[i];
                 /****************dy_inv + J*********************/
                VectorXd pos_j(7);
                VectorXd vel_j(7);
                VectorXd tor_j(7);
                pos_j<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                vel_j<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                tor_j<<m_Support.torr_inv[0],m_Support.torr_inv[1],m_Support.torr_inv[2],0.0 ,0.0 ,0.0, 0.0;
                jointMove(pos_j,vel_j,tor_j,TORMODE);

                if(cmd=='l')
                {
                    rt_printf("leave the  gblance mode\n");
                    m_SM.action = SM_ACTION::NONE;
                    m_SM.state = SM_STATE::STOPPED;
                    m_Support.onoff = false;

                    if (m_task_state==0) //辅助搬运
                    {
                        rt_printf("搬运 力控结束\n");
                        float vel = 15.0f*gSpeed;
                        float acc = 2.0f*gSpeed;
                        Math::Vector3d pos,rpy;
                        Eigen::VectorXd tp1(m_Dofs);
                        VectorXd jp(m_Dofs);
                        VectorXd jp1(m_Dofs+1);
                        m_dynamics.getJointPositions(jp);
                        m_dynamics.forwardKinematics(jp,pos,rpy);
                        tp1 << pos[0],pos[1],pos[2]-0.05, 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0;

                        if(datename == "JointPosition1")
                        {
                        jp1 <<  35.0f * PI / 180.0f, 93.0f * PI / 180.0f, -175.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
                        }
                        if(datename == "JointPosition4")
                        {
                        jp1 <<  -35.0f * PI / 180.0f, 93.0f * PI / 180.0f, -175.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
                        }
                        clearRmlPoints();
//                        addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/2,acc/2);
                        addRmlPoint(MOTION_TYPE::JOINT_RML,jp1,vel,acc);
                        beginMotion(MOTION_TYPE::JOINT_RML);
                        cmd  = 'w';

                    }
                    else if(m_task_state==1) //过顶支撑
                    {
                        rt_printf("过顶支撑 力控结束\n");
                        float vel = 15.0f*gSpeed;
                        float acc = 2.0f*gSpeed;
                        Math::Vector3d pos,rpy;
                        Eigen::VectorXd tp1(m_Dofs);
                        VectorXd jp(m_Dofs);
                        VectorXd jp1(m_Dofs+1);
                        m_dynamics.getJointPositions(jp);
                        m_dynamics.forwardKinematics(jp,pos,rpy);
                        tp1 << pos[0],pos[1],pos[2]-0.05, 90.0f * PI / 180.0f,  0.0f * PI / 180.0f, -90.0f * PI / 180.0;

                        if(datename == "JointPosition1")
                        {
                        jp1 <<  35.0f * PI / 180.0f, 93.0f * PI / 180.0f, -175.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
                        }
                        if(datename == "JointPosition4")
                        {
                        jp1 <<  -35.0f * PI / 180.0f, 93.0f * PI / 180.0f, -175.0f* PI / 180.0f, 0.0 * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f, 0.0f * PI / 180.0f;     //work position
                        }
                        clearRmlPoints();
//                        addRmlPoint(MOTION_TYPE::CARTESIAN_RML_6D,tp1,vel/2,acc/2);
                        addRmlPoint(MOTION_TYPE::JOINT_RML,jp1,vel,acc);
                        beginMotion(MOTION_TYPE::JOINT_RML);
                        cmd  = 'w';
                    }
                }






            }
        }
        getMotorStates(m_Real.posr,m_Real.velr,m_Real.torr);
    }
    else
    {
        int a =0;
        fetchMotorStates(m_Real.posr,m_Real.velr,m_Real.torr);
    }
    if(ctrlloopcount % t_send == 0)
    {
        m_vrep.setJointPositions(m_Desire.posd,m_Real.posr,datename);
        //cout  <<"m_Real.posr:"<<m_Real.posr<<endl;
    }
//    if(curvename=="GraphData1")
//    {
//        curvedata[0] = m_Desire.veld[1];
//        curvedata[1] = m_Desire.accd[1];

//        curvedata[2] =    motortor_r[2] ;
//        curvedata[3] =   m_canComm->motors[2]->m_cmd.tcmd ;
//        curvedata[4] =  Jc_Real.vel[1];
//        curvedata[5] =  Jc_Real.vel_f[1];
//        VrepGraphAddData(curvedata);
//    }
}
robotArm::robotArm()
{
    m_StopCtrl = false;
    m_MotorsEnabled = false;
    m_MainCtrlTask = NULL;
    m_TStep = 0.001f;
    m_SM.state = SM_STATE::STOPPED;
}
robotArm::~robotArm()
{
    rt_task_delete(m_MainCtrlTask);
    m_canComm ->close();
    //    m_485Comm.close();
    m_vrep.close();
}
void robotArm::setZeroPos()
{
    VectorXd encoder_pos,b;
    encoder_pos =VectorXd::Zero(3);
    b =VectorXd::Zero(4);
    for(int i=0;i<4;i++)
    {
        m_canComm->canTx(i+7,0,NULL);
    }
    usleep(10000);
    for(int i=0;i<4;i++)
    {
        m_canComm ->canTx(i+0x607,0,NULL);
        m_canComm ->encoders[i]->setZeroPos();
        rt_printf("encoder[%d]:%0.3f\n",i+1,m_canComm ->encoders[i]->pos_init*180/PI);
    }
//    encoder_pos<< m_canComm ->encoders[0]->pos_init, m_canComm ->encoders[1]->pos_init, m_canComm ->encoders[2]->pos_init;
     encoder_pos<< 0.0, m_canComm ->encoders[1]->pos_init, m_canComm ->encoders[2]->pos_init;
    jointToMotor(encoder_pos,encoder_offset);
    motorpos_ini[0]  =  encoder_offset[0];
    motorpos_ini[1]  =  encoder_offset[1];
    motorpos_ini[2]  =  encoder_offset[2];
    usleep(10000);
    VectorXd mp(m_mitmotor_num),mv(m_mitmotor_num),mt(m_mitmotor_num);
    float pr,vr,tr;
    for(int i=0;i<m_mitmotor_num;i++)
    {
        m_canComm->motors[i]->enableMotor(false);
        m_canComm->motors[i]->getState(pr,vr,tr);
        mp[i] = pr + motorpos_ini[i];
        mv[i] = vr;
        mt[i] = tr;
    }
    for(int i=0;i<m_mitmotor_num;i++)
    {
        m_canComm ->motors[i]->setZeroPos();
        m_canComm ->motors[i]->m_cmd.pcmd_last = m_canComm ->motors[i]->m_param.zeroPos;
        rt_printf("m_param.zeroPos[%d]:%0.3f\n",i,m_canComm ->motors[i]->m_param.zeroPos);
        rt_printf("m_cmd.pcmd_last[%d]:%0.3f\n",i,m_canComm ->motors[i]->m_cmd.pcmd_last);
    }
     m_RMLMotion.q_inv_last<<encoder_pos[0],encoder_pos[1],encoder_pos[2],0,0,0;
    //    //改
    //            m_Desire.posd = m_Real.posr;
    //            m_dynamics.setJointPositions(m_Desire.posd);
    //        m_MotorsEnabled=1;
    motorToJoint(mp,mv,mt,m_Real.posr,m_Real.velr,m_Real.torr);
}
bool robotArm::init(rtCan* canComm,rt485 m_485Com,vrepsim * vrep ,dynamics* dynamic, char jointpos_date[], char curve_date[],float tstep,int time_send)
//bool robotArm::init(vrepsim* vrep,rtCan* canComm,float tstep)
{
    m_TStep = tstep;
    m_TVrepRefresh = 0.02f;
    m_MotionMode = MODE_POSITION;
    //m_Dofs = m_dynamics.getDofs();
    m_Dofs = 6;
    m_mitmotor_num = 3;
    m_magneticencoder_num = 4;
    m_dynamixel_num = 4;
    t_send = time_send;
    m_motionPlan.init(m_Dofs+1,m_TStep);
    m_dynamics = *dynamic;
    datename = jointpos_date;
    curvename = curve_date;
    //    m_t265.init();
    m_canComm = canComm;
    m_485Comm = m_485Com;
    m_vrep  = *vrep;
    //    add the can node
    for(int i=0;i<m_mitmotor_num;i++)
    {
        m_canComm ->addmotor(1+i);
    }
    for(int i=0;i<m_magneticencoder_num;i++)
    {
        m_canComm ->addencoder(7+i);
    }
    //add the 485 node
    if(datename == "JointPosition1")
    {
        //        m_ftsensor.init("192.168.199.100");
        //        m_ftsensor.start();
        float initpos_l[4]= {178.59,179.21,92.988,150.645};  // left  limb
        for(int i=0;i<4;i++)
        {
            m_485Comm.adddynamixel(i+1);
            m_485Comm.dynamixels[i]->initpos = initpos_l[i];
        }
    }
    if(datename == "JointPosition4")
    {

        float initpos_r[4]= {178.86,199.78,91.58,0};  // right limb
        for(int i=0;i<4;i++)
        {
            m_485Comm.adddynamixel(i+1);
            m_485Comm.dynamixels[i]->initpos = initpos_r[i];;
        }
    }
    m_RMLMotion.q_inv_last = VectorXd::Zero(m_Dofs);
    m_Desire.posd = VectorXd::Zero(m_Dofs);
    m_Desire.veld = VectorXd::Zero(m_Dofs);
    m_Desire.accd = VectorXd::Zero(m_Dofs);
    m_Desire.tord = VectorXd::Zero(m_Dofs);
    m_Desire.posd_last= VectorXd::Zero(m_Dofs);
    m_Desire.veld_last = VectorXd::Zero(m_Dofs);
    m_Desire.accd_last = VectorXd::Zero(m_Dofs);
    m_Desire.tord_last = VectorXd::Zero(m_Dofs);
    m_Real.posr = VectorXd::Zero(m_Dofs);
    m_Real.velr = VectorXd::Zero(m_Dofs);
    m_Real.torr = VectorXd::Zero(m_Dofs);
    fiter.value = VectorXd::Zero(3);
    fiter.value_last = VectorXd::Zero(3);
    r_ctrl.ep = VectorXd::Zero(3);
    r_ctrl.ep_last = VectorXd::Zero(3);
    r_ctrl.ev = VectorXd::Zero(3);
    r_ctrl.ev_last = VectorXd::Zero(3);
    r_ctrl.ea = VectorXd::Zero(3);
    r_ctrl.ea_last = VectorXd::Zero(3);
    r_ctrl.epi = VectorXd::Zero(3);
    r_ctrl_rod.ep = VectorXd::Zero(1);
    r_ctrl_rod.ep_last = VectorXd::Zero(1);
    r_ctrl_rod.ev = VectorXd::Zero(3);
    r_ctrl_rod.ev_f = VectorXd::Zero(3);
    r_ctrl_rod.ev_last = VectorXd::Zero(3);
    r_ctrl_rod.ea = VectorXd::Zero(1);
    r_ctrl_rod.ea_f = VectorXd::Zero(1);
    r_ctrl_rod.ea_last = VectorXd::Zero(1);
    r_ctrl_rod.epi = VectorXd::Zero(1);


    m_G_Blance.torr = VectorXd::Zero(m_Dofs);
    m_G_Blance.torr_inv = VectorXd::Zero(m_Dofs);
    m_G_Blance.torr_j = VectorXd::Zero(m_Dofs);
    m_G_Blance.tor_ext = VectorXd::Zero(m_Dofs);
    m_G_Blance.torr_G = VectorXd::Zero(m_Dofs);
    m_G_Blance.torr_motor = VectorXd::Zero(m_Dofs);
    m_G_Blance.posr = VectorXd::Zero(m_Dofs);
    m_G_Blance.posr_last = VectorXd::Zero(m_Dofs);
    m_G_Blance.velr = VectorXd::Zero(m_Dofs);
    m_G_Blance.velr_last = VectorXd::Zero(m_Dofs);
    m_G_Blance.accr =  VectorXd::Zero(m_Dofs);
    m_G_Blance.accr_last = VectorXd::Zero(m_Dofs);
    m_G_Blance.G = MatrixNd::Zero(m_Dofs,m_Dofs);
    m_G_Blance.F_ext_end = VectorXd::Zero(m_Dofs);
    m_G_Blance.F_ext_end_copy = VectorXd::Zero(m_Dofs);
    m_G_Blance.f_ext.resize(m_Dofs+1);
    for(unsigned int i=0; i<m_Dofs+1;++i){
        m_G_Blance.f_ext[i]=SpatialVector::Zero();
    }

/************************force support*****************/
    m_Support.torr = VectorXd::Zero(m_Dofs);
    m_Support.torr_inv = VectorXd::Zero(m_Dofs);
    m_Support.torr_j = VectorXd::Zero(m_Dofs);
    m_Support.tor_ext = VectorXd::Zero(m_Dofs);
    m_Support.torr_motor = VectorXd::Zero(m_Dofs);
    m_Support.posr = VectorXd::Zero(m_Dofs);
    m_Support.posr_last = VectorXd::Zero(m_Dofs);
    m_Support.velr = VectorXd::Zero(m_Dofs);
    m_Support.velr_last = VectorXd::Zero(m_Dofs);
    m_Support.accr =  VectorXd::Zero(m_Dofs);
    m_Support.accr_last = VectorXd::Zero(m_Dofs);
    m_Support.G = MatrixNd::Zero(m_Dofs,m_Dofs);
    m_Support.F_ext_end = VectorXd::Zero(m_Dofs);
    m_Support.F_ext_end_copy = VectorXd::Zero(m_Dofs);
    m_Support.f_ext.resize(m_Dofs+1);
    for(unsigned int i=0; i<m_Dofs+1;++i){
        m_Support.f_ext[i]=SpatialVector::Zero();
    }

    m_Xuangua.torr = VectorXd::Zero(m_Dofs);
    m_Xuangua.tor_ext = VectorXd::Zero(m_Dofs);
    m_Xuangua.torr_motor = VectorXd::Zero(m_Dofs);
    m_Xuangua.posr = VectorXd::Zero(m_Dofs);
    m_Xuangua.posr_last = VectorXd::Zero(m_Dofs);
    m_Xuangua.velr = VectorXd::Zero(m_Dofs);
    m_Xuangua.velr_last = VectorXd::Zero(m_Dofs);
    m_Xuangua.accr =  VectorXd::Zero(m_Dofs);
    m_Xuangua.accr_last = VectorXd::Zero(m_Dofs);
    m_Xuangua.f_ext.push_back(SpatialVector(0,0,0,0,0,10));
    m_Xuangua.G = MatrixNd::Zero(6,6);
    m_Xuangua.F_ext_end = VectorXd::Zero(m_Dofs);
    m_Xuangua.F_ext_end_copy = VectorXd::Zero(m_Dofs);
    encoder_offset = VectorXd::Zero(4);
    motorpos_ini = VectorXd::Zero(4);
    jointtor_r = VectorXd::Zero(3);
    jointvel_r = VectorXd::Zero(3);
    jointpos_r = VectorXd::Zero(3);
    motortor_r = VectorXd::Zero(3);
    motorvel_r = VectorXd::Zero(3);
    motorpos_r = VectorXd::Zero(3);
    mtor_r_f = VectorXd::Zero(3);
    epos_f=VectorXd::Zero(3);
    Mc_Real.pos = VectorXd::Zero(3);
    Mc_Real.vel = VectorXd::Zero(3);
    Mc_Real.acc = VectorXd::Zero(3);
    Mc_Real.pos_last = VectorXd::Zero(3);
    Mc_Real.vel_last = VectorXd::Zero(3);
    Jc_Real.pos = VectorXd::Zero(3);
    Jc_Real.vel = VectorXd::Zero(3);
    Jc_Real.vel_f = VectorXd::Zero(3);
    Jc_Real.acc = VectorXd::Zero(3);
    Jc_Real.acc_f = VectorXd::Zero(3);
    Jc_Real.pos_last = VectorXd::Zero(3);
    Jc_Real.vel_last = VectorXd::Zero(3);
    mpos_d=VectorXd::Zero(m_Dofs+1);
    mvel_d=VectorXd::Zero(m_Dofs+1);
    mtor_d=VectorXd::Zero(m_Dofs+1);
    epos = VectorXd::Zero(3);
    m_canComm ->motors[0]->m_param.dir = 1;
    m_canComm ->motors[1]->m_param.dir = 1;
    m_canComm ->motors[2]->m_param.dir = -1;
    cout<<"1111\n"<<endl;
    return true;
}
void robotArm::startCtrl(const char *taskname)
{
    int i, err;
    //signal(SIGTERM, signal_handler);
    //signal(SIGINT, signal_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);
    m_MainCtrlTask = new RT_TASK();
    err = rt_task_create(m_MainCtrlTask, taskname, 0, 60, 0);
    rt_printf("err:%d\n",err);
    if (err) {
        printf("receivetest: Failed to create rt task, code %d\n",
               errno);
    }
    rt_task_start(m_MainCtrlTask, robotArm::rtCtrlProc, this);
    if (err) {
        printf("receivetest: Failed to start rt task, code %d\n",
               errno);
    }
}
void robotArm::stopCtrl()
{
    m_StopCtrl = true;
}
void robotArm::simulate()
{
}
void robotArm::enableMotors(bool enableOrDisable)
{
    //     printf("z\n");
    m_SM.action = enableOrDisable?SM_ACTION::ENABLE_MOTORS:SM_ACTION::DISABLE_MOTORS;
    //    printf("q\n");
}
void robotArm::setMotionMode(const int mode) //0:IK,1:FK,2:torque
{
    m_MotionMode = mode;
}
int robotArm::getMotionMode()
{
    return m_MotionMode;
}
void robotArm::setJointPositions(VectorXd q)
{
    m_dynamics.setJointPositions(q);
}
void robotArm::getJointPositions(VectorXd &q)
{
    m_dynamics.getJointPositions(q);
}
void robotArm::getEndPose(Math::Vector3d& pos, Math::Vector3d& rpy)
{
    m_dynamics.getEndPose(pos, rpy);
}
void robotArm::getEndPosition(Math::Vector3d& pos)
{
    m_dynamics.getEndPosition(pos);
}
void robotArm::getEndOrientation(Math::Vector3d& rpy)
{
    m_dynamics.getEndOrientation(rpy);
}
void robotArm::addRmlPoint(int type, const Eigen::VectorXd pos,float vel,float acc)
{
    rt_printf("enter the addRmlPoint\n");
    RMLMotionPoint rmlPoint;
    rmlPoint.type = type;
    rmlPoint.pos = pos;
    rmlPoint.vel = vel;
    rmlPoint.acc = acc;
    m_RMLMotion.ps.push_back(rmlPoint);
    rt_printf("enter the addRmlPoint\n");
}
void robotArm::clearRmlPoints()
{
    m_RMLMotion.ps.clear();
}
void robotArm::beginMotion(int motiontype)
{
    if(m_SM.state == SM_STATE::STOPPED)
    {
        m_SM.action = SM_ACTION::BEGIN_MOTION;
        m_SM.motiontype = motiontype;
        //        rt_printf("m_SM.action:%d,m_SM.state:%d,m_SM.motiontype:%d\n",m_SM.action,m_SM.state,m_SM.motiontype);
        //        rt_printf("change the motiontype:%d\n",m_SM.motiontype);
    }
    else
    {
        rt_printf("previous motion is not finished, motion command is ignored!");
    }
}
int robotArm::getMotionState()
{
    return m_SM.state;
}
void robotArm::stopMotion()
{
    m_SM.action = SM_ACTION::STOP_MOTION;
}
