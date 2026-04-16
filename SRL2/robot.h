#ifndef ROBOT_H
#define ROBOT_H
#include "motionplanning.h"
#include "dynamics.h"
#include "mitmotor.h"
#include "rtcan.h"
#include "rt485.h"
#include "vrepsim.h"
#include "t265camera.h"
#include "magneticencoder.h"
#include "Dynamixel.h"
#include "ftsensor.h"

#define POSMODE 1
#define VELMODE 2
#define TORMODE 3

#define  mtor_l 9
#define  mvel_l 800*PI/180
#define  mpos_l 4*PI/180


using namespace RigidBodyDynamics::Math;
class RT_TASK;

namespace robot
{
using namespace  std;
enum MOTION_MODE
{
    MODE_POSITION = 0,
    MODE_VELOCITY = 1,
    MODE_TORQUE = 2
};

enum MOTION_TYPE
{
    JOINT_RML = 0x01,
    CARTESIAN_RML_6D = 0x03,
    CARTESIAN_RML_TRANSLATION = 0x04,
    CARTESIAN_RML_ROTATION = 0x05,
    BIRDHEAD = 0x06,
    BIRDHEAD_Rod = 0x07,
    BIRDHEAD_Planing = 0x08,
    G_Blance = 0x09,

    JOINT_Planing = 0x10,
    GRIPPER_Open = 11,
    GRIPPER_Close =12,
    JOINT_Error = 13,
    Identify = 14,
    SUPPORT = 15,
    G_Blance_Xuangua=16,
};

enum SM_STATE
{
    RUNNING = 0x01,
    STOPPED = 0x00
};

enum SM_ACTION
{
    NONE        = 0x00,
    ENABLE_MOTORS = 0x01,
    DISABLE_MOTORS = 0x02,
    BEGIN_MOTION = 0x03,
    STOP_MOTION = 0x04
};

//********************Sun**********************
enum FORCECONTROLTEST_TYPE
{
    ALL = 0x01,
    POSITION = 0x02,
    ROTATION = 0x03,
    XPOS = 0x04,
    YPOS = 0x05,
    ZPOS = 0x06,
    XROT = 0x07,
    YROT = 0x08,
    ZROT = 0x09,

};
//*********************Sun**********************

class robotArm
{
public:

    struct MOTION_CMD
    {
        VectorXd posd;
        VectorXd veld;
        VectorXd accd;
        VectorXd tord;

        VectorXd posd_last;
        VectorXd veld_last;
        VectorXd accd_last;
        VectorXd tord_last;
    }m_Desire;

    struct Diff_va
    {
        VectorXd pos;
        VectorXd vel;
        VectorXd vel_f;
        VectorXd acc;
        VectorXd acc_f;

        VectorXd pos_last;
        VectorXd vel_last;
    }Mc_Real,Jc_Real;

    /*add*/
    struct FITER_STATUS
    {
        VectorXd value;
        VectorXd value_last;
    }fiter;

    struct BIRDHEAD_PID
    {
        float kp = 0.0002f;
//                float ki =  0.01f;
        float ki =  0.01f;
                float kd = 0.000001f;
//        float kd = 0.0000008f;
        float epi_sum = 100.0;

        VectorXd ep ;
        VectorXd ep_last;
        VectorXd ev ;
        VectorXd ev_last;
        VectorXd ea;
        VectorXd ea_last;
        VectorXd epi ;
    }r_ctrl;

    struct BIRDHEADROD_ROD_PID
    {
/*增量pid参数*/
//        float ki = 0.00003f;    //0
//            float ki = 0.00005f;  //1
        float ki = 0.0001f;  //2

 /*绝对pid参数*/
         float kp = 0.000055f;    //0
         float kd = 0.00000f;    //0
//            float kd = 0.000005f;    //0
         VectorXd ep ;
         VectorXd ep_last;
         VectorXd ev ;
         VectorXd ev_f;
         VectorXd ev_last;
         VectorXd ea;
         VectorXd ea_f;
         VectorXd ea_last;
         VectorXd epi ;

//         float ep ;
//         float ep_last;
//         float ev ;
//         float ev_last;
//         float ea;
//         float ea_last;
//         float epi ;
    }r_ctrl_rod;

    struct BIRDHEAD_Motion_Rod
    {
        bool onoff;
        bool firsttime;
        float initpos = 0;
//        Eigen::Vector4d initpose;
        Eigen::Vector3d rob_p0;
        Eigen::Vector3d rob_o0;
        Eigen::Vector3d rob_pd;

    }m_BirdHeadMotion_Rod;

    struct BIRDHEAD_Motion_Planing
    {
        bool onoff;
        bool firsttime;
        float x = 0;
        float z = 0;

        float y1 = 0;
        float z1 = 0;
        float theta = 0;
        float detax = - 0.005;
        float deta_theta =  0.005;

        Eigen::Vector3d rob_p0;
        Eigen::Vector3d rob_o0;
        Eigen::Vector3d rob_pd;

    }m_BirdHeadMotion_Planing;

    struct Dynamical_Blance
    {
        bool onoff;
        bool firsttime;
        VectorXd torr;
        VectorXd torr_inv;
        VectorXd torr_j;
        VectorXd tor_ext ;
        VectorXd torr_G;
        VectorXd torr_motor;
        std::vector<SpatialVector> f_ext;
        Math::MatrixNd G;
        VectorXd F_ext_end;

         VectorXd F_ext_end_copy;

        VectorXd posr ;
        VectorXd posr_last;
        VectorXd velr ;
        VectorXd velr_last;
        VectorXd accr;
        VectorXd accr_last;
        float kp = 0.1;
    }m_G_Blance;

    struct RML_Planing
    {
        bool onoff;
        bool firsttime1;
        bool firsttime2;
        float t = 0;
        float deta_t =  0.001;
        float t_out =0;
    }m_Joint_Planing;

    struct Gripper_Open
    {
        bool onoff;
        bool firsttime;
    }m_Gripper_Open;

    struct Gripper_Close
    {
        bool onoff;
        bool firsttime;
    }m_Gripper_Close;

    struct Joint_Error
    {
        bool onoff;
        bool firsttime;
        float t = 0;
        float deta_t = 0.001;
        float t_out =0;
        int  jointerr_flag =0;
    }m_Joint_Error;

    struct Identify
    {
        bool onoff;
        bool firsttime;
        float t = 0;
        float deta_t = 0.001;
        float t_out =0;
    }m_Identify;

    struct SUPPORT_Motion
    {
        bool onoff;
        bool firsttime;
        VectorXd torr;
        VectorXd torr_inv;
        VectorXd torr_j;
        VectorXd tor_ext ;
        VectorXd torr_motor;
        std::vector<SpatialVector> f_ext;
        Math::MatrixNd G;
        VectorXd F_ext_end;

         VectorXd F_ext_end_copy;

        VectorXd posr ;
        VectorXd posr_last;
        VectorXd velr ;
        VectorXd velr_last;
        VectorXd accr;
        VectorXd accr_last;
        float kp = 0.1;
        float F = 8.0;

    }m_Support;


//*********************jing******************//
    struct Xuangua
    {
        bool onoff;
        bool firsttime;
        VectorXd torr;
        VectorXd tor_ext ;
        VectorXd torr_motor;
        std::vector<SpatialVector> f_ext;
        Math::MatrixNd G;
        VectorXd F_ext_end;

         VectorXd F_ext_end_copy;

        VectorXd posr ;
        VectorXd posr_last;
        VectorXd velr ;
        VectorXd velr_last;
        VectorXd accr;
        VectorXd accr_last;
        float kp = 0.1;
    }m_Xuangua;
//***********************jing*************************//


    struct MOTION_STATUS
    {
        VectorXd posr;
        VectorXd velr_c;
        VectorXd velr;
        VectorXd accr_c;
        VectorXd accr;
        VectorXd torr;

        VectorXd posr_last;
        VectorXd velr_c_last;
        VectorXd velr_last;
        VectorXd accr_c_last;
        VectorXd accr_last;
        VectorXd torr_last;
    }m_Real;


    struct protection
    {
        VectorXd mtor_limit;
        VectorXd jtor_limit;

        VectorXd mvel_limit;
        VectorXd jvel_limit;

        VectorXd mpos_limit;
        VectorXd jpos_limit;

    }protect;

    struct RMLMotionPoint
    {
        int type;
        VectorXd pos;
        float vel;
        float acc;
    };

    struct RML_Motion
    {
        Math::Vector3d pos0;
        Math::Matrix3d mat0;
        VectorXd jointPos0;
        vector<RMLMotionPoint> ps;
        uint idx;
        uint substep;
        bool onoff;
        bool firsttime;
        float t = 0;
        VectorXd q_inv_last;
    }m_RMLMotion;

    struct BIRDHEAD_Motion
    {
        bool onoff;
        bool firsttime;
        Eigen::Vector3d rob_p0;
        Eigen::Vector3d rob_o0;
        Eigen::Vector3d rob_pd;
        Eigen::Vector3d rob_pd1;
        Eigen::Vector3d cam_p0;
        Eigen::Vector3d cam_pry0;

        Eigen::Vector3d cam_v;
        Eigen::Vector3d cam_a;
    }m_BirdHeadMotion;

    struct StateMachine
    {
        int action;
        int state;//0x00,disabled,0x01-enabled,0x-running,3-stoped
        int motiontype;
    }m_SM;

    VectorXd encoder_offset;
    VectorXd jointtor_r;
    VectorXd jointvel_r;
    VectorXd jointvel_r_f;
    VectorXd jointpos_r;

    VectorXd motortor_r;
    VectorXd mtor_r_f;
    VectorXd motorvel_r;
    VectorXd motorpos_r;

    VectorXd motorpos_ini;
    VectorXd epos;
    VectorXd epos_f;



    struct joint_date
    {
        vector<double> mpos_r;  //电机编码器得
        vector<double> mpos_d;  //jointToMotor：jpos_d转换得

        vector<double> mvel_r;
        vector<double> mvel_d;

        vector<double> macc_r;
        vector<double> macc_d;

        vector<double> mtor_r;
        vector<double> mtor_d;

        vector<double> jpos_r;  //编码器得
        vector<double> jpos_d;  //指令给

        vector<double> jvel_r;
        vector<double> jvel_d;

        vector<double> jacc_r;
        vector<double> jacc_d;

        vector<double> jtor_r;
        vector<double> jtor_d;

        vector<double> cjpos;  //motorToJoint:mpos_r得
        vector<double> cjvel;
        vector<double> cjacc;
        vector<double> cjtor;


    }joint_date[3];



    struct encoder_data
    {
        double epos[3];
        double evel[3];
        double eacc[3];

        double epos_last[3];
        double evel_last[3];

    }encoder_data;

    vector<double> torque_joint1_r;
    vector<double> torque_joint2_r;
    vector<double> torque_joint3_r;

    vector<double> torque_joint1_d;
    vector<double> torque_joint2_d;
    vector<double> torque_joint3_d;

    vector<double> angle_joint1;
    vector<double> angle_joint2;
    vector<double> angle_joint3;
    VectorXd mpos_d;
    VectorXd mvel_d;
    VectorXd mtor_d;

    vector<double> t;


    robotArm();
    ~robotArm();
    static  void rtCtrlProc(void* arg);
//     static  void rtCtrlProc(robotArm* arm);
    bool init(rtCan* canComm,rt485 m_485Com,vrepsim * vrep,dynamics* dynamic, char jointpos_date[], char curve_date[], float tstep,int time_send);
//     bool init(rtCan* canComm,float tstep);

    void simulate();
    void enableMotors(bool enableOrDisable);

    float getSimulationTime();

    //    void calcDynamics(VectorXd q,VectorXd qd,VectorXd qdd,VectorXd& tau,bool firsttime);

    void setMotionMode(const int mode);
    int getMotionMode();


    void setJointPositions(VectorXd q);
    void getJointPositions(VectorXd &q);
    //    void setEndPose(const float ep[]){};
    void getEndPose(Math::Vector3d& pos, Math::Vector3d& rpy);
    void getEndPosition(Math::Vector3d& pos);
    void getEndOrientation(Math::Vector3d& rpy);

    void addRmlPoint(int type, const Eigen::VectorXd pos,float vel,float acc);
    //    void beginRmlMotion();
    void clearRmlPoints();
    void beginMotion(int motiontype);
    void stopMotion();

    void startCtrl(const char* taskname);

    void stopCtrl();

    void ctrlStateMachine();

    void sendMotorCmds(VectorXd pos,VectorXd vel,VectorXd tff,int type);

//    void VrepGraphAddData(float curvedata[]);

    void getMotorStates(VectorXd &p,VectorXd &v,VectorXd &t);

    void fetchMotorStates(VectorXd &p,VectorXd &v,VectorXd &t);
    double round_to_lmt(double v, double lmt_up ,double lmt_down);

    void jointToMotor(VectorXd jpos,VectorXd jvel,VectorXd jtor,VectorXd &mpos,VectorXd &mvel,VectorXd &mtor);

    void jointToMotor(VectorXd jpos,VectorXd &mpos);

    void motorToJoint(VectorXd mpos,VectorXd mvel,VectorXd mtor,VectorXd &jpos,VectorXd &jvel,VectorXd &jtor);

    void motorToJoint(VectorXd mpos,VectorXd &jpos);

    void jointMove(VectorXd pos,VectorXd vel,VectorXd tor,int type);
    void jointMove_t(VectorXd tor);

    void setZeroPos();

    void calcPVAfromP(VectorXd pos, MOTION_CMD& mc,bool firsttime);
    void calcPVAfromP(VectorXd pos, MOTION_STATUS& mc,bool firsttime);
    void calcPVAfromP(VectorXd pos, Diff_va& mc,bool firsttime,float n_TStep);
    VectorXd Filter(VectorXd value,struct FITER_STATUS *fiter);
    int getMotionState();

    long                ctrlloopcount = 0;
    double              ctrltime = 0;
    bool                m_StopCtrl;
    bool                m_MotorsEnabled;
    motionplanning      m_motionPlan;
    dynamics            m_dynamics;
    vrepsim             m_vrep;
    rtCan*               m_canComm;
    rt485               m_485Comm;
    Dynamixel           m_dynamixel;
    int                 m_MotionMode;
    int                 m_Dofs;
    int                 m_mitmotor_num;
    int                 m_magneticencoder_num;
    int                 m_dynamixel_num;
    float               m_TStep;
    float               m_TVrepRefresh ;
    float               gripperpos_last =0;
    char* datename;
    char* curvename;
    int t_send;
    RT_TASK*            m_MainCtrlTask;
    T265Camera          m_t265;
private:


};
}
#endif // ROBOT_H
