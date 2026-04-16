#ifndef MITMOTOR_H
#define MITMOTOR_H

#define CAN_PERIOD 1000000 /* 1 ms period */
//default motor PID
//#include <Eigen/Dense>
//using namespace Eigen;
#include <assert.h>
class rtCan;
class magneticEncoder;
#define PI 3.1415926f
#define POSMODE 1
#define VELMODE 2
#define TORMODE 3

class mitMotor
{
public:
    mitMotor();
    bool init(int canid, rtCan* canComm);
    void setMode(int mode);
    int getMode();
    /// CAN Command Packet Structure ///
    /// 16 bit position command, between -4*pi and 4*pi
    /// 12 bit velocity command, between -30 and + 30 rad/s
    /// 12 bit kp, between 0 and 500 N-m/rad
    /// 12 bit kd, between 0 and 100 N-m*s/rad
    /// 12 bit feed forward torque, between -18 and 18 N-m
    /// CAN Packet is 8 8-bit words
    /// Formatted as follows.  For each quantity, bit 0 is LSB
    /// 0: [position[15-8]]
    /// 1: [position[7-0]]
    /// 2: [velocity[11-4]]
    /// 3: [velocity[3-0], kp[11-8]]
    /// 4: [kp[7-0]]
    /// 5: [kd[11-4]]
    /// 6: [kd[3-0], torque[11-8]]
    /// 7: [torque[7-0]]

    void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff,unsigned char data[8]);

    /// CAN Reply Packet Structure ///
    /// 16 bit position, between -4*pi and 4*pi
    /// 12 bit velocity, between -30 and + 30 rad/s
    /// 12 bit current, between -40 and 40;
    /// CAN Packet is 5 8-bit words
    /// Formatted as follows.  For each quantity, bit 0 is LSB
    /// 0: [position[15-8]]
    /// 1: [position[7-0]]
    /// 2: [velocity[11-4]]
    /// 3: [velocity[3-0], current[11-8]]
    /// 4: [current[7-0]]

    void unpack_reply(unsigned char data[6]);

    void getState(float &pr,float &vr,float &tr);

    void fetchState(float &pr,float &vr,float &tr);

    void setCtrlPara(float kp, float kd){m_cmd.kp = kp;m_cmd.kd = kd;}
    int* protect(int type);
    int* sendCMD(float p_des, float v_des, float t_ff ,int type);
    void sendCMD_t(float t_ff = 0);
    void enableMotor(bool flag);

    void setZeroPos();

struct param
    {
        float zeroPos = 0.0f;
        int dir = 1;
    }m_param;

    struct ctrl
    {
        float kpp = 50.0f;
        float kpi =  2.0f;
        float ep = 0.0f;
        float epi = 0.0f;
        float ev = 0.0f;
        bool enabled = false;
    }m_ctrl;

    struct cmd
    {
        float pcmd = 0;
        float pcmd_last = 0;
        float vcmd = 0;
        float kp = 100.0f;
        float kd = 2.0f;
        float tcmd = 0;
    }m_cmd;

    int id;
    rtCan* m_pComm;
    magneticEncoder* encordor;
    float pos_limit= 6*PI/180;
    float cmdpos_limit= 850*PI/180;

    float cmdspeed_limit= 800*PI/180;
    float realspeed_limit= 800*PI/180;//520*PI/180;
    float tor_limit= 30;//6;//7.5;
    long m_txrxCount;
    float posr;
    float velr;
    float torr;
    float velr_last = 0;
    float posr_last = 0;
    float pcmd_last= 0;

    float vel_diff = 0;
    float vel_diff_last = 0;
    float vel_diff_f = 0;
    float posr_f = 0;
    float posr_f_last = 0;
//private:
//    float posr;
//    float posr_last = m_param.zeroPos;
//    float velr;
//    float torr;

};

#endif // MITMOTOR_H
