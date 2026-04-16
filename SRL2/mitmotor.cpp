#include "mitmotor.h"
#include "math.h"
#include "rtcan.h"
#include "iostream"
/// Value Limits for KA80-9///
#define P_MIN -95.5f
#define P_MAX 95.5f
#define V_MIN -22.5f
#define V_MAX 22.5f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define T_MIN -54.0f
#define T_MAX 54.0f

mitMotor::mitMotor()
{
    m_param.dir = 1.0f;
    m_param.zeroPos = 0.0f;
    posr = 0;
    velr = 0;
    torr = 0;
    m_txrxCount = 0;
}

bool mitMotor::init(int canid, rtCan* canComm)
{
    id = canid;
    m_pComm = canComm;
    return true;
}

float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
}

float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
}

float fmaxf3(float x, float y, float z){
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

float fminf3(float x, float y, float z){
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

void limit_norm(float *x, float *y, float limit){
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrt(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x * limit/norm;
        *y = *y * limit/norm;
    }
}


int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


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

void mitMotor::pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff,BYTE data[8])
{
    /// limit data to be within bounds ///
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
    kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    /// convert floats to unsigned ints ///
    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    /// pack ints into the can buffer ///
    data[0] = p_int>>8;
    data[1] = p_int&0xFF;
    data[2] = v_int>>4;
    data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
    data[4] = kp_int&0xFF;
    data[5] = kd_int>>4;
    data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
    data[7] = t_int&0xff;
    m_txrxCount++;
    if(m_txrxCount >= 10) rt_printf("ERROR : motor %d can recv missing %d!\n",id,m_txrxCount);
}

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

void mitMotor::unpack_reply(unsigned char data[6])
{
    /// unpack ints from can buffer ///
    int id = data[0];
    int p_int = (data[1]<<8)|data[2];
    int v_int = (data[3]<<4)|(data[4]>>4);
    int i_int = ((data[4]&0xF)<<8)|data[5];
    /// convert ints to floats ///
    posr_last = posr;
    velr_last = velr;
    posr = uint_to_float(p_int, P_MIN, P_MAX, 16);
    velr = uint_to_float(v_int, V_MIN, V_MAX, 12);
    torr = uint_to_float(i_int, T_MIN, T_MAX, 12);

    vel_diff = (posr-posr_last)/0.002;
    vel_diff_f = 0.9*vel_diff_last + 0.1*vel_diff;
    vel_diff_last = vel_diff;
    m_txrxCount--;
}
int* mitMotor::protect(int type)
{
//    BYTE data_off[8];
    static int back[3]={0,0,0};
    if(type==1)
    {
        //  位置跟踪误差保护
        bool flag1=fabs(m_cmd.pcmd - posr) >= pos_limit;
        if(flag1)
        {
            std::cout<<"motor"<<id <<":over position_tracking_error_limit"<<std::endl;
            m_ctrl.enabled=false;
            back[0]=1;

        }
    }

    bool flag2=fabs((posr -posr_last)/0.005) >= realspeed_limit;
    bool flag3=fabs(torr) >= tor_limit;
    //rt_printf("motor %d torr: %0.3f\n",id,torr);
    bool flag=flag2||flag3;

    if(flag)
    {
        if(flag2)
        {
            std::cout<<"motor"<<id<<":over realspeed_limit"<<std::endl;
            m_ctrl.enabled=false;
            back[1]=1;
        }
        if(flag3)
        {
            std::cout<<"motor"<<id<<":over realtorque_limit. torr = "<<torr<<std::endl;
            m_ctrl.enabled=false;
            back[2]=1;
        }

    }
    return back;

}


//更改
int* mitMotor::sendCMD(float p_des, float v_des, float t_ff,int type)
{
     static int back[3]={0,0,0};
    if(m_ctrl.enabled)
    {
        BYTE data[8];
        if(type==1)
        {

            m_ctrl.ep = p_des - ( posr- m_param.zeroPos) / m_param.dir;
            m_ctrl.epi = m_ctrl.epi + m_ctrl.ep;
            m_ctrl.epi = fminf(fmaxf(-1.0f, m_ctrl.epi), 1.0f);
            float fricStatic = 0.03f;
            float pcmd = p_des;
            float v_err = m_ctrl.kpp*m_ctrl.ep+m_ctrl.kpi*m_ctrl.epi;

            float vcmd = v_des + v_err;

//            float vcmd = v_des;
            float t_fric = (v_des>0)?fricStatic:-fricStatic;
            float tcmd = 0;//t_ff + t_fric;

            m_cmd.pcmd = pcmd * m_param.dir + m_param.zeroPos;
            m_cmd.vcmd = vcmd * m_param.dir;
            m_cmd.tcmd = tcmd * m_param.dir;


            bool flag1=fabs((m_cmd.pcmd - m_cmd.pcmd_last)/0.005) >= cmdspeed_limit;
            bool flag2=fabs(m_cmd.tcmd) >= tor_limit;
            bool flag = flag1 || flag2;
            if(flag)
            {

                if(flag1)
                {
                    std::cout<<"motor"<<id <<":over cmdspeed_limit"<<", pcmd="<<m_cmd.pcmd<<", pcmd_last="<<m_cmd.pcmd_last<<std::endl;
                    back[0] = 1;
                    //rt_printf("motor[%d],m_cmd.pcmd:%0.3f,m_cmd,pcmd_last:%0.3f\n",id,m_cmd.pcmd,m_cmd.pcmd_last);
                }
                if(flag2)
                {
                    std::cout<<"motor:"<<id <<"over cmdtorque_limit"<<std::endl;
                    back[1] = 1;
                }
            }
            else
            {
                pack_cmd(m_cmd.pcmd,m_cmd.vcmd,m_cmd.kp,m_cmd.kd,m_cmd.tcmd,data);
//              rt_printf("1111motor[%d],m_cmd.pcmd:%0.3f,m_cmd,pcmd_last:%0.3f\n",id,m_cmd.pcmd,m_cmd.pcmd_last);
//              rt_printf("motor:%d pcmd=%0.3f,vd=%0.3f,td=%0.3f,ep=%0.3f,epi=%0.3f,posr=%0.3f,velr=%0.3f,torr=%0.3f,kp=%0.3f,kd=%0.3f\n",id,m_cmd.pcmd,m_cmd.vcmd,m_cmd.tcmd,m_ctrl.ep,m_ctrl.epi,posr,velr,torr,m_cmd.kp,m_cmd.kd);
//              std::cout<<"m_ctrl.ep:"<<m_ctrl.ep <<std::endl;
//              std::cout<<"posr:"<< posr <<std::endl;
                m_pComm->canTx(id,8,data);
                m_cmd.pcmd_last = m_cmd.pcmd;

            }

        }
        else if(type==2)
        {
            int v=2;
        }
        else
        {
            float t_fric = 0;
            if(id == 1)
            {
//                t_fric = (vel_diff_f>0)?(0.0008*vel_diff_f+0.1313):(0.0011*vel_diff_f-0.1558);
                t_fric = (vel_diff_f>0)?(0.06):(-0.05);
//                cout<<"vel_diff_f:"<<vel_diff_f<<endl;
            }
            if(id == 2)
            {
//                t_fric = (vel_diff_f>0)?(0.0055*vel_diff_f+0.1764):(0.0063*vel_diff_f-0.2001);
                 t_fric = (vel_diff_f>0)?(0.12):(-0.15);
            }
            if(id == 3)
            {
//                t_fric = (vel_diff_f>0)?(0.0007*vel_diff_f+0.1551):(0.0006*vel_diff_f-0.1294);
                 t_fric = (vel_diff_f>0)?(0.15):(-0.08);
            }
            t_fric = 0.0;
            m_cmd.tcmd = t_ff * m_param.dir + t_fric;
            bool flag1=fabs( m_cmd.tcmd) >= tor_limit || isnan(m_cmd.tcmd);
            if(flag1)
            {
                std::cout<<"motor:"<<id <<"over cmd torque_limit"<<m_cmd.tcmd<< " torr: " <<torr<<std::endl;
                back[0] = 1;
            }
            else
            {
                pack_cmd(0,0,0,0,m_cmd.tcmd,data);
                m_pComm->canTx(id,8,data);
                m_cmd.pcmd = posr;
                m_cmd.pcmd_last = m_cmd.pcmd;
            }


        }

    }
    return back;
}




void mitMotor::sendCMD_t(float t_ff)
{
    if(m_ctrl.enabled)
    {
        BYTE data[8];
        m_cmd.tcmd = t_ff * m_param.dir;
        pack_cmd(0,0,0,0, m_cmd.tcmd,data);

        //        rt_printf(" id:%d, m_cmd.tcmd:%0.3f\n",id,m_cmd.tcmd);
        // real volocity protect
        if(abs(posr - posr_last) >= pos_limit*PI/180)
        {
            BYTE data_off[8];
            data_off[0] = 0xFF;
            data_off[1] = 0xFF;
            data_off[2] = 0xFF;
            data_off[3] = 0xFF;
            data_off[4] = 0xFF;
            data_off[5] = 0xFF;
            data_off[6] = 0xFF;
            data_off[7] = 0xFD;
            for(int i=0;i<3;i++)
            {
                m_pComm->canTx(i+1,8,data_off);
            }

            for(int i=0;i<3;i++)
            {
                BYTE data_zero[8];
                pack_cmd(0,0,0,0,0,data_zero);
                m_pComm->canTx(i+1,8,data_zero);
            }
            std::cout<<"over speed"<<std::endl;

        }
        // torque protect
        if(abs(torr) >= tor_limit)
        {

            BYTE data_off[8];
            data_off[0] = 0xFF;
            data_off[1] = 0xFF;
            data_off[2] = 0xFF;
            data_off[3] = 0xFF;
            data_off[4] = 0xFF;
            data_off[5] = 0xFF;
            data_off[6] = 0xFF;
            data_off[7] = 0xFD;
            for(int i=0;i<3;i++)
            {
                m_pComm->canTx(i+1,8,data_off);
            }

            for(int i=0;i<3;i++)
            {
                BYTE data_zero[8];
                pack_cmd(0,0,0,0,0,data_zero);
                m_pComm->canTx(i+1,8,data_zero);
            }
            std::cout<<"over torque"<<std::endl;
        }

        m_pComm->canTx(id,8,data);
        m_cmd.pcmd =  posr;
        posr_last = posr;
    }
}

void mitMotor::getState(float &pr,float &vr,float &tr)
{
    vr = 0;
    tr = 0;
    pr = (posr - m_param.zeroPos) / m_param.dir;
    if( m_ctrl.enabled)
    {
        vr = velr / m_param.dir;
        tr = torr / m_param.dir;
    }
}

void mitMotor::fetchState(float &pr,float &vr,float &tr)
{
    BYTE data[8];
    getState(pr,vr,tr);

    bool flag=fabs((m_cmd.pcmd - m_cmd.pcmd_last)/0.005) >= cmdspeed_limit;
    if(flag)
    {
        std::cout<<"fetch state motor:"<<id <<"over cmdspeed_limit"<<std::endl;
    }
    else
    {
//        pack_cmd(m_cmd.pcmd_last,0,m_cmd.kp,m_cmd.kd,0,data);
           pack_cmd(m_cmd.pcmd,m_cmd.vcmd,m_cmd.kp,m_cmd.kd,m_cmd.tcmd,data);
        m_pComm->canTx(id,8,data);
        m_cmd.pcmd_last = m_cmd.pcmd;

    }
//    rt_printf("motor %d torr: %0.3f\n",id,torr);



//    pack_cmd(m_cmd.pcmd_last,0,m_cmd.kp,m_cmd.kd,0,data);
//    m_pComm->canTx(id,8,data);
//    m_cmd.pcmd_last = m_cmd.pcmd;
    if(m_txrxCount >= 10) rt_printf("ERROR : motor %d can recv missing %d!\n",id,m_txrxCount);

}

void mitMotor::enableMotor(bool flag)
{
    m_ctrl.enabled = flag;
    m_ctrl.epi = 0;
    m_ctrl.ep = 0;
    m_ctrl.ev = 0;
    rt_printf("motor %d m_cmd.pcmd:%0.3f,posr:%0.3f\n",id, m_cmd.pcmd , posr);
    m_cmd.pcmd = posr;
    m_cmd.vcmd = 0;
    m_cmd.tcmd = 0;
    BYTE data[8];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = flag?0xFC:0xFD;
    m_pComm->canTx(id,8,data);
    m_txrxCount++;
    if(m_txrxCount >= 10) rt_printf("ERROR : motor %d can recv missing %d!\n",id,m_txrxCount);
}

void mitMotor::setZeroPos()
{
    m_param.zeroPos = posr;
}
