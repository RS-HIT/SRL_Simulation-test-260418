#ifndef RTCAN_H
#define RTCAN_H

#include <libpcan.h>
#include  "magneticencoder.h"
#include <vector>

class mitMotor;
class magneticEncoder;

class RT_TASK;

class rtCan
{
public:
    rtCan();
    void init(int port,float sensor_init[6]);
    int startCanRec(const char* taskname);
    void close();
    void canTx(int id, int len, BYTE* data);
    void canrx();
//    void canrx_sensor();
    //static void         signal_handler(int status);

    static void  canrxTask(void * arg);
    HANDLE       can_handle;
    void addmotor(int canid);
    void addencoder(int canid);

    RT_TASK*      canrx_task;
    std::vector<mitMotor*>     motors;
    std::vector<magneticEncoder*>    encoders;
//    std::vector<float> lh_sensor_angle;
//    std::vector<float> rh_sensor_angle ;
//    float lu_sensor_angle[6]= {0};
    float sensor_angle_init[6]= {0};

    int ccc1 =0;
    int ccc2 =0;
    int ccc3 =0;

    int ccc27 =0;
    int ccc28 =0;
    int ccc29 =0;
    float   m_TStep = 0.007;



};

#endif // RTCAN_H
