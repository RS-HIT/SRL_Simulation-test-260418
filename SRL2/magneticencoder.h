#ifndef MAGNETICENCODER_H
#define MAGNETICENCODER_H
class rtCan;

class magneticEncoder
{
public:
    magneticEncoder();  //空的构造函数
    bool init(int canid, rtCan* canComm);
    void unpack_reply(unsigned char data[8],float init_angle,int id);
    void unpack_reply1(unsigned char data[8],float init_angle,int id);
    void unpack_reply2(unsigned char data[8],float init_angle);
    void getPos(float &p);
    void setZeroPos();
    int id;
    rtCan* m_pComm;
    float pos_init;
    float pos;
    float pos_last;
//    private:
//        float pos;
};

#endif // MAGNETICENCODER_H
