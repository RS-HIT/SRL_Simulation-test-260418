#ifndef SENSOR_H
#define SENSOR_H
class megneticEncoder
{
   public:
    megneticEncoder();

   void unpack_reply();
   private:
        float pos;
        long m_txrxCount_sensor;
};


#endif // SENSOR_H
