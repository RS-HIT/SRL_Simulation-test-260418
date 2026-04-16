#ifndef T265CAMERA_H
#define T265CAMERA_H
#include <librealsense2/rs.hpp>
#include <Eigen/Geometry>
using namespace Eigen;

class T265Camera
{
public:
    T265Camera();
    bool getPose(Vector3f  &p,Vector3f &pry,Quaternionf &q);
    bool getPose(Vector3f &p,Vector3f &pry,Quaternionf &q,Vector3f &v,Vector3f &a);
    void init();
    void init(int ID);
    bool getPose(Vector3f &p,Vector3f &pry,Quaternionf &q,Vector3f &v,Vector3f &a,int ID);
private:
    rs2::pipeline m_Pipe;   
    rs2::pipeline m_Pipe1;
    rs2::pipeline m_Pipe2;
    rs2::pipeline m_Pipe3;
    rs2::pipeline m_Pipe4;
    rs2::pipeline m_Pipe5;
};

#endif // T265CAMERA_H
