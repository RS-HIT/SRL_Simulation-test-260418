#include "t265camera.h"
#include "iostream"
#include "math.h"
using namespace std;

T265Camera::T265Camera()
{

}
void T265Camera::init(int ID)
{
    rs2::context                          ctx;        // Create librealsense context for managing devices
    std::vector<std::string>              serials;
    rs2::config cfg;
    if(ID==1)
    {
        for (auto&& dev : ctx.query_devices())
        {
            const char* result = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            serials.push_back(result);
            cout <<"serials mumber:" << result << endl;
        }

        try{
            cfg.enable_device("146322110087");
            cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
            m_Pipe1.start(cfg);
             rt_printf("back T265 connected success\n");
        }catch(...)
        {
            rt_printf("no back T265 connected\n");
        }
    }

    if(ID==2)
    {
        try{
            cfg.enable_device("146322110281");
            cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
            m_Pipe2.start(cfg);
            rt_printf("lin1_r T265 connected success\n");
        }catch(...)
        {
            rt_printf("no lin1_r T265 connected\n");
        }
     }

    if(ID==3)
    {
        try{
            cfg.enable_device("911412110838");
            cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
            m_Pipe3.start(cfg);
            rt_printf("lin2_r T265 connected success\n");
        }catch(...)
        {
            rt_printf("no lin2_r T265 connected\n");
        }
     }
    if(ID==4)
    {
        try{
            cfg.enable_device("224622112195");
            cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
            m_Pipe4.start(cfg);
             rt_printf("lin1_l T265 connected success\n");

        }catch(...)
        {
            rt_printf("no lin1_l T265 connected\n");
        }
     }
    if(ID==5)
    {
        try{
            cfg.enable_device("230322110556");
            cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
            m_Pipe5.start(cfg);
             rt_printf("lin2_l T265 connected success\n");

        }catch(...)
        {
            rt_printf("no lin2_l T265 connected\n");
        }
     }




}

void T265Camera::init()
{
    rs2::context                          ctx;        // Create librealsense context for managing devices
    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
    {
        const char* result = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        serials.push_back(result);
        cout <<"serials mumber:" << result << endl;
    }
    rs2::config cfg;
    try{

       // cfg.enable_device("011622110824"); // 宝哥
        cfg.enable_device("146322110281");
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        m_Pipe1.start(cfg);
    }catch(...)
    {
        printf("no T2651 connected\n");
    }

    try{
//       cfg.enable_device("146322110281");
         cfg.enable_device("911412110838");
         cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
         m_Pipe2.start(cfg);
    }catch(...)
    {
        printf("no T2652 connected\n");
    }
}

bool T265Camera::getPose(Vector3f &p,Vector3f &pry,Quaternionf &q,Vector3f &v,Vector3f &a,int ID)
{
    if(ID==1)
    {
        // Wait for the next set of frames from the camera
        rs2::frameset frames;
        bool  ret = m_Pipe1.poll_for_frames(&frames);
        //printf("T265Camera m_Pipe.poll_for_frames ret:%d\n",ret);
        if(ret)
        {
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            // Cast the frame to pose_frame and get its data
            rs2_pose pose_data = f.as<rs2::pose_frame>().get_pose_data();

            p[1] = -pose_data.translation.x;
            p[2] =  pose_data.translation.y;
            p[0] = -pose_data.translation.z;

            v[1] = -pose_data.velocity.x;
            v[2] =  pose_data.velocity.y;
            v[0] = -pose_data.velocity.z;

            a[1] = -pose_data.acceleration.x;
            a[2] =  pose_data.acceleration.y;
            a[0] = -pose_data.acceleration.z;



            q.w() =  pose_data.rotation.w;
            q.x() =  -pose_data.rotation.z;
            q.y() =  pose_data.rotation.x;
            q.z() =  -pose_data.rotation.y;

            // //PRY转换 y->pitch:angle[0];x->roll:angle[1];z->yaw:angle[2
            pry[0] = -asin(2.0 * (q.x() * q.z()- q.w() * q.y()));
            pry[1] =  -atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
            pry[2] =  atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
        }
        return ret;
    }
    else if(ID==2)
    {
        // Wait for the next set of frames from the camera
        rs2::frameset frames;
        bool  ret = m_Pipe2.poll_for_frames(&frames);
        //printf("T265Camera m_Pipe.poll_for_frames ret:%d\n",ret);
        if(ret)
        {
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            // Cast the frame to pose_frame and get its data
            rs2_pose pose_data = f.as<rs2::pose_frame>().get_pose_data();

            p[1] = -pose_data.translation.x;
            p[2] =  pose_data.translation.y;
            p[0] = -pose_data.translation.z;

            v[1] = -pose_data.velocity.x;
            v[2] =  pose_data.velocity.y;
            v[0] = -pose_data.velocity.z;

            a[1] = -pose_data.acceleration.x;
            a[2] =  pose_data.acceleration.y;
            a[0] = -pose_data.acceleration.z;



            q.w() =  pose_data.rotation.w;
            q.x() =  -pose_data.rotation.z;
            q.y() =  -pose_data.rotation.x;
            q.z() =  pose_data.rotation.y;

            // //PRY转换 y->pitch:angle[0];x->roll:angle[1];z->yaw:angle[2
            pry[0] = -asin(2.0 * (q.x() * q.z()- q.w() * q.y()));
            pry[1] =  -atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
            pry[2] =  atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
        }
        return ret;
    }
    else if(ID==3)
    {
        // Wait for the next set of frames from the camera
        rs2::frameset frames;
        bool  ret = m_Pipe3.poll_for_frames(&frames);
        //printf("T265Camera m_Pipe.poll_for_frames ret:%d\n",ret);
        if(ret)
        {
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            // Cast the frame to pose_frame and get its data
            rs2_pose pose_data = f.as<rs2::pose_frame>().get_pose_data();

            p[1] = -pose_data.translation.x;
            p[2] =  pose_data.translation.y;
            p[0] = -pose_data.translation.z;

            v[1] = -pose_data.velocity.x;
            v[2] =  pose_data.velocity.y;
            v[0] = -pose_data.velocity.z;

            a[1] = -pose_data.acceleration.x;
            a[2] =  pose_data.acceleration.y;
            a[0] = -pose_data.acceleration.z;



            q.w() =  pose_data.rotation.w;
            q.x() =  -pose_data.rotation.z;
            q.y() =  -pose_data.rotation.x;
            q.z() =   pose_data.rotation.y;

            // //PRY转换 y->pitch:angle[0];x->roll:angle[1];z->yaw:angle[2
            pry[0] = -asin(2.0 * (q.x() * q.z()- q.w() * q.y()));
            pry[1] =  -atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
            pry[2] =  atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
        }
        return ret;
    }
    else if(ID==4)
    {
        // Wait for the next set of frames from the camera
        rs2::frameset frames;
        bool  ret = m_Pipe4.poll_for_frames(&frames);
        //printf("T265Camera m_Pipe.poll_for_frames ret:%d\n",ret);
        if(ret)
        {
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            // Cast the frame to pose_frame and get its data
            rs2_pose pose_data = f.as<rs2::pose_frame>().get_pose_data();

            p[1] = -pose_data.translation.x;
            p[2] =  pose_data.translation.y;
            p[0] = -pose_data.translation.z;

            v[1] = -pose_data.velocity.x;
            v[2] =  pose_data.velocity.y;
            v[0] = -pose_data.velocity.z;

            a[1] = -pose_data.acceleration.x;
            a[2] =  pose_data.acceleration.y;
            a[0] = -pose_data.acceleration.z;



            q.w() =  pose_data.rotation.w;
            q.x() =  -pose_data.rotation.z;
            q.y() =  pose_data.rotation.x;
            q.z() =  -pose_data.rotation.y;

            // //PRY转换 y->pitch:angle[0];x->roll:angle[1];z->yaw:angle[2
            pry[0] = -asin(2.0 * (q.x() * q.z()- q.w() * q.y()));
            pry[1] =  -atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
            pry[2] =  atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
        }
        return ret;
    }
    else if(ID==5)
    {
        // Wait for the next set of frames from the camera
        rs2::frameset frames;
        bool  ret = m_Pipe5.poll_for_frames(&frames);
        //printf("T265Camera m_Pipe.poll_for_frames ret:%d\n",ret);
        if(ret)
        {
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            // Cast the frame to pose_frame and get its data
            rs2_pose pose_data = f.as<rs2::pose_frame>().get_pose_data();

            p[1] = -pose_data.translation.x;
            p[2] =  pose_data.translation.y;
            p[0] = -pose_data.translation.z;

            v[1] = -pose_data.velocity.x;
            v[2] =  pose_data.velocity.y;
            v[0] = -pose_data.velocity.z;

            a[1] = -pose_data.acceleration.x;
            a[2] =  pose_data.acceleration.y;
            a[0] = -pose_data.acceleration.z;



            q.w() =  pose_data.rotation.w;
            q.x() =  -pose_data.rotation.z;
            q.y() =  pose_data.rotation.x;
            q.z() =  -pose_data.rotation.y;

            // //PRY转换 y->pitch:angle[0];x->roll:angle[1];z->yaw:angle[2
            pry[0] = -asin(2.0 * (q.x() * q.z()- q.w() * q.y()));
            pry[1] =  -atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
            pry[2] =  atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
        }
        return ret;
    }
}



bool T265Camera::getPose(Vector3f &p,Vector3f &pry,Quaternionf &q,Vector3f &v,Vector3f &a)
{
    rs2::frameset frames;
    // Wait for the next set of frames from the camera
    bool  ret = m_Pipe.poll_for_frames(&frames);
    //printf("T265Camera m_Pipe.poll_for_frames ret:%d\n",ret);
    if(ret)
    {
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        rs2_pose pose_data = f.as<rs2::pose_frame>().get_pose_data();

        p[1] = -pose_data.translation.x;
        p[2] = pose_data.translation.y;
        p[0] = -pose_data.translation.z;

        v[1] = -pose_data.velocity.x;
        v[2] = pose_data.velocity.y;
        v[0] = -pose_data.velocity.z;

        a[1] = -pose_data.acceleration.x;
        a[2] = pose_data.acceleration.y;
        a[0] = -pose_data.acceleration.z;

        q.w() = pose_data.rotation.w;
        q.x() = -pose_data.rotation.z;
        q.y() = pose_data.rotation.x;
        q.z() = -pose_data.rotation.y;

        // //PRY转换 y->pitch:angle[0];x->roll:angle[1];z->yaw:angle[2]
//        pry[0] = -asin(2.0 * (q.x() * q.z()- q.w() * q.y())) * 180.0 / 3.1416;
//        pry[1] =  -atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z()) * 180.0 / 3.1416;
//        pry[2] =  atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() - q.z() * q.z()) * 180.0 / 3.1416;
        pry[0] = -asin(2.0 * (q.x() * q.z()- q.w() * q.y()));
        pry[1] =  -atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
        pry[2] =  atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    }
    return ret;
}

//--------jing
bool T265Camera::getPose(Vector3f &p,Vector3f &pry,Quaternionf &q)
{
    rs2::frameset frames;
    // Wait for the next set of frames from the camera
    bool  ret = m_Pipe.poll_for_frames(&frames);
    //printf("T265Camera m_Pipe.poll_for_frames ret:%d\n",ret);
    if(ret)
    {
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        rs2_pose pose_data = f.as<rs2::pose_frame>().get_pose_data();

        /*p[1] = -pose_data.translation.x;
        p[2] = pose_data.translation.y;
        p[0] = -pose_data.translation.z;*/

        p[0] = pose_data.translation.x;
        p[1] = pose_data.translation.y;
        p[2] = pose_data.translation.z;

//        q.w() = pose_data.rotation.w;
//        q.x() = -pose_data.rotation.z;
//        q.y() = pose_data.rotation.x;
//        q.z() = -pose_data.rotation.y;

        q.w() = pose_data.rotation.w;
        q.x() = pose_data.rotation.x;
        q.y() = pose_data.rotation.y;
        q.z() = pose_data.rotation.z;

        // //PRY转换 y->pitch:angle[0];x->roll:angle[1];z->yaw:angle[2]
        pry[0] = -asin(2.0 * (q.x() * q.z()- q.w() * q.y())) * 180.0 / 3.1416;
        pry[1] =  -atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z()) * 180.0 / 3.1416;
        pry[2] =  atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() - q.z() * q.z()) * 180.0 / 3.1416;
       /* pry[0] = -asin(2.0 * (q.x() * q.z()- q.w() * q.y()));
        pry[1] =  -atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
        pry[2] =  atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() - q.z() * q.z());*/

    }
    return ret;
}
