#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
//#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
//#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
//#include <boost/numeric/odeint/integrate/integrate_adaptive.hpp>
//#include <boost/numeric/odeint/stepper/generation/make_controlled.hpp>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <addons/urdfreader/urdfreader.h>
//using namespace boost::numeric::odeint;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
#define PI 3.1415926f
#define j1_limit1 -180*PI/180
#define j1_limit2 30*PI/180

#define j2_limit1 -85*PI/180
#define j2_limit2 120*PI/180

#define j3_limit1 -175*PI/180
#define j3_limit2 30*PI/180

#define j4_limit1 -90*PI/180
#define j4_limit2 90*PI/180

#define j5_limit1 -90*PI/180
#define j5_limit2 90*PI/180

#define j6_limit1 -90*PI/180
#define j6_limit2 90*PI/180

class dynamics
{
public:
    dynamics();
    bool init(const char* filename,const char* Linkname, Vector3d Dummy_pos);
    void inverseDynamics(    const VectorNd &Q,
                             const VectorNd &QDot,
                             const VectorNd &QDDot,
                             VectorNd &Tau,
                             std::vector<SpatialVector> *f_ext = NULL);

    void forwardDynamics (  const VectorNd &Q,
                            const VectorNd &QDot,
                            const VectorNd &Tau,
                            VectorNd &QDDot,
                            std::vector<SpatialVector> *f_ext = NULL);
    bool HoloInverseKinematics(Vector3d pos, Vector3d rpy,VectorNd& qres);
    bool inverseKinematics(Vector3d pos, Matrix3d ori,VectorNd& qres);
    bool inverseKinematics(Vector3d pos, Vector3d rpy,VectorNd& qres);
    void forwardKinematics(VectorNd qin, Vector3d& endpos,Matrix3d& endori);
    void forwardKinematics(VectorNd qin, Vector3d& endpos,Vector3d& endrpy);

    void getEndPose(Vector3d& pos, Matrix3d& ori);
    void getEndPose(Vector3d& pos, Vector3d& rpy);

    void setEndPose(const Vector3d pos, const Matrix3d ori);
    void setEndPose(const Vector3d pos, const Vector3d rpy);
    void getEndPosition(Vector3d& pos);
    void getEndOrientation(Vector3d& rpy);
    void getJacobi(Math::MatrixNd& G);
    void getendpos_f(Vector3d& pos);
    void getD435iPose(Vector3d& pos, Matrix3d& ori);


    void setJointPositions(const VectorNd jp);
    void getJointPositions(VectorNd& jp);
    double round_to_lmt(double v, double lmt_up ,double lmt_down);

    int getDofs();

    Matrix3d rpyToMatrix(Vector3d rpy);
    Vector3d endpoint;

private:

    Model* model;
    VectorNd    q;
    VectorNd    qd;
    VectorNd    qdd;
    VectorNd    tau;
    unsigned int    end;
    unsigned int    Link5;

};

#endif // dynamics_H
