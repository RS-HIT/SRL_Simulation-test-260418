#include "dynamics.h"
//====================================================================
// Boost stuff
//====================================================================
double dynamics::round_to_lmt(double v, double lmt_up ,double lmt_down)
{
    if(v >= lmt_up) v = lmt_up;
    if (v <= lmt_down) v = lmt_down;
    return v;
}

//void f(const state_type &x, state_type &dxdt, const double t);
Matrix3d dynamics::rpyToMatrix(Vector3d rpy)
{
    Eigen::Matrix3d matrix;

    Matrix3d x = Eigen::AngleAxisd(rpy[0],Vector3d(-1,0,0)).toRotationMatrix();
    Matrix3d y = Eigen::AngleAxisd(rpy[1],Vector3d(0,-1,0)).toRotationMatrix();
    Matrix3d z = Eigen::AngleAxisd(rpy[2],Vector3d(0,0,-1)).toRotationMatrix();

    matrix=  z*y*x;

//    matrix = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) *
//            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
//            Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
    //std::cout<<matrix.transpose()<<std::endl;
    return matrix;
}

void dynamics::inverseDynamics(    const VectorNd &Q,
                                   const VectorNd &QDot,
                                   const VectorNd &QDDot,
                                   VectorNd &Tau,
                                   std::vector<SpatialVector> *f_ext)
{
    UpdateKinematics(*model, q, qd,qdd);
//    std::cout<<"q"<<q<<std::endl;
//     std::cout<<"qd"<<qd<<std::endl;
//      std::cout<<"qdd"<<qdd<<std::endl;
    InverseDynamics(*model,Q,QDot,QDDot,Tau,f_ext);
}

void dynamics::forwardDynamics (
        const VectorNd &Q,
        const VectorNd &QDot,
        const VectorNd &Tau,
        VectorNd &QDDot,
        std::vector<SpatialVector> *f_ext )
{
    ForwardDynamics (*model,Q,QDot,Tau,QDDot);
}
bool dynamics::HoloInverseKinematics(Vector3d pos, Vector3d rpy,VectorNd& qres)
{
    //Vector3d local_point = Vector3d::Zero();
    Matrix3d ori=rpyToMatrix(rpy);
    InverseKinematicsConstraintSet cs;
//    cs.AddPointConstraint(end,endpoint,pos);
    cs.AddFullConstraint(end,endpoint,pos,ori);

    bool result = InverseKinematics (*model, q, cs, qres);
    //q = qres;
    //printf("inverseKinematics result:%d\n",result);
    return true;
}

bool dynamics::inverseKinematics(Vector3d pos, Matrix3d ori,VectorNd& qres)
{
    VectorNd limit_up = VectorNd::Zero(6);
    VectorNd limit_down = VectorNd::Zero(6);
    limit_up   << j1_limit2,j2_limit2,j3_limit2,j4_limit2,j5_limit2,j6_limit2;
    limit_down   << j1_limit1,j2_limit1,j3_limit1,j4_limit1,j5_limit1,j6_limit1;
    InverseKinematicsConstraintSet cs;
//    cs.AddPointConstraint(end,endpoint,pos);
    cs.AddFullConstraint(end,endpoint,pos,ori);
    bool result = InverseKinematics (*model, q, cs, qres);
    assert(result==1);

//   if(arm[0].datename == "JointPosition1")
//    for(int i=0;i<6;i++)
//    {
//         qres[i] = round_to_lmt(qres[i],limit_up[i],limit_down[i]);
//    }
    q = qres;
    //printf("inverseKinematics result:%d\n",result);
    return true;
}

bool dynamics::inverseKinematics(Vector3d pos, Vector3d rpy,VectorNd& qres)
{
    return inverseKinematics(pos,rpyToMatrix(rpy),qres);
}

void dynamics::forwardKinematics(VectorNd qin, Vector3d& endpos,Matrix3d& endori)
{
    endpos =  CalcBodyToBaseCoordinates(*model,qin,end,endpoint);
    endori =  CalcBodyWorldOrientation(*model,qin,end);
}

void dynamics::forwardKinematics(VectorNd qin, Vector3d& endpos,Vector3d& endrpy)
{
    endpos =  CalcBodyToBaseCoordinates(*model,qin,end,endpoint);
    endrpy =  CalcBodyWorldOrientation(*model,qin,end).eulerAngles(0, 1, 2);
}

void dynamics::getEndPose(Vector3d& pos, Matrix3d& ori)
{
    pos =  CalcBodyToBaseCoordinates(*model,q,end,endpoint);
    ori =  CalcBodyWorldOrientation(*model,q,end);

    Vector3d euler = ori.eulerAngles(0, 1, 2);
}

void dynamics::getEndPose(Vector3d& pos, Vector3d& rpy)
{
    getEndPosition(pos);
    getEndOrientation(rpy);
}

void dynamics::getEndPosition(Vector3d& pos)
{
    pos =  CalcBodyToBaseCoordinates(*model,q,end,endpoint);
}
void dynamics::getEndOrientation(Vector3d& rpy)
{
    rpy =  CalcBodyWorldOrientation(*model,q,end).eulerAngles(0, 1, 2);
}

void dynamics::setEndPose(const Vector3d pos, const Matrix3d ori)
{
    VectorNd qres;
    inverseKinematics(pos,ori,qres);
}

void dynamics::setEndPose(const Vector3d pos, const Vector3d rpy)
{
    setEndPose(pos,rpyToMatrix(rpy));
}

void dynamics::setJointPositions(const VectorNd jp)
{
    q = jp;
}

void dynamics::getJointPositions(VectorNd& jp)
{
    jp = q;
}

int dynamics::getDofs()
{
    return model->dof_count;
}

void dynamics::getJacobi(Math::MatrixNd& G)
{
    CalcPointJacobian6D (*model,q,3,{0.3,0,0},G);
}

void dynamics::getendpos_f(Vector3d& pos)
{
     pos =  CalcBodyToBaseCoordinates(*model,q,3,{0.3,0,0});
}

void dynamics::getD435iPose(Vector3d& pos,Matrix3d& ori)
{
    pos =  CalcBodyToBaseCoordinates(*model,q,Link5,{0.0506, 0.0325, 0.05125});
    ori =  CalcBodyWorldOrientation(*model,q,Link5);
}

//void dynamics::getLinkPose(VectorNd& jp)
//{
//    jp = q;
//}

bool dynamics::init(const char* filename,const char* Linkname, Vector3d Dummy_pos )
{
    rbdl_check_api_version (RBDL_API_VERSION);

    model         = new Model();

    //3a. The URDF model is read in here, and turned into a series of
    if (!Addons::URDFReadFromFile (filename, model, false)) {
        std::cerr << "Error loading model " << filename << std::endl;
        abort();
    }

    //model->gravity = VeLink3ctor3d::Zero();
    std::cout << "Degree of freedom overview:" << std::endl;
    std::cout << Utils::GetModelDOFOverview(*model);

    std::cout << "Model Hierarchy:" << std::endl;
    std::cout << Utils::GetModelHierarchy(*model);
    int     dofs     = model->dof_count;
    //printf("DoF: %i\n",dofs);

    q = VectorNd::Zero(dofs);
    qd = VectorNd::Zero(dofs);
    qdd = VectorNd::Zero(dofs);
    UpdateKinematics(*model, q, qd,qdd);
    Link5 = model->GetBodyId("Link5");

    uint tip = model->GetBodyId(Linkname);
//     uint tip = model->GetBodyId("Link3");
    end = model->GetBodyId(Linkname);
//    Vector3d tiptobase = CalcBodyToBaseCoordinates(*model, q, tip, {0,0,0}, false);
//    endpoint = {0.295,0,0.03};//CalcBaseToBodyCoordinates(*model, q, end, tiptobase, false);
    endpoint =Dummy_pos;//CalcBaseToBodyCoordinates(*model, q, end, tiptobase, false);
    //birdhead
//    endpoint = {0.347f,0,0.03f};//CalcBaseToBodyCoordinates(*model, q, end, tiptobase, false);
    //birdhead_rob
//     endpoint = {0.352f,0,0.03f};//CalcBaseToBodyCoordinates(*model, q, end, tiptobase, false);
    //motion planing
//     endpoint = {0.278f,0,0.03f};//CalcBaseToBodyCoordinates(*model, q, end, tiptobase, false);
    //std::cout<<endpoint.transpose()<<std::endl;
    return true;

}

dynamics::dynamics()
{
    model = NULL;
}
