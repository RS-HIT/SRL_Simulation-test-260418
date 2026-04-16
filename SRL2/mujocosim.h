#ifndef MUJOCOSIM_H
#define MUJOCOSIM_H
#include "include/mujoco.h"
#include "include/glfw3.h"
#include "eigen-3.3.9/Eigen/StdVector"
#include "eigen-3.3.9/Eigen/Dense"
class mujocosim
{
public:
    mujocosim();
    ~mujocosim();
    void init(const char* file);
    static void motorcontrol(const mjModel* m,mjData* d);

    static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    static void mouse_button(GLFWwindow* window, int button, int act, int mods);
    static void mouse_move(GLFWwindow* window,double xpos, double ypos);
    static void scroll(GLFWwindow* window,double xoffset, double yoffset);
    void createWindows();
    void mujocoUpdate();
    void mujocoSceneUpdate();
    void mujocoprintModel(const char*filename);
    void mujocoprintData(const char*filename);
    bool detect_foot_contact(const int foot_id);


    // MuJoCo data structures
    mjModel* m = NULL;                  // MuJoCo model
    mjData* d = NULL;                   // MuJoCo data
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context
    // mouse interaction
    bool button_left = false;
    bool button_middle = false;
    bool button_right =  false;
    double lastx = 0;
    double lasty = 0;
    float x=0,y=0,z=0,phi=0,theta=0,psai=0;
    //ik M_IK;
    Eigen::VectorXd motor_angle_left;
    Eigen::VectorXd motor_angle_right;

    bool pause=false;
    bool one_step = false;
    int count_loop =0;
    GLFWwindow* windows;

};

#endif // MUJOCOSIM_H
