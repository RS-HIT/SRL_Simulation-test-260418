#include "mujocosim.h"
#include <stdbool.h>
#include <math.h>
#include <cstdio>
#include <cstring>
//#include "include/glfw3.h"
static mujocosim *A;
#define PI 3.141592654f


mujocosim::mujocosim()
{
    m = NULL;
    d = NULL;
    A = this;
    motor_angle_left.resize(2);
    motor_angle_left << 0,0;
    motor_angle_right.resize(2);
    motor_angle_right << 0,0;


}

mujocosim::~mujocosim()
{
    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
  #if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
  #endif
}

void mujocosim::init(const char* file)
{
    char error[1000] = "Could not load binary model";
     // load and compile model
    m = mj_loadXML(file, 0, error, 1000);
    if (!m) {
      mju_error_s("Load model error: %s", error);
    }
    // make data
    d = mj_makeData(m);
    // init GLFW
//    if (!glfwInit()) {
//      mju_error("Could not initialize GLFW");
//    }
}

void mujocosim::motorcontrol(const mjModel* m, mjData* d)
{
    A->d->ctrl[0] = A->motor_angle_left[0];   //lj1
    A->d->ctrl[2] = A->motor_angle_right[0];   //rj1
    A->d->ctrl[4] = A->motor_angle_left[1];   //lj2
    A->d->ctrl[6] = A->motor_angle_right[1];   //rj2
    // printf("%f %f %f %f\n", A->motor_angle_left[0], A->motor_angle_left[1],A->motor_angle_right[0],A->motor_angle_right[1]);
    // A->d->ctrl[0] = -0.8*(A->d->sensordata[0]-A->motor_angle_left[0])-0.09*A->d->sensordata[1];//lj1
    // A->d->ctrl[1] = -0.8*(A->d->sensordata[2]-A->motor_angle_right[0])-0.09*A->d->sensordata[3];//rj1
    // A->d->ctrl[2] = -0.8*(A->d->sensordata[4]-A->motor_angle_left[1])-0.09*A->d->sensordata[5];//lj2
    // A->d->ctrl[3] = -0.8*(A->d->sensordata[6]-A->motor_angle_right[1])-0.09*A->d->sensordata[7];//rj2
    //printf("%f %f %f %f\n", A->d->ctrl[0], A->d->ctrl[1],A->d->ctrl[2],A->d->ctrl[3]);
    //printf("%f %f %f %f\n", A->d->sensordata[0],A->d->sensordata[1],A->d->sensordata[2],A->d->sensordata[3]);
}

// keyboard callback
void mujocosim::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(A->m, A->d);
    mj_forward(A->m, A->d);
  }

  if  (act==GLFW_PRESS && key==GLFW_KEY_SPACE)
  {
      A->pause = !A->pause; 
  }
  if (key==GLFW_KEY_RIGHT)
  {
      A->one_step = true;
  }
  if (key == GLFW_KEY_DOWN)
  {
    A->count_loop =10;
  }

  if (key == GLFW_KEY_Q)
  {
    A->d->mocap_pos[0]=A->d->mocap_pos[0]+0.001;
  }
  if (key == GLFW_KEY_A)
  {
    A->d->mocap_pos[0]=A->d->mocap_pos[0]-0.001;
  }
  if (key == GLFW_KEY_W)
  {
    A->d->mocap_pos[2]=A->d->mocap_pos[2]+0.001;
  }
  if (key == GLFW_KEY_S)
  {
    A->d->mocap_pos[2]=A->d->mocap_pos[2]-0.001;
  }

}


// mouse button callback
void mujocosim::mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  A->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  A->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  A->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &A->lastx, &A->lasty);
}

// mouse move callback
void mujocosim::mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!A->button_left && !A->button_middle && !A->button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - A->lastx;
  double dy = ypos - A->lasty;
  A->lastx = xpos;
  A->lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (A->button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (A->button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(A->m, action, dx/height, dy/height, &A->scn, &A->cam);
}

// scroll callback
void mujocosim::scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(A->m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &A->scn, &A->cam);
}

void mujocosim::createWindows()
{
    // create window, make OpenGL context current, request v-sync
    windows = glfwCreateWindow(1200, 900, "bipedal", NULL, NULL);
    glfwMakeContextCurrent(windows);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);


    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(windows, keyboard);
    glfwSetCursorPosCallback(windows, mouse_move);
    glfwSetMouseButtonCallback(windows, mouse_button);
    glfwSetScrollCallback(windows, scroll);


    double viewarray[6]={-93.400000,-4.200000,0.985387,0.009696,-0.019427,-0.116378};
    cam.azimuth=viewarray[0];
    cam.elevation=viewarray[1];
    cam.distance=viewarray[2];
    cam.lookat[0]=viewarray[3];
    cam.lookat[1]=viewarray[4];
    cam.lookat[2]=viewarray[5];
    mjcb_control = motorcontrol;
}


void mujocosim::mujocoUpdate()
{
    
    mj_forward(m, d);
//    mj_f(m, d);
    //printf("%f,%f,%f,%f,%f,%f\n",cam.azimuth,cam.elevation,cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);
//    mj_step(m,d);
//    mj_inverse(m,d);
}

void mujocosim::mujocoSceneUpdate()
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(windows, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(windows);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}
void mujocosim::mujocoprintModel(const char*filename)
{
    mj_printModel(m,filename);
}
void mujocosim::mujocoprintData(const char*filename)
{
    mj_printData(m,d,filename);
}

bool mujocosim::detect_foot_contact(const int foot_id)
{
    if (d->ncon==0)
    {
      return false;
    }
    else
    {
      for (int i=0; i < d->ncon ;i++)
      {
          if (d->contact[i].geom2 == foot_id)
          {
            return true;
          }

      }
      return false;

    }

}



