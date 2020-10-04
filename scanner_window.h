#include <X11/Xutil.h>
#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <deque>
#ifdef Success //this is stupid
  #undef Success
#endif
#include <eigen3/Eigen/Dense>



#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1024
#define STEREO_DISPARITY .068 //milimeters.
#define EPS 1e-7

using namespace Eigen;

typedef struct {
    float theta_l, theta_r, phi_l, phi_r;
} ProjectedPoints;

typedef struct{
    int xl, yl, xr, yr;
} StereoPixels;

typedef struct{
    float x, y, z;
} Point3D;

typedef struct{
    Vector3f pos;
    float radius;
} Sphere;

typedef struct{
    Vector3f pos;
    Matrix3f rot;
    Vector3f vertices;
} RigidBody;



void init_window();
void del_window();
void* window_thread(void*);
int is_window_alive();


int rainbow(int c);
Matrix3f get_rotation(float x, float y, float z);
ProjectedPoints project_point(Vector3f point_v, Vector3f view_v);
StereoPixels projection_to_pixels(ProjectedPoints pp);
void draw_stereo_line(Vector3f start_point, Vector3f end_point);
void draw_stereo_point(Vector3f point);
void draw_stereo_sphere(Sphere sphere);


void runge_kutta(float *X, float *Xt1, void (ode)(float*,float*), float ts, int len);
void lorenz_ode(float *X, float *Xd);