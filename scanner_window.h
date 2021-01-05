#pragma once

#include <X11/keysymdef.h>
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
#include "super_scanner.h"



#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1024
#define STEREO_DISPARITY .068 //milimeters.
#define EPS 1e-7


#define SCANNER_3D_MENU 0
#define SCAN_PATH_MENU 1
#define NODE_MASS_MENU 2
#define NODE_DAMPING_MENU 3
#define CONNECTIVITY_MENU 4
#define SCANNER_2D_MENU 5
#define CONSTRAINT_MENU 6
#define EQ_POS_MENU 7
#define INFO_MENU 8
#define ADSR_MENU 9


//xdrawstring character width is 6 pixels. height I think is 8


using namespace Eigen;

typedef struct {
  float theta_l, theta_r, phi_l, phi_r;
} ProjectedStereoPoints;

typedef struct {
  float theta, phi;
} ProjectedPoint;

typedef struct{
  int xl, yl, xr, yr;
} StereoPixels;

typedef struct{
  int x, y;
} MonoPixel;

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



void init_window(SuperScanner *s);
void del_window();
void* window_thread(void*);


void draw_scanner(Display *dpy, Window w, GC gc, int mono);
void handle_scanner_menu(Display *dpy, Window w, GC gc, int &menu_id, int &mono);

void handle_adsr_menu(Display *dpy, Window w, GC gc, int &menu_id);
void handle_eq_pos_menu(Display *dpy, Window w, GC gc, int &node_sel_x, int &node_sel_y, int &menu_id);
void handle_scanner_2d_menu(Display *dpy, Window w, GC gc, int &menu_id);
void handle_connectivity_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel_x, int &node_sel_y);
void handle_node_mass_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel);
void handle_scan_path_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel);
void handle_node_damping_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel);
void handle_node_constraint_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel);
void handle_scanner_node_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel, int *params, int max);
void handle_info_menu(Display *dpy, Window w, GC gc, int &menu_id);

void draw_adsr_menu(Display *dpy, Window w, GC gc, int node_sel_x);
void draw_eq_pos_menu(Display *dpy, Window w, GC gc, int node_sel_x, int node_sel_y);
void draw_scanner_2d_menu(Display *dpy, Window w, GC gc);
void draw_connectivity_menu(Display *dpy, Window w, GC gc, int node_sel_x, int node_sel_y);
void draw_scan_path_menu(Display *dpy, Window w, GC gc, int node_sel);
void draw_node_mass_menu(Display *dpy, Window w, GC gc, int node_sel);
void draw_node_damping_menu(Display *dpy, Window w, GC gc, int node_sel);
void draw_node_constraint_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel);
void draw_scanner_node_menu(Display *dpy, Window w, GC gc, int node_sel, int *params);
void draw_info_menu(Display *dpy, Window w, GC gc);

void print_keybindings();


void draw_float(Display *dpy, Window w, GC gc, int x, int y, float num, int decimals);
void draw_node_labels(Vector3f *node_pos, int num_nodes);
void draw_text_boxes(int * params);
void draw_select_box(int node_sel);

int rainbow(int c);
Matrix3f get_rotation(float x, float y, float z);
int get_num();
float get_adsr_color_scalar();

ProjectedStereoPoints project_stereo_point(Vector3f point_v, Vector3f view_v);
StereoPixels projection_to_stereo_pixels(ProjectedStereoPoints pp);
void draw_stereo_line(Vector3f start_point, Vector3f end_point);
void draw_stereo_point(Vector3f point);
void draw_stereo_sphere(Sphere sphere);
void draw_stereo_lines(Vector3f *start_point, Vector3f *end_point, int len);
void draw_stereo_points(Vector3f *point, int len);


ProjectedPoint project_mono_point(Vector3f point_v, Vector3f view_v);
MonoPixel projection_to_mono_pixel(ProjectedPoint pp);
//void draw_mono_line(Vector3f start_point, Vector3f end_point);
//void draw_mono_point(Vector3f point);
//void draw_mono_sphere(Sphere sphere);
void draw_mono_lines(Vector3f *start_point, Vector3f *end_point, int len);
void draw_mono_points(Vector3f *point, int len);


void runge_kutta(float *X, float *Xt1, void (ode)(float*,float*), float ts, int len);
void lorenz_ode(float *X, float *Xd);
