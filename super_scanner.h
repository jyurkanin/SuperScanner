#pragma once

#include "controller.h"
#ifdef Success //this is stupid
  #undef Success
#endif
#include <eigen3/Eigen/Dense>



int is_window_open();


class SuperScanner{
public:
  
  SuperScanner();
  ~SuperScanner();

  void strike();
  float tick(int note, float volume);

  void setHammer(int num);

  void sim();
  void runge_kutta(float *X, float *Xt1, void (ode)(float*,float*), float ts, int len);
  
  int init_window();
  int del_window();
  
  int start();
  int stop();

  
private:
  void* simulate(void*);
  
  float freqs[12] = {27.5, 29.135, 30.868, 32.703, 34.648, 36.708, 38.891, 41.203, 43.654, 46.249, 48.999, 51.913}; // frequencies of the lowest octave
  Controller controller;

  pthread_t scan_thread;
  
  int scan_len; //size of the table getting scanned
  int hammer_num;
  int num_nodes;
  
  float *scan_path;
  float *hammer_table;
  float *stiffness_matrix; //Oh boyo. Upper triangular. to avoid repetition.
  float *displacement_matrix;
  int *constrained_nodes;
  float *restoring_stiffness;
  float *node_damping;
  float *node_mass;

  Vector3f *node_pos;
  Vector3f *node_vel;
  Vector3f *node_acc;
};
