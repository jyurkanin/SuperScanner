#pragma once

#include "controller.h"
#ifdef Success //this is stupid
  #undef Success
#endif
#include <eigen3/Eigen/Dense>


using namespace Eigen;

int is_window_open();


class SuperScanner{
public:
  
  SuperScanner(int s);
  ~SuperScanner();
  
  void strike();
  float tick(int note, float volume);
  
  void setHammer(int num);
  
  void sim();
  
  void update_params();
  
  int init_window();
  int del_window();
  
  int start();
  int stop();


  const int num_nodes;
  Vector3f *node_pos;
  float *stiffness_matrix; //Oh boyo. Upper triangular. to avoid repetition.
  int sim_mutex;
private:
  static void* simulate_wrapper(void*);
  void simulate();
  void ODE(Vector3f *acc);
  
  float freqs[12] = {27.5, 29.135, 30.868, 32.703, 34.648, 36.708, 38.891, 41.203, 43.654, 46.249, 48.999, 51.913}; // frequencies of the lowest octave
  Controller controller;
  
  pthread_t scan_thread;
  
  int scan_len; //size of the table getting scanned
  int hammer_num;
  float timestep;
  float sample_rate;
  float m_volume;
  
  uint32_t k_;
  
  int *scan_path;
  float *hammer_table;
  float *displacement_matrix; //Also upper triangular. Represents the equillibrium length of the spring connecting two nodes.
  int *constrained_nodes;
  Vector3f *node_eq_pos;
  float *restoring_stiffness;
  float *node_damping;
  float *node_mass;


  Vector3f *node_vel;
  Vector3f *node_acc;
};
