#pragma once

#include <iostream>
#include <fstream>

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
  
  void release();
  void strike();
  float tick(int note, float volume);
  void compute_scan_table();
  float get_adsr_gain();
  void sort_adsr_table();
  void startNote();
  
  void setHammer(int num);
  
  void sim();
  
  void update_params();
  
  int init_window();
  int del_window();
  
  int start();
  int stop();

  const int oversample_factor = 2;
  const int num_nodes;
  Vector3f *node_pos;
  float *stiffness_matrix; //Oh boyo. Upper triangular. to avoid repetition.
  volatile int sim_mutex;
  volatile int has_scan_update; //volatile just in case.
  int scan_len; //size of the table getting scanned
  int *scan_path;
  int *node_damping;
  int *node_mass;
  
  
//private:
  static void* simulate_wrapper(void*);
  void simulate();
  void ODE(Vector3f *X, Vector3f *Xd);
  void solveRungeKutta(Vector3f *X, Vector3f *X1);
  void solveForwardEuler(Vector3f *X, Vector3f *X1);
  
  float freqs[12] = {27.5, 29.135, 30.868, 32.703, 34.648, 36.708, 38.891, 41.203, 43.654, 46.249, 48.999, 51.913}; // frequencies of the lowest octave
  Controller controller;
  
  pthread_t scan_thread;
  
  std::ofstream log_file;
  
  volatile int release_flag;
  int hammer_num;
  float timestep;
  int sample_rate;
  float m_volume;
  float release_stiffness;
  float release_damping;
  float mass_bias;
  float damping_bias;
  float stiffness_bias;
  float test_knob;
  
  uint32_t k_;
  float *scan_table;
  float *old_scan_table;
  float *hammer_table;
  float *displacement_matrix; //Also upper triangular. Represents the equillibrium length of the spring connecting two nodes.
  int *constrained_nodes;
  Vector3f *node_eq_pos;
  float restoring_stiffness;
  int scan_method;
  
  int pad_mode;

  const unsigned adsr_table_len = 12;
  Vector2f *adsr_table;
  unsigned adsr_state;
  float adsr_gain;
  float adsr_gain_naive;
  float gain_before_release;
  unsigned release_k;
  
  Vector3f origin;
  Vector3f scanner_rot;
  
  Vector3f *node_vel;
  Vector3f *node_acc;
  
  Vector3f *rk4_temp;
  Vector3f *k1;
  Vector3f *k2;
  Vector3f *k3;
  Vector3f *k4;

};
