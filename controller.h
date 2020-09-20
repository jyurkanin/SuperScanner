#pragma once

#include <pthread.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#define PEDAL 176

class Controller{
 public:
  Controller();
  void activate();
  unsigned char get_slider(int index);
  unsigned char get_knob(int index);
  unsigned char get_button(int index);
  unsigned char get_big_slider();
  unsigned char get_big_knob();
  unsigned char was_start_pressed();
  unsigned char was_stop_pressed();
  unsigned char was_record_pressed();
  unsigned char was_loop_back_pressed();
  unsigned char was_fastforward_pressed();
  unsigned char was_rewind_pressed();
  static int has_new_data();
  static int init_controller(int argc, char *argv[]);
  static int exit_controller();
  int slider[9];
  int knob[9];
  int button[9];
 private:
  static void *read_controller(void *nothing);
  unsigned char start;
  unsigned char stop;
  unsigned char record;
  unsigned char loop_back;
  unsigned char fastforward;
  unsigned char rewind;
  unsigned char big_slider;
  unsigned char big_knob;
  static int has_new;
  static int controllerFD;
  static pthread_t c_thread;
  static Controller *active_controller;
};

