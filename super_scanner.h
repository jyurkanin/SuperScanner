#pragma once

int is_window_open();


class SuperScanner{
public:
  
  SuperScanner();
  ~SuperScanner();

  void strike();
  float tick(int note, float volume);
  
  int init_window();
  int del_window();
  
  int start();
  int stop();
  
private:
  float freqs[12] = {27.5, 29.135, 30.868, 32.703, 34.648, 36.708, 38.891, 41.203, 43.654, 46.249, 48.999, 51.913}; // frequencies of the lowest octave
};
