#include "super_scanner.h"
#include "scanner_window.h"

int is_window_open_ = 0;
int is_window_open(){
  return is_window_open_;
}

SuperScanner::SuperScanner(){
  is_window_open_ = 1;
}

SuperScanner::~SuperScanner(){
  is_window_open_ = 0;
}

float SuperScanner::tick(int note, float volume){
  float freq = freqs[(note-21) % 12] * (1 << (1+(int)(note-21)/12));
  return freq;
}

