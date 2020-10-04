#include "super_scanner.h"
#include "scanner_window.h"
#include "audio_engine.h"

int main(int argc, char *argv[]){
  SuperScanner scanner;
  
  init_midi(argc, argv);
  init_audio(&scanner);
  
  init_window();
  scanner.start();
  while(is_window_alive()){
    usleep(1000); //to prevent is_window_open from burning cycles.
  }
  scanner.stop();
  del_window();
  
  del_audio();
  del_midi();
  return 0;
}
