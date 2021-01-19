#include "super_scanner.h"
#include "scanner_window.h"
#include "audio_engine.h"

int main(int argc, char *argv[]){
  if(argc < 2 ){
    printf("usage ./scan <midi keyboard> <midi controller>\n");
    exit(-1);
  }
  
  SuperScanner scanner(16);

  init_audio(&scanner); //this has to go first apparently.
  init_midi(argc, argv);
  
  init_window(&scanner);
  scanner.start();
  while(is_window_open()){
    usleep(1000); //to prevent is_window_open from burning cycles.
  }
  //scanner.stop();
  del_window();
  
  del_audio();
  del_midi();
  return 0;
}
