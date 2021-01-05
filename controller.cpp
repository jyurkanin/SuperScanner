#include "super_scanner.h"
#include "controller.h"
#include <string.h>
#include <fcntl.h>
#include <queue>
#include <sys/ioctl.h>

int Controller::controllerFD;
pthread_t Controller::c_thread;
Controller *Controller::active_controller;

Controller::Controller(){
  memset(slider, 0, 9*sizeof(int));
  memset(knob, 0, 9*sizeof(int));
  memset(button, 0, 9*sizeof(int));
  start = 0;
  stop = 0;
  record = 0;
  loop_back = 0;
  fastforward = 0;
  rewind = 0;
  big_slider = 0;
  big_knob = 0;
}

unsigned char Controller::get_slider(int index){
  return slider[index];
}

unsigned char Controller::get_knob(int index){
  return knob[index];
}

unsigned char Controller::get_big_slider(){
  return big_slider;
}

unsigned char Controller::get_big_knob(){
  return big_knob;
}

unsigned char Controller::get_button(int index){
  int length = (sizeof(button)/sizeof(*(button)));
  if(index >= length){
    return 0;
  }
  return button[index];
}

unsigned char Controller::was_start_pressed(){
  if(start){
    start = 0;
    return 127;
  }
  else{
    return 0;
  }
}

unsigned char Controller::was_stop_pressed(){
  if(stop){
    stop = 0;
    return 127;
  }
  else{
    return 0;
  }
}

unsigned char Controller::was_record_pressed(){
  if(record){
    record = 0;
    return 127;
  }
  else{
    return 0;
  }
}

unsigned char Controller::was_loop_back_pressed(){
  if(loop_back){
    loop_back = 0;
    return 127;
  }
  else{
    return 0;
  }
}

unsigned char Controller::was_fastforward_pressed(){
  if(fastforward){
    fastforward = 0;
    return 127;
  }
  else{
    return 0;
  }
}

unsigned char Controller::was_rewind_pressed(){
  if(rewind){
    rewind = 0;
    return 127;
  }
  else{
    return 0;
  }
}

int Controller::has_new;

int Controller::has_new_data(){
  if(has_new){
    has_new = 0;
    return 1;
  }
  return 0;
}

void *Controller::read_controller(void *nothing){
    unsigned char packet[4];
    has_new = 1;
    std::queue<unsigned char> incoming;
    unsigned char temp;
    while(is_window_open()){
        if(read(controllerFD, &temp, sizeof(temp)) <= 0){
            usleep(1000);
            continue;
        }
        else{
            incoming.push(temp);
        }
    
        if(incoming.size() >= 3){
            packet[0] = incoming.front(); incoming.pop();
            packet[1] = incoming.front(); incoming.pop();
            packet[2] = incoming.front(); incoming.pop();
        }
        else continue;
    
        has_new = 1;
    
        if(packet[0] == PEDAL){
            if(packet[1] >= 3 && packet[1] <= 11){
                active_controller->slider[packet[1] - 3] = packet[2];
            }
            else if(packet[1] >= 14 && packet[1] <= 22){
                active_controller->knob[packet[1] - 14] = packet[2];
            }
            else if(packet[1] == 9){
                active_controller->big_slider = packet[2];
            }
            else if(packet[1] == 10){
                if(packet[2]) active_controller->big_knob = packet[2];
            }
            else if(packet[1] >= 23 && packet[1] <= 31 && packet[2]){
                active_controller->button[packet[1] - 23] ^= 1; //this toggles the state of the button
            }
            else if(packet[2]){ //only assign it if it was pressed. The application that reads it will have to reset the button.
                switch(packet[1]){
                case 49:
                    active_controller->loop_back = packet[2];
                    break;
                case 47:
                    active_controller->rewind = packet[2];
                    break;
                case 48:
                    active_controller->fastforward = packet[2];
                    break;
                case 46:
                    active_controller->stop = packet[2];
                    break;
                case 45:
                    active_controller->start = packet[2];
                    break;
                case 44:
                    active_controller->record = packet[2];
                    break;
                }
            }
        }
    }
    printf("Controller Thread is DEADBEEF\n");
    return NULL;
}

//this sets this controller to the one being written to by the
// controller reading thread
void Controller::activate(){
  active_controller = this;
}

int Controller::init_controller(int argc, char *argv[]){
  char *midi_controller;
  if(argc == 3)
   midi_controller = argv[2];
  else{
    midi_controller = argv[1];
  }
  int flags;
  controllerFD = open(midi_controller, O_RDONLY);

  flags = fcntl(controllerFD, F_GETFL, 0);
  fcntl(controllerFD, F_SETFL, flags | O_NONBLOCK);
  
  if(controllerFD < 0){
    printf("Couldn't open device %s\n", midi_controller);
    exit(-1);
  }
  
  pthread_create(&c_thread, NULL, &read_controller, NULL);
  return EXIT_SUCCESS;
}

int Controller::exit_controller(){
  pthread_join(c_thread, NULL);
  close(controllerFD);
  return EXIT_SUCCESS;
}
