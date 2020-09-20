#include "super_scanner.h"
#include "scanner_window.h"

int is_program_running_ = 0;

void start_program(){
    is_program_running_ = 1;
}

void stop_program(){
    is_program_running_ = 0;
}

int is_program_running(){
    return is_program_running_;
}

SuperScanner *get_scanner(){
    return super_scanner;
}


int main(){
    init_window();
    while(is_window_alive()){
        
    }
    printf("bromine\n");
    del_window();
    printf("brotato chip\n");
    
}
