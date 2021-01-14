#include "super_scanner.h"
#include "scanner_window.h"
#include "audio_engine.h"
#include <stdio.h>

void compare_scanners(SuperScanner &scan1, SuperScanner &scan2){
    if(scan1.scan_len != scan2.scan_len){
        printf("BAD SHIT BRO. Scan Len\n"); 
    }
    if(scan1.num_nodes != scan2.num_nodes){
        printf("BAD SHIT BRO. Num Nodes\n"); 
    }
    if(scan1.hammer_num != scan2.hammer_num){
        printf("BAD SHIT BRO. Hammer Num %d %d\n", scan1.hammer_num, scan2.hammer_num); 
    }
    if(scan1.timestep != scan2.timestep){
        printf("BAD SHIT BRO. Timestep\n"); 
    }
    
    if(scan1.m_volume != scan2.m_volume){
        printf("BAD SHIT BRO. m_volume\n"); 
    }
    if(scan1.release_stiffness != scan2.release_stiffness){
        printf("BAD SHIT BRO. release_stiffness\n"); 
    }
    if(scan1.release_damping != scan2.release_damping){
        printf("BAD SHIT BRO. release_damping\n"); 
    }
    if(scan1.mass_bias != scan2.mass_bias){
        printf("BAD SHIT BRO. mass_bias\n"); 
    }
    if(scan1.damping_bias != scan2.damping_bias){
        printf("BAD SHIT BRO. damping_bias\n"); 
    }
    if(scan1.stiffness_bias != scan2.stiffness_bias){
        printf("BAD SHIT BRO. stiffness_bias\n"); 
    }
    if(scan1.restoring_stiffness != scan2.restoring_stiffness){
        printf("BAD SHIT BRO. restoring_stiffness\n"); 
    }
    if(scan1.scan_method != scan2.scan_method){
        printf("BAD SHIT BRO. scan_method\n"); 
    }
    if(scan1.pad_mode != scan2.pad_mode){
        printf("BAD SHIT BRO. pad_mode\n"); 
    }
    if(scan1.origin != scan2.origin){
        printf("BAD SHIT BRO. origin\n"); 
    }
    if(scan1.scanner_rot != scan2.scanner_rot){
        printf("BAD SHIT BRO. scanner_rot\n"); 
    }

    for(int i = 0; i < 9; i++){
        if(scan1.controller.get_slider(i) != scan2.controller.get_slider(i)){
            printf("BAD SHIT BRO. Sliders dont match %d %d\n", scan1.controller.get_slider(i), scan2.controller.get_slider(i)); 
        }
        if(scan1.controller.get_knob(i) != scan2.controller.get_knob(i)){
            printf("BAD SHIT BRO. Knob dont match %d %d\n", scan1.controller.get_knob(i), scan2.controller.get_knob(i)); 
        }
        if(scan1.controller.get_button(i) != scan2.controller.get_button(i)){
            printf("BAD SHIT BRO. Buttons dont match %d %d\n", scan1.controller.get_button(i), scan2.controller.get_button(i)); 
        }


        
        if(scan1.reverb.controller.get_slider(i) != scan2.reverb.controller.get_slider(i)){
            printf("BAD SHIT BRO. Reverb Sliders dont match\n"); 
        }
        if(scan1.reverb.controller.get_knob(i) != scan2.reverb.controller.get_knob(i)){
            printf("BAD SHIT BRO. Reverb Knob dont match\n"); 
        }
        if(scan1.reverb.controller.get_button(i) != scan2.reverb.controller.get_button(i)){
            printf("BAD SHIT BRO. Reverb Buttons dont match\n"); 
        }
    }
    
    
    
    
    
    for(int i = 0; i < scan1.scan_len; i++){
        if(scan1.scan_path[i] != scan2.scan_path[i]){
            printf("BAD SHIT BRO. scan path\n"); 
        }
    }
    
    
    
    for(int i = 0; i < scan1.num_nodes; i++){
        if(scan1.node_damping[i] != scan2.node_damping[i]){
            printf("BAD SHIT BRO. Damping\n"); 
        }
        if(scan1.node_mass[i] != scan2.node_mass[i]){
            printf("BAD SHIT BRO. Mass\n"); 
        }
        if(scan1.constrained_nodes[i] != scan2.constrained_nodes[i]){
            printf("BAD SHIT BRO. Constrained_nodes\n"); 
        }
        if(!(scan1.node_eq_pos[i] - scan2.node_eq_pos[i]).isMuchSmallerThan(.01)){
            printf("BAD SHIT BRO. eq_pos\n"); 
        }
    }

    for(int i = 0; i < (scan1.num_nodes*scan1.num_nodes); i++){
        if(fabs(scan1.stiffness_matrix[i] - scan2.stiffness_matrix[i]) > 1e-3){
            printf("BAD SHIT BRO. Stiffness Matrix %d %f %f\n", i, scan1.stiffness_matrix[i], scan2.stiffness_matrix[i]); 
        }
        if(fabs(scan1.displacement_matrix[i] - scan2.displacement_matrix[i]) > 1e-3){
            printf("BAD SHIT BRO. Displacement Matrix\n"); 
        }
    }

    for(unsigned i = 0; i < scan1.adsr_table_len; i++){
        if(scan1.adsr_table[i] != scan2.adsr_table[i]){
            printf("BAD SHIT BRO. adsr table does not match.");
        }
    }
}

void randomize(SuperScanner &scanner){
    scanner.scanner_rot[0] = .1;
    scanner.scanner_rot[1] = .2;
    scanner.scanner_rot[2] = .3;

    scanner.origin[0] = .4;
    scanner.origin[1] = .5;
    scanner.origin[2] = .6;

    scanner.scan_len = 16;
    scanner.num_nodes = 35;

    for(int i = 0; i < 9; i++){
        scanner.controller.slider[i] = i*2;
        scanner.controller.knob[i] = i*3;
        scanner.controller.button[i] = i*5;
    }

    for(int i = 0; i < 9; i++){
        scanner.reverb.controller.slider[i] = i*6;
        scanner.reverb.controller.knob[i] = i*7;
        scanner.reverb.controller.button[i] = i*8;
    }
    
    for(unsigned i = 0; i < scanner.adsr_table_len; i++){
        scanner.adsr_table[i][0] = .34*i;
        scanner.adsr_table[i][1] = .9*i;
    }
    
}

void test_save_load(){
    SuperScanner scanner(64);
    SuperScanner scanner2(64);

    randomize(scanner);
    scanner.save("test.scan");
    scanner.controller.set_new_data();
    scanner.update_params();
    scanner.reverb.controller.set_new_data();
    scanner.reverb.update_params();
    
    printf("DERP \n");
    
    scanner2.load("test.scan");
    compare_scanners(scanner, scanner2);
    
}

int main(int argc, char *argv[]){
    test_save_load();
}
