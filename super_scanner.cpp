#include "super_scanner.h"
#include "scanner_window.h"
#include "wavfile.h"

int is_window_open_ = 0;
int is_window_open(){
  return is_window_open_;
}

SuperScanner::SuperScanner(){
  is_window_open_ = 1;
  controller = Controller();
  controller.activate();
  
  scan_len = 32;
  hammer_num = 0;
  hammer_table = new float[scan_len];
  scan_path = new float[scan_len];

  //create a linear network
  num_nodes = 128;
  stiffness_matrix = new float[num_nodes*num_nodes];
  displacement_matrix = new float[num_nodes*num_nodes];
  memset(stiffness_matrix, 0, sizeof(float)*num_nodes*num_nodes);
  memset(displacement_matrix, 0, sizeof(float)*num_nodes*num_nodes);
  for(int i = 0; i < num_nodes-1; i++){
    stiffness_matrix[(i*num_nodes) + (i+1)] = 1.0;
    displacement_matrix[(i*num_nodes) + (i+1)] = 1.0; //natural displacement of the spring
  }
  
  constrained_nodes = new int[num_nodes];
  memset(constrained_nodes, 0, sizeof(int)*num_nodes);
  constrained_nodes[0] = 1;
  constrained_nodes[num_nodes-1] = 1;
  
  node_damping = new float[num_nodes];
  memset(node_damping, 0.01, sizeof(int)*num_nodes);
  
  restoring_stiffness = new float[num_nodes];
  memset(restoring_stiffness, 0.01, sizeof(float)*num_nodes);

  node_mass = new float[num_nodes];
  memset(node_mass, 1.0, sizeof(float)*num_nodes);
  
  node_pos = new Vector3f[3*num_nodes];
  node_vel = new Vector3f[3*num_nodes];
  node_acc = new Vector3f[3*num_nodes];
  
  
  setHammer(0);
}

SuperScanner::~SuperScanner(){
  is_window_open_ = 0;
  delete[] scan_path;
  delete[] hammer_table;
  delete[] stiffness_matrix;
  delete[] displacement_matrix;
  delete[] constrained_nodes;
  delete[] restoring_stiffness;
  delete[] node_damping;
  delete[] node_mass;
  delete[] node_pos;
  delete[] node_vel;
  delete[] node_acc;
}

float SuperScanner::tick(int note, float volume){
  float freq = freqs[(note-21) % 12] * (1 << (1+(int)(note-21)/12));
  return freq;
}

void* SuperScanner::simulate(void*){
  float timer = 0;
  Vector3f F_restore;
  Vector3f F_damping;
  Vector3f F_spring;

  Vector3f d_pos;
  
  float diff;
  float d_pos_norm;
  
  //Let N be num_nodes
  while(is_window_open_){
    for(int i = 0; i < num_nodes; i++){
      F_restore = -node_pos[i]*(.5*restoring_stiffness[i]*node_pos[i].norm()); //N restoring forces
      F_damping = -node_vel[i]*node_damping[i]; //N damping forces.

      F_spring[0] = 0;
      F_spring[1] = 0;
      F_spring[2] = 0;
      
      for(int j = 0; j < num_nodes; j++){
	d_pos = node_pos[j] - node_pos[i];
	d_pos_norm = d_pos.norm();
	diff = d_pos_norm - displacement_matrix[(i*num_nodes) + j];
	F_spring += d_pos*(.5*stiffness_matrix[(i*num_nodes) + j]*diff*diff/d_pos_norm);
      }
      
      node_acc[i] = node_mass[i].cwiseQuotient(F_spring);
    }
  }
}

void SuperScanner::strike(){
  
}

void SuperScanner::start(){
  is_window_open_ = 1;
  pthread_create(&scan_thread, NULL, &simulate, NULL);
  
  return 0;
}

int SuperScanner::stop(){
  is_window_open_ = 0;
  pthread_join(scan_thread, 0);
  return 0;
}

void SuperScanner::setHammer(int num){
    char fn[100];
    memset(fn, 0, sizeof(fn));
    sprintf(fn, "AKWF/AKWF_%04d.wav", num);

    printf("hammertime\n");
    hammer_num = num;
    if(hammer_num == 0){ //special case.
        for(int i = 0; i < scan_len; i++){
            hammer_table[i] = 5*sinf(M_PI*(i+1)/(scan_len+2));
        }
    }
    else if(hammer_num == 101){ //another special case I felt was worth including.
        for(int i = 0; i < scan_len; i++){
            hammer_table[i] = 5*fabs(((scan_len+2) / 2.0) - (i+1)) / ((scan_len+2)/2);
        }
    }
    else{ //load the file from AKWF.
        WavFile wavfile(fn); //opens the wav file associated with the waveform given in the string.
        //now you need to linearly interpolate to make the wavfile fit into the hammer table.
        if(scan_len == wavfile.data_len){
            for(int i = 0; i < scan_len; i++){
                hammer_table[i] = wavfile.data[i]*5;
            }
        }
        else if(scan_len < wavfile.data_len){
            for(int i = 0; i < scan_len; i++){
                hammer_table[i] = 5*wavfile.data[(int)(i*wavfile.data_len/(float)scan_len)];
            }
        }
        else if(scan_len > wavfile.data_len){ //need to linearly interpolate.
            float index;
            int l_index; //lower
            int u_index; //upper
            for(int i = 0; i < scan_len; i++){
                index = (i*wavfile.data_len/(float)scan_len);
                l_index = (int) index;
                u_index = l_index+1;
                if(u_index < wavfile.data_len)
                    hammer_table[i] = 5*((index-l_index)*wavfile.data[u_index] + (u_index-index)*wavfile.data[l_index]); //linear interpolation
                else //edge case
                    hammer_table[i] = 5*((index-l_index)*wavfile.data[wavfile.data_len] + (u_index-index)*wavfile.data[l_index]); 
            }            
        }
    }
}


void SuperScanner::runge_kutta(float *X, float *Xt1, void (ode)(float*,float*), float ts, int len){
    float temp[3];
    float k1[3]; ode(X, k1); //Calculate derivative.
    
    for(int i = 0; i < len; i++) temp[i] = X[i]+.5*ts*k1[i];
    float k2[3]; ode(temp, k2);
    
    for(int i = 0; i < len; i++) temp[i] = X[i]+.5*ts*k2[i];
    float k3[3]; ode(temp, k3);
    
    for(int i = 0; i < len; i++) temp[i] = X[i]+ts*k3[i];
    float k4[3]; ode(temp, k4);
    
    for(int i = 0; i < len; i++){
        Xt1[i] = X[i] + (ts/6)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }
}
