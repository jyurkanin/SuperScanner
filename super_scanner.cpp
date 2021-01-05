#include "super_scanner.h"
#include "scanner_window.h"
#include "wavfile.h"
#include "audio_engine.h"
#include <sys/time.h>
#include <unistd.h>
#include <algorithm>
#include <stdlib.h>


void break_on_me(){}
  
int is_window_open_ = 0;
int is_window_open(){
  return is_window_open_;
}

SuperScanner::SuperScanner(int s) : num_nodes(s){
  is_window_open_ = 1;
  sim_mutex = 0;
  has_scan_update = 0;
  release_flag = 1;
  
  controller = Controller();
  controller.activate();

  release_damping = 1;
  release_stiffness = 1;

  mass_bias = 0;
  damping_bias = 0;
  stiffness_bias = 0;
  scan_method = 0;
  
  origin = {-10,4,0};
  scanner_rot = {0,0,0};
  
  scan_len = num_nodes; //create a linear network
  hammer_num = 0;
  hammer_table = new float[scan_len];
  scan_path = new int[scan_len];
  scan_table = new float[scan_len];
  old_scan_table = new float[scan_len];
  
  memset(scan_table, 0, sizeof(float)*scan_len);
  memset(old_scan_table, 0, sizeof(float)*scan_len);
  
  for(int i = 0; i < scan_len; i++){
      scan_path[i] = i;
  }
  
  
  
  //create a linear network
  stiffness_matrix = new float[num_nodes*num_nodes];
  displacement_matrix = new float[num_nodes*num_nodes];
  memset(stiffness_matrix, 0, sizeof(float)*num_nodes*num_nodes);
  memset(displacement_matrix, 0, sizeof(float)*num_nodes*num_nodes);
  
  
  for(int i = 0; i < num_nodes-1; i++){
      //float temp_stiffness = (2.0f*rand())/RAND_MAX;
      float temp_stiffness = 1+(i/(float)num_nodes);
      stiffness_matrix[(i*num_nodes) + (i+1)] = temp_stiffness;
      displacement_matrix[(i*num_nodes) + (i+1)] = 1.0; //natural displacement of the spring
      
      stiffness_matrix[((i+1)*num_nodes) + i] = temp_stiffness;
      displacement_matrix[((i+1)*num_nodes) + i] = 1.0;
  }
  

  /*
  float temp;
  for(int i = 0; i < num_nodes; i++){
    for(int j = i+1; j < num_nodes; j++){
        temp = (.125f*rand())/RAND_MAX;
        if(temp < .123) continue;
        stiffness_matrix[(i*num_nodes) + j] = temp;
        displacement_matrix[(i*num_nodes) + j] = 1.0;
        stiffness_matrix[(j*num_nodes) + i] = temp;
        displacement_matrix[(j*num_nodes) + i] = 1.0;
    } 
  }
  */

  /*
  int side_len = 8;
  int x[] = {0,0,1,-1};
  int y[] = {1,-1,0,0};

  int index = 0;
  int neighbor = 0;
  for(int i = 0; i < side_len; i++){
        for(int j = 0; j < side_len; j++){
            index = (i*side_len) + j;
            for(int k = 0; k < 4; k++){
                if((i + x[k]) < 0) continue;
                if((i + x[k]) > (side_len-1)) continue;
                if((j + y[k]) < 0) continue;
                if((j + y[k]) > (side_len-1)) continue;
                
                neighbor = ((i+x[k])*side_len) + j + y[k];

                float temp_stiffness = (.5f*rand())/RAND_MAX;
                stiffness_matrix[(index*num_nodes) + neighbor] = temp_stiffness;
                displacement_matrix[(index*num_nodes) + neighbor] = 1.0; //natural displacement of the spring
                stiffness_matrix[(neighbor*num_nodes) + index] = temp_stiffness;
                displacement_matrix[(neighbor*num_nodes) + index] = 1.0; //natural displacement of the spring
            }
        }
  }
  */
  
  constrained_nodes = new int[num_nodes];
  memset(constrained_nodes, 0, num_nodes*sizeof(int));
  
  node_damping = new int[num_nodes];
  restoring_stiffness = 2;
  for(int i = 0; i < num_nodes; i++){
      node_damping[i] = 1; //this is divided by 100 so chill.
  }
  
  
  adsr_table = new Vector2f[adsr_table_len];
  adsr_table[adsr_table_len-1][0] = 1.00;
  adsr_table[adsr_table_len-1][1] = 0;
  adsr_state = 0;
  adsr_gain = 1;
  
  node_pos = new Vector3f[num_nodes];
  node_vel = new Vector3f[num_nodes];
  node_acc = new Vector3f[num_nodes];
  node_eq_pos = new Vector3f[num_nodes];
  
  
  for(int i = 0; i < num_nodes; i++){
      node_eq_pos[i] = Vector3f(0, i*4, 0);
      node_pos[i] = Vector3f(0, i*4, 0);
      node_vel[i] = Vector3f(0, 0, 0);
  }

  constrained_nodes[0] = 1;
  constrained_nodes[63] = 1;
  
  /*
  for(int i = 0; i < side_len; i++){
      for(int j = 0; j < side_len; j++){
          node_eq_pos[(i*side_len)+j] = Vector3f(i*4, j*4, 0);
          node_pos[(i*side_len)+j] = Vector3f(i*4, j*4, 0);
          node_vel[(i*side_len)+j] = Vector3f(0, 0, 0);
      }
  }
  */
  
  constrained_nodes[0] = 1;
  //constrained_nodes[7] = 1;
  //constrained_nodes[(7*side_len)] = 1;
  constrained_nodes[num_nodes-1] = 1;
  
  node_mass = new int[num_nodes];
  for(int i = 0; i < num_nodes; i++){
      node_mass[i] = 1;
  }
  
  timestep = .001; //randomly chosen. .01 was unstable.

  sample_rate = 44100;
  
  //init_params associated with controller

  pad_mode = 1;
  k_ = 0;
  setHammer(55); //Nice. I like this function. Really gets the job done.
}

SuperScanner::~SuperScanner(){
  is_window_open_ = 0;
  delete[] adsr_table;
  delete[] scan_path;
  delete[] hammer_table;
  delete[] stiffness_matrix;
  delete[] displacement_matrix;
  delete[] constrained_nodes;
  delete[] node_damping;
  delete[] node_mass;
  delete[] node_pos;
  delete[] node_vel;
  delete[] node_acc;
}

/* cases:
 * note is > 0 and !release : portamento to note
 * note is > 0 and release : play note, no portamento
 * note is -1 and !release : hold same note.
 * note is -1 and release : Idk how this could hapen.
 */
float SuperScanner::tick(int note, float volume){
  static int was_released = 1;
  static float p_freq = 0;
  static uint32_t k_update = 0;

  if(has_scan_update){
      k_update = k_;
      has_scan_update = 0;
  }
  
  float freq = 0;
  if(note != -1){
      freq = (freqs[(note-21) % 12] * (1 << (1+(int)(note-21)/12)))/2;
  }
  
  if((note != -1) && was_released){p_freq = freq;}
  else if((note != -1) && !was_released){p_freq += (freq - p_freq)/1000;}

  /*
  if((k_ % 1000) == 0){
      log_file << adsr_gain << ',' << k_ <<'\n';
  }
  */
  
  compute_scan_table();
  
  float idx = fmod(p_freq*k_*scan_len/((float)sample_rate), scan_len);
  int lower = floorf(idx);
  float diff = idx - lower;
  float sample = scan_table[lower]*(1-diff) + scan_table[lower+1]*(diff); //interpolate along scan table axis
  float old_sample = old_scan_table[lower]*(1-diff) + old_scan_table[lower+1]*(diff);

  diff = std::min(1.0f, (k_ - k_update)/(sample_rate*timestep));
  float i_sample = sample*diff + old_sample*(1-diff); //interpolate along time axis.
  
  
  for(int i = 0; i < scan_len; i++){
      old_scan_table[i] = scan_table[i];
  }
  
  
  k_++;
  was_released = release_flag;
  return i_sample * .2 * m_volume * volume / 127.0;
}

float SuperScanner::get_adsr_gain(){
    unsigned final_sustain_state = floorf(adsr_table_len*.75);
    unsigned k_norm; 
    float tc = 2*sample_rate; //time constant. to samples. 2 means the total envelope is 2 seconds long.
    float y1, y2, slope, dist;
    float gain;
    unsigned k_temp;
    
    if(release_flag){
        k_temp = k_ - release_k + tc*adsr_table[final_sustain_state][0];
        if((k_temp >= (tc*adsr_table[adsr_state+1][0])) && (adsr_state != (adsr_table_len-1))){
            printf("adsr_state release%d\n", adsr_state);
            adsr_state++;
        }
        if(adsr_state != (adsr_table_len-1)){
            k_norm = k_temp - tc*adsr_table[adsr_state][0];
            y1 = adsr_table[adsr_state][1];
            y2 = adsr_table[adsr_state+1][1];
            dist = (adsr_table[adsr_state+1][0] - adsr_table[adsr_state][0])*tc;
            slope = (y2-y1)/dist;
            gain = (k_norm*slope) + y1;
        }
        else{
            gain = adsr_table[adsr_table_len-1][1];
        }
    }
    else{
        if((k_ >= (tc*adsr_table[adsr_state+1][0])) && (adsr_state != final_sustain_state)){
            printf("adsr_state %d\n", adsr_state);
            adsr_state++;
        }
        if(adsr_state != final_sustain_state){
            k_norm = k_ - (tc*adsr_table[adsr_state][0]); //k_norm counts samples from the current state.
            y1 = adsr_table[adsr_state][1];
            y2 = adsr_table[adsr_state+1][1];
            dist = (adsr_table[adsr_state+1][0] - adsr_table[adsr_state][0])*tc;
            slope = (y2-y1)/dist;
            gain = (k_norm*slope) + y1;
        }
        else{
            gain = adsr_table[final_sustain_state][1];
        }
    }
    adsr_gain = gain;
    return gain;
}

void SuperScanner::compute_scan_table(){
    float sum;
    Vector3f temp;
    int select = scan_method/10;
    for(int i = 0; i < scan_len; i++){
        switch(select){
        case 0:
            scan_table[i] = node_pos[scan_path[i]][2]; //this is one possible method.   
            break;
        case 1:
            scan_table[i] = ((node_pos[scan_path[i]] - node_eq_pos[scan_path[i]]).norm()); //this only gives positive numbers. Possibly lame
            break;
        case 2:
            temp = (node_pos[scan_path[i]] - node_eq_pos[scan_path[i]]);
            sum = temp[0] + temp[1] + temp[2];
            scan_table[i] = sum;
            break;
        }
    }
}

void* SuperScanner::simulate_wrapper(void *context){
  ((SuperScanner*)context)->simulate();
  return 0;
}

void SuperScanner::solveRungeKutta(Vector3f *X, Vector3f *X1){
    if(!sim_mutex){
        for(int i = 0; i < num_nodes; i++){
            X[i] = node_pos[i];
            X[num_nodes+i] = node_vel[i];
        }
        //Runge-Kutta 4th order method.
        
        ODE(X, k1);
        for(int i = 0; i < 2*num_nodes; i++){
            rk4_temp[i] = X[i] + (.5*timestep*k1[i]);
        }
        
        if(sim_mutex) return;
        
        ODE(rk4_temp, k2);
        for(int i = 0; i < 2*num_nodes; i++){
            rk4_temp[i] = X[i] + (.5*timestep*k2[i]);
        }

        if(sim_mutex) return;
            
        ODE(rk4_temp, k3);
        for(int i = 0; i < 2*num_nodes; i++){
            rk4_temp[i] = X[i] + (timestep*k3[i]);
        }

        if(sim_mutex) return;
            
        ODE(rk4_temp, k4);

        for(int i = 0; i < 2*num_nodes; i++){
            X1[i] = X[i] + (timestep/6)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        }
            
        if(sim_mutex) return;
            
        for(int i = 0; i < num_nodes; i++){
            if(constrained_nodes[i]){
                node_pos[i] = node_eq_pos[i];
                continue;
            }
                
            node_pos[i] = X1[i];
            node_vel[i] = X1[i+num_nodes];
        }
            
        has_scan_update = 1;
    }
}



void SuperScanner::solveForwardEuler(Vector3f *X, Vector3f *X1){
    //have k1, k2, k3, k4, and rk4_temp arrays to work with.
    if(!sim_mutex){
        for(int i = 0; i < num_nodes; i++){
            X[i] = node_pos[i];
            X[num_nodes+i] = node_vel[i];
        }
        
        ODE(X, k1);
        for(int i = 0; i < 2*num_nodes; i++){
            X1[i] = X[i] + (timestep*k1[i]);
        }
                    
        for(int i = 0; i < num_nodes; i++){
            if(constrained_nodes[i]){
                node_pos[i] = node_eq_pos[i];
                continue;
            }
                
            node_pos[i] = X1[i];
            node_vel[i] = X1[i+num_nodes];
        }
            
        has_scan_update = 1;
    }
}

void SuperScanner::simulate(){
    struct timeval start, end;

    Vector3f X[num_nodes*2];
    Vector3f X1[num_nodes*2];

    rk4_temp = new Vector3f[num_nodes*2];
    k1 = new Vector3f[num_nodes*2];
    k2 = new Vector3f[num_nodes*2];
    k3 = new Vector3f[num_nodes*2];
    k4 = new Vector3f[num_nodes*2];
    
    while(is_window_open_){
        unsigned int usecs = 1000;
        gettimeofday(&start, NULL);

        //solveBackwardEuler(X, X1);
        //solveForwardEuler(X, X1);
        solveRungeKutta(X, X1);
        
        
        gettimeofday(&end, NULL);
        //ensures each iteration takes a millisecond.
        unsigned int usec_diff = ((end.tv_sec - start.tv_sec)*1000000) + end.tv_usec - start.tv_usec;
        unsigned temp = std::min(usec_diff, usecs); //wait at least 1 millisecond.
        usleep(usecs - temp);
        
        end = start;
        
    }
    printf("Scanner dead\n");
    
    delete[] rk4_temp;
    delete[] k1;
    delete[] k2;
    delete[] k3;
    delete[] k4;
    
}


// Xd = ODE(X);
//Vector3f *acc
void SuperScanner::ODE(Vector3f *X, Vector3f *Xd){
    Vector3f F_restore;
    Vector3f F_damping;
    Vector3f F_spring;
    Vector3f F_sum[num_nodes];
  
    float diff[num_nodes];
  
    Vector3f d_pos[num_nodes];
    float d_pos_norm[num_nodes];


  
    Vector3f eq_dist; //displacement from node eq. position.
  
    //Let N be num_nodes
    float stiffness_boost = stiffness_bias;
    float damping_boost = damping_bias;
    if(release_flag){
        stiffness_boost = release_stiffness;
        damping_boost = release_damping;
        //printf("Release Damping %f\n",release_damping);
    }
  
    for(int i = 0; i < num_nodes; i++){
        //N restoring forces and damping.
        eq_dist = node_eq_pos[i] - X[i];
        F_restore = eq_dist*(.5*(stiffness_boost + restoring_stiffness)*eq_dist.norm());
        F_damping = -X[num_nodes+i]*(damping_boost + (node_damping[i]/100.0)); //N damping forces.

        //N^2 Stuff.
        F_spring[0] = 0;
        F_spring[1] = 0;
        F_spring[2] = 0;
    
        for(int j = 0; j < num_nodes; j++){ //I'm hoping that writing it this way will allow the compiler to vectorize easier. Or later I can vectorize it manually.
            float stiff = stiffness_matrix[(i*num_nodes) + j];
            if(stiff == 0) continue;
            d_pos[j] = X[j] - X[i];
            d_pos_norm[j] = d_pos[j].norm();
            diff[j] = d_pos_norm[j];// - displacement_matrix[(i*num_nodes) + j];
            F_spring += d_pos[j]*(.5*stiff*diff[j]*diff[j]/d_pos_norm[j]);
        }

        F_sum[i] = F_spring + F_restore + F_damping;
    }
    for(int i = 0; i < num_nodes; i++){
        for(int j = 0; j < 3; j++){
            Xd[i+num_nodes][j] = F_sum[i][j] / (mass_bias + (node_mass[i]/10.0));
        }
        Xd[i] = X[i+num_nodes];
    }

    for(int i = 0; i < num_nodes; i++){
        if(constrained_nodes[i]){
            Xd[i][0] = 0;
            Xd[i][1] = 0;
            Xd[i][2] = 0;
            Xd[i+num_nodes][0] = 0;
            Xd[i+num_nodes][1] = 0;
            Xd[i+num_nodes][2] = 0;
        }
    }
  
}

void SuperScanner::release(){
    state_before_release = adsr_state;
    release_flag = 1;
    release_k = k_;
}

void SuperScanner::startNote(){
  k_= 0;
  release_flag = 0;
  adsr_state = 0;
}

void SuperScanner::strike(){
  sim_mutex = 1;
  for(int i = 0; i < scan_len; i++){
    int n = scan_path[i];
    if(constrained_nodes[n]) continue; //hammer doesnt affect constrained nodes.    
    node_pos[n][0] = node_eq_pos[n][0];
    node_pos[n][1] = node_eq_pos[n][1];
    node_pos[n][2] = hammer_table[i];
  }
  for(int i = 0; i < num_nodes; i++){
    node_vel[i][0] = 0;
    node_vel[i][1] = 0;
    node_vel[i][2] = 0;

    //node_acc[i][0] = 0;
    //node_acc[i][1] = 0;
    //node_acc[i][2] = 0;
  }
  sim_mutex = 0;
}

int SuperScanner::start(){
  log_file.open("log_file.csv");
  log_file << "gain,k\n";
  
  is_window_open_ = 1;
  pthread_create(&scan_thread, NULL, &(simulate_wrapper), (void*)this);
  return 0;
}

int SuperScanner::stop(){
  printf("Super scanner says good bye bye\n");
  log_file.close();
  
  is_window_open_ = 0;
  pthread_join(scan_thread, 0);
  return 0;
}

void SuperScanner::setHammer(int num){
    char fn[100];
    memset(fn, 0, sizeof(fn));
    sprintf(fn, "AKWF/AKWF_%04d.wav", num);

    //printf("hammertime\n");
    hammer_num = num;
    if(hammer_num == 0){ //special case.
        for(int i = 0; i < scan_len; i++){
	    hammer_table[i] = 5*sinf(M_PI*i/(scan_len-1));
        }
    }
    else if(hammer_num == 101){ //another special case I felt was worth including.
        for(int i = 1; i < scan_len-1; i++){
            hammer_table[i] = 5*fabs(((scan_len-1) / 2.0) - i) / ((scan_len-1)/2);
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

void SuperScanner::update_params(){
  if(controller.has_new_data()){
    m_volume = controller.get_slider(0)/127.0;
    
    setHammer(controller.get_slider(1) % 102);
    restoring_stiffness = controller.get_slider(2)/8.0;
    
    release_damping = controller.get_slider(3)/127.0;
    release_stiffness = controller.get_slider(4)/127.0;
        
    mass_bias = controller.get_knob(0);
    damping_bias = controller.get_knob(1);
    stiffness_bias = controller.get_knob(2);
    
    scan_method = controller.get_knob(3);
    
    origin[0] = -controller.get_slider(5);
    origin[1] = controller.get_slider(6) - 63.5;
    origin[2] = controller.get_slider(7) - 63.5;
    
    scanner_rot[0] = controller.get_knob(5)*M_PI*2/127.0;
    scanner_rot[1] = controller.get_knob(6)*M_PI*2/127.0;
    scanner_rot[2] = controller.get_knob(7)*M_PI*2/127.0;
    
    
    pad_mode = controller.get_button(0);
  }
}

