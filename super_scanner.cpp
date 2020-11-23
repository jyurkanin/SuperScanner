#include "super_scanner.h"
#include "scanner_window.h"
#include "wavfile.h"
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

  release_stiffness = .1;
  release_damping = .1;
  
  attack_stiffness = 1;
  attack_damping = .001;
  
  
  mass_bias = 0;
  damping_bias = 0;
  stiffness_bias = 0;

  stiffness_adsr = 0;
  damping_adsr = 0;
  
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
    //fill the two diagnols off the main diagnol with 1's for a string.
    stiffness_matrix[(i*num_nodes) + (i+1)] = 1.0;
    displacement_matrix[(i*num_nodes) + (i+1)] = 1.0; //natural displacement of the spring

    stiffness_matrix[((i+1)*num_nodes) + i] = 1.0;
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
  
  
  constrained_nodes = new int[num_nodes];
  memset(constrained_nodes, 0, num_nodes*sizeof(int));
  
  node_damping = new int[num_nodes];
  restoring_stiffness = 0;
  for(int i = 0; i < num_nodes; i++){
    node_damping[i] = 1;
  }
  

  node_pos = new Vector3f[num_nodes];
  node_vel = new Vector3f[num_nodes];
  node_acc = new Vector3f[num_nodes];
  node_eq_pos = new Vector3f[num_nodes];
  for(int i = 0; i < 8; i++){
        for(int j = 0; j < 8; j++){
            node_eq_pos[(i*8)+j] = Vector3f(i*4, j*4, 0);
            node_pos[(i*8)+j] = Vector3f(i*4, j*4, 0);
            node_vel[(i*8)+j] = Vector3f(0, 0, 0);
        }
  }

  constrained_nodes[0] = 1;
  constrained_nodes[7] = 1;
  constrained_nodes[7*8] = 1;
  constrained_nodes[num_nodes-1] = 1;
  
  node_mass = new int[num_nodes];
  for(int i = 0; i < num_nodes; i++){
    node_mass[i] = 1;
  }
  
  timestep = .001; //randomly chosen. .01 was unstable.

  sample_rate = 44100;
  
  //init_params associated with controller
  
  k_ = 0;
  setHammer(0); //Nice. I like this function. Really gets the job done.
}

SuperScanner::~SuperScanner(){
  is_window_open_ = 0;
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

void SuperScanner::start_adsr(){
  k_ = 0;
  release_flag = 1;
}

void SuperScanner::get_adsr_gain(){
  stiffness_adsr = 0;
  damping_adsr = 0;
  
  if(k_ < (44100.0*attack_time)){
    stiffness_adsr = attack_stiffness;
    damping_adsr = attack_damping;
  }
  if(release_flag){
    stiffness_adsr = release_stiffness;
    damping_adsr = release_damping;
  }
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
      p_freq = freq;
  }
  else{
    return 0;
  }

  /*
  if((note != -1) && was_released){p_freq = freq;}
  else if((note != -1) && !was_released){p_freq += (freq - p_freq)/1000;}
  else if((note == -1) && !was_released){
    return 0;
  }
  */
  
  //  if((k_ % 10) == 0){
  //    log_file << freq << "," << p_freq << '\n';
  //  }
  
  compute_scan_table();
  
  float idx = fmod(p_freq*k_*scan_len/sample_rate, scan_len);
  int lower = floorf(idx);
  float diff = idx - lower;
  float sample = scan_table[lower]*(1-diff) + scan_table[lower+1]*(diff); //interpolate along scan table axis
  float old_sample = old_scan_table[lower]*(1-diff) + old_scan_table[lower+1]*(diff);

  diff = std::min(1.0f, (k_ - k_update)/(44100.0f*timestep));
  float i_sample = sample*diff + old_sample*(1-diff); //interpolate along time axis.
  
  
  for(int i = 0; i < scan_len; i++){
      old_scan_table[i] = scan_table[i];
  }
  
  
  k_++;
  was_released = release_flag;
  return i_sample * .005 * m_volume * volume / 127.0;
}

void SuperScanner::compute_scan_table(){
    for(int i = 0; i < scan_len; i++){
        scan_table[i] = node_pos[scan_path[i]][2]; //this is one possible method. Other methods include eq displacement or vector norm.
    }
}

void* SuperScanner::simulate_wrapper(void *context){
  ((SuperScanner*)context)->simulate();
  return 0;
}

void SuperScanner::simulate(){
    struct timeval start, end;

    Vector3f X[num_nodes*2];
    Vector3f X1[num_nodes*2];
    Vector3f temp[num_nodes*2];

    Vector3f k1[num_nodes*2];
    Vector3f k2[num_nodes*2];
    Vector3f k3[num_nodes*2];
    Vector3f k4[num_nodes*2];

    
    while(is_window_open_){
        unsigned int usecs = 1000;
        gettimeofday(&start, NULL);
        
        if(!sim_mutex){
            for(int i = 0; i < num_nodes; i++){
                X[i] = node_pos[i];
                X[num_nodes+i] = node_vel[i];
            }
            //Runge-Kutta 4th order method.
            
            ODE(X, k1);
            for(int i = 0; i < 2*num_nodes; i++){
                temp[i] = X[i] + (.5*timestep*k1[i]);
            }

            if(sim_mutex) continue;
            
            ODE(temp, k2);
            for(int i = 0; i < 2*num_nodes; i++){
                temp[i] = X[i] + (.5*timestep*k2[i]);
            }

            if(sim_mutex) continue;
            
            ODE(temp, k3);
            for(int i = 0; i < 2*num_nodes; i++){
                temp[i] = X[i] + (timestep*k3[i]);
            }

            if(sim_mutex) continue;
            
            ODE(temp, k4);

            for(int i = 0; i < 2*num_nodes; i++){
                X1[i] = X[i] + (timestep/6)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
            }
            
            if(sim_mutex) continue;
            
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

        
        
        gettimeofday(&end, NULL);
        //ensures each iteration takes a millisecond.
        unsigned int usec_diff = ((end.tv_sec - start.tv_sec)*1000000) + end.tv_usec - start.tv_usec;
        unsigned temp = std::min(usec_diff, usecs); //wait at least 1 millisecond.
        usleep(usecs - temp);
        
        end = start;
        
    }
    
    printf("Scanner dead\n");
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
  
  
  //get_adsr_gain();
  
  //Let N be num_nodes
  
  for(int i = 0; i < num_nodes; i++){
    //N restoring forces and damping.
    eq_dist = node_eq_pos[i] - X[i];
    F_restore = eq_dist*(.5*(stiffness_adsr + stiffness_bias + restoring_stiffness)*eq_dist.norm());
    F_damping = -X[num_nodes+i]*(damping_adsr + damping_bias + (node_damping[i]/100.0)); //N damping forces.

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
      //If(diff[j] < 0) continue; //cant push rope, as they say
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
  release_flag = 1;
}

void SuperScanner::strike(){
  release_flag = 0;
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
  log_file << "U,Y\n";
  
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

    printf("hammertime\n");
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
    m_volume = 1;//controller.get_slider(0)/127.0;
    restoring_stiffness = controller.get_slider(1);
    //setHammer(controller.get_slider(2));
    release_stiffness = controller.get_slider(3)/127.0;
    release_damping = controller.get_slider(4)/127.0;

    mass_bias = controller.get_slider(5);
    damping_bias = controller.get_slider(6);
    stiffness_bias = controller.get_slider(7);
  }
}

/*
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
*/
