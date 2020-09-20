#include "audio_engine.h"
#include "super_scanner.h"
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <vector>
#include <queue>
#include <sys/ioctl.h>

snd_pcm_t *playback_handle;

pthread_t m_thread;
pthread_t piano_midi_thread;
int MidiFD;
MidiByte midiNotesPressed[0xFF]; /* this records all the notes jus pressed by pitch maximum notes on is KEYS*/
MidiByte midiNotesReleased[0xFF]; //records the notes just released
MidiByte midiNotesSustained[0xFF];
int sustain = 0;

float freqs[12] = {27.5, 29.135, 30.868, 32.703, 34.648, 36.708, 38.891, 41.203, 43.654, 46.249, 48.999, 51.913}; // frequencies of the lowest octave

Controller controller;
SuperScanner *super_scanner;


void breakOnMe(){
  //break me on, Break on meeeeeee
}

void save_program(char* filename){
  char file_path[110];
  sprintf(file_path, "programs/%s", filename);
  std::ofstream out;
  out.open(file_path, std::ofstream::out);

  //out << synth_algorithms.size() << "\n";
  
  for(int j = 0; j < 9; j++){
    out << controller.slider[j] << " ";
  }
  for(int j = 0; j < 9; j++){
    out << controller.knob[j] << " ";
  }
  for(int j = 0; j < 9; j++){
      out << controller.button[j] << " ";
  }
  
  out.close();
}

void load_program(char* filename){
  std::ifstream in;
  char file_path[110];
  sprintf(file_path, "programs/%s", filename);
  in.open(file_path, std::ifstream::in);
  
  for(int j = 0; j < 9; j++){
    in >> controller.slider[j];
  }
  for(int j = 0; j < 9; j++){
    in >> controller.knob[j];
  }
  for(int j = 0; j < 9; j++){
    in >> controller.button[j];
  }
  
}

Controller* getMainController(){
  return &controller;
}

/* n = which note
 * t = frames since pressed,
 * s = frames since Release state is entered
 * 
 */
float synthesize(int n, int t, int s, int volume, int& state){
  float freq = freqs[(n-21) % 12] * (1 << (1+(int)(n-21)/12));
  return synthesize(freq, t, s, volume, state);
}

float synthesize_portamento(int curr, int last, int t, int s, int volume, int& state){
  static float p_freq = 0; //portamento freq
  float curr_freq = freqs[(curr-21) % 12] * (1 << (1+(int)(curr-21)/12));
  
  if(last == -1){ //no note was being held down. So immedietely play curr_freq.
    p_freq = curr_freq;
  }
  else{
    p_freq += .001*(curr_freq - p_freq);
  }
  
  return super_scanner->tick(p_freq*.5)*volume;
}

void *audio_thread(void *arg){
    float sum_frames[441];
    
    int err;
    int num_on = 0;
    int frames_to_deliver;
    
    int lowest_note;
    int lowest_index;
    
    int last_note = -1; //for modes that are monophonic and use portamento or whatever its called
    int curr_note = -1;
    int is_monophonic = 0; //scanned synthesis only at the moment
    
//    RFilter rfilter(22000, .5);
//    LPFilter lpfilter(22000);
    
    float stereo_frames[441*NUM_CHANNELS];
    
    
    while(is_program_running()){
        snd_pcm_wait(playback_handle, 100);
        frames_to_deliver = snd_pcm_avail_update(playback_handle);
        if ( frames_to_deliver == -EPIPE){
            snd_pcm_prepare(playback_handle);
            printf("Epipe\n");
            continue;
        }
        
        frames_to_deliver = frames_to_deliver > 441 ? 441 : frames_to_deliver;
        memset(sum_frames, 0, 441*sizeof(float));
        
        is_monophonic = 1;
        
        for(int k = 21; k <= 108; k++){
            //this is going to assume that the midi thread handles the sustaining. ANd will only issue a note off if the sustain is not active
            if(midiNotesPressed[k]){
                volume = midiNotesPressed[k];
                midiNotesPressed[k] = 0;
                num_on = 1;
                super_scanner->strike();
                
                index = 1;
                index_s = 0;
                
                if(curr_note != -1 && k != curr_note){ //so that when you press the same note twice it doesnt shut it off
                    index = 0;
                }
                last_note = curr_note;
                curr_note = k;
            }
            else if(midiNotesReleased[k]){
                midiNotesReleased[k] = 0; //lets just pray we dont have race conditions.
                index_s = 1; //this will cause the state to transition to Release
            }
            
            
            if(index){
                for(int j = 0; j < frames_to_deliver; j++){
                    sum_frames[j] = synthesize_portamento(curr_note, last_note, sample.index[k], sample.index_s[k], sample.volume[k], sample.state[k]);
                    sample.index[k]++;
                    if(sample.state[k] == Operator::RELEASE) sample.index_s[k]++;
                }
                
                if(state == Operator::IDLE){
                    curr_note = -1;
                    last_note = -1; //useful for monophonic mode
                    index = 0;
                    index_s = 0;
                    num_on--;
                }
                
                if(!lowest_note){
                    lowest_note = k;
                    lowest_index = sample.index[k];
                }
            }
        }
        
        
        if(main_controller.get_button(1)){
            rfilter.setCutoff(22000*.0078125*(1+main_controller.get_knob(0)));
            rfilter.setQFactor(main_controller.get_knob(2)/128.0);
        }
        if(main_controller.get_button(0)){
            lpfilter.setCutoff(22000*.0078125*(1+main_controller.get_knob(0)));
            lpfilter.setQFactor((1 + main_controller.get_knob(1))/12.8);
        }
        
        if(main_controller.get_button(0) && main_controller.get_button(1)){
            for(int j = 0; j < frames_to_deliver; j++){
                sum_frames[j] = lpfilter.tick(sum_frames[j]) + rfilter.tick(sum_frames[j]);
            }
        }
        else if(main_controller.get_button(0)){
            for(int j = 0; j < frames_to_deliver; j++){
                sum_frames[j] = lpfilter.tick(sum_frames[j]);
            }
        }
        else if(main_controller.get_button(1)){
            for(int j = 0; j < frames_to_deliver; j++){
                sum_frames[j] = rfilter.tick(sum_frames[j]);
            }
        }

        if(main_controller.get_button(2)){
            for(int j = 0; j < frames_to_deliver; j++){
                reverb.tick(sum_frames[j], sum_frames[j], stereo_frames[2*j], stereo_frames[2*j + 1]);
            }
        }
        else{
            for(int j = 0; j < frames_to_deliver; j++){
                stereo_frames[2*j] = sum_frames[j]; //split
                stereo_frames[2*j + 1] = sum_frames[j];
            }
        }
        
        if(num_on){
            set_wave_buffer(lowest_note, lowest_index, frames_to_deliver, sum_frames);
            while((err = snd_pcm_writei (playback_handle, stereo_frames, frames_to_deliver)) != frames_to_deliver && is_window_open()) {
                snd_pcm_prepare (playback_handle);
                fprintf (stderr, "write to audio interface failed (%s)\n", snd_strerror (err));
            }
        }
        else{
            clear_wave_buffer();
            usleep(10);
        }
    }
    printf("Audio Thread is DEADBEEF\n");
    return NULL;
}


int init_midi(int argc, char *argv[]){
  char *MIDI_DEVICE = argv[1];
  int flags;

  if(argc == 3){
    input_mode_ = MIDI_MODE;
    MidiFD = open(MIDI_DEVICE, O_RDONLY);
    
    flags = fcntl(MidiFD, F_GETFL, 0);
    if(fcntl(MidiFD, F_SETFL, flags | O_NONBLOCK)){
      printf("A real bad ERror %d\n", errno);
    }
    
    if(MidiFD < 0){
      printf("Error: Could not open device %s\n", MIDI_DEVICE);
      exit(-1);
    }
    
    pthread_create(&piano_midi_thread, NULL, &midi_loop, NULL);
    pthread_create(&m_thread, NULL, &audio_thread, NULL);
  }
  else{
    init_record();
    input_mode_ = ACOUSTIC_MODE;
    pthread_create(&m_thread, NULL, &audio_thread_s, NULL);
  }
  
  main_controller.activate();
  return Controller::init_controller(argc, argv);
}


int exit_midi(){
  if(input_mode_ == MIDI_MODE){
    pthread_join(piano_midi_thread, NULL);
    close(MidiFD);
  }
  else{
    exit_record();
  }
  pthread_join(m_thread, NULL);
  for(unsigned i = 0; i < synth_algorithms.size(); i++){
    delete synth_algorithms[i];
  }
  return Controller::exit_controller();
}

int init_record(){
    int err;
    unsigned int sample_rate = 44100;
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_uframes_t buffer_size = 441;

    printf("HEy\n");
    capture_handle = 0;
    if ((err = snd_pcm_open (&capture_handle, "hw:3,0", SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf (stderr, "cannot open audio device %s\n", snd_strerror (err));
        return EXIT_FAILURE;
    }
    
    snd_pcm_hw_params_malloc (&hw_params);
    snd_pcm_hw_params_any (capture_handle, hw_params);
    snd_pcm_hw_params_set_access (capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format (capture_handle, hw_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_rate_near (capture_handle, hw_params, &sample_rate, NULL);
    snd_pcm_hw_params_set_buffer_size_near(capture_handle, hw_params, &buffer_size);
    snd_pcm_hw_params_set_channels (capture_handle, hw_params, NUM_CHANNELS);
    snd_pcm_hw_params (capture_handle, hw_params);
    snd_pcm_hw_params_free (hw_params);
    
    snd_pcm_uframes_t buf_size;
    snd_pcm_uframes_t period_size;
    snd_pcm_get_params(capture_handle, &buf_size, &period_size);
    printf("%d derp %d\n", buf_size, period_size);    
    snd_pcm_prepare (capture_handle);

    pthread_create(&cap_thread, NULL, &capture_thread, NULL);
    //capture_thread(NULL);
    return EXIT_SUCCESS;  
}

int exit_record(){
  pthread_join(cap_thread, NULL);
  return snd_pcm_close (capture_handle);
}

int init_alsa(){
    int err;
    unsigned int sample_rate = 44100;
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_sw_params_t *sw_params;
    snd_pcm_uframes_t buffer_size = 441;
    
    playback_handle = 0;
    if ((err = snd_pcm_open (&playback_handle, "default", SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf (stderr, "cannot open audio device %s\n", snd_strerror (err));
        return EXIT_FAILURE;
    }
    
    snd_pcm_hw_params_malloc (&hw_params);
    snd_pcm_hw_params_any (playback_handle, hw_params);
    snd_pcm_hw_params_set_access (playback_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format (playback_handle, hw_params, SND_PCM_FORMAT_FLOAT_LE);
    snd_pcm_hw_params_set_rate_near (playback_handle, hw_params, &sample_rate, NULL);
    snd_pcm_hw_params_set_channels (playback_handle, hw_params, NUM_CHANNELS); //this thing
    snd_pcm_hw_params_set_buffer_size_near(playback_handle, hw_params, &buffer_size);
    snd_pcm_hw_params (playback_handle, hw_params);
    snd_pcm_hw_params_free (hw_params);
    
    /* 
       tell ALSA to wake us up whenever 441 or more frames
       of playback data can be delivered. Also, tell
       ALSA that we'll start the device ourselves.
    */
	
    snd_pcm_sw_params_malloc (&sw_params);
    snd_pcm_sw_params_current (playback_handle, sw_params);
    snd_pcm_sw_params_set_avail_min (playback_handle, sw_params, 441);
    snd_pcm_sw_params_set_start_threshold (playback_handle, sw_params, 0U);
    snd_pcm_sw_params (playback_handle, sw_params);
    snd_pcm_sw_params_free(sw_params);
    
    /* the interface will interrupt the kernel every 441 frames, and ALSA
       will wake up this program very soon after that.
    */
    
    snd_pcm_uframes_t buf_size;
    snd_pcm_uframes_t period_size;
    snd_pcm_get_params(playback_handle, &buf_size, &period_size);
    //printf("%d derp %d\n", buf_size, period_size);    
    snd_pcm_prepare(playback_handle);

    memset(&sample, 0, sizeof(sample));    
    return EXIT_SUCCESS;
}

int exit_alsa(){
    snd_pcm_close (playback_handle);
    snd_config_update_free_global();
    return EXIT_SUCCESS;
}

void *midi_loop(void *ignoreme){
  int bend = 64;
  MidiByte packet[4];
  std::queue<unsigned char> incoming;
  unsigned char temp;
  unsigned char last_status_byte;
  while(is_window_open()){
    if(read(MidiFD, &temp, sizeof(temp)) <= 0){
      usleep(10);
      continue;
    }
    else{
      if(incoming.size() == 0 && !(temp & 0b10000000)){ //so if the first byte in the sequence is not a status byte, use the last status byte.
          incoming.push(last_status_byte);
      }
      incoming.push(temp);
    }
    
    if(incoming.size() >= 3){
      packet[0] = incoming.front(); incoming.pop();
      packet[1] = incoming.front(); incoming.pop();
      packet[2] = incoming.front(); incoming.pop();
    }
    else continue;
    
//    printf("keyboard %d %d %d\n", packet[0], packet[1], packet[2]);
    
    last_status_byte = packet[0];
    
    switch(packet[0] & 0b11110000){
    case(MIDI_NOTE_OFF):
      if(sustain < 64){	
	midiNotesReleased[packet[1]] = 1;
      }
      else{
	midiNotesSustained[packet[1]] = 1;
      }
      break;
    case(MIDI_NOTE_ON):
      midiNotesPressed[packet[1]] = packet[2];            
      break;
    case(PEDAL):
      if(sustain >= 64 && packet[2] < 64){ //if the sustain is released.
	for(int i = 0; i < 128; i++){ //releases all the notes and sets the sustained notes to 0.
	  midiNotesReleased[i] |= midiNotesSustained[i];
	  midiNotesSustained[i] = 0;
	}
      }
      sustain = packet[2];
      break;
    case(PITCH_BEND):
      bend = packet[2];
      break;
    default:
      lseek(MidiFD, 0, SEEK_END);
      break;
    }
    
  }
  printf("Piano Thread is DEADBEEF\n");
  return 0;
}
