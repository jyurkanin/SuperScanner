#include "audio_engine.h"
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

static SuperScanner *scanner;



void breakOnMe(){
  //break me on, Break on meeeeeee
}


float anti_alias8(float in){
    float num[] = {8.6095831e-08, 6.8876665e-07, 2.4106832e-06, 4.8213665e-06, 6.0267084e-06,
                   4.8213665e-06, 2.4106832e-06, 6.8876665e-07, 8.6095831e-08};
    float den[] = {1,          -6.539904,    18.824587,   -31.133986,    32.34734,
                   -21.611418,     9.064311,    -2.18151,      0.23060125};
    
    const int order = 8;
    static float x[order+1] = {0,0,0,0,0,0,0,0,0};
    static float y[order+1] = {0,0,0,0,0,0,0,0,0};
    
    for(int i = order; i > 0; i--){
        x[i] = x[i-1];
    }
    x[0] = in;
    
    float x_sum = 0;
    float y_sum = 0;
    for(int i = 0; i < order+1; i++){
        x_sum += (x[i] * num[i]);
    }
    for(int i = 1; i < order+1; i++){
        y_sum += (y[i-1] * den[i]);
    }
    for(int i = order; i > 0; i--){
        y[i] = y[i-1];
    }    
    y[0] = x_sum - y_sum;
    
    return y[0];
        
}

float anti_alias4(float in){
    float num[] = {7.2792000e-07, 7.2792004e-06, 3.2756401e-05, 8.7350403e-05, 1.5286320e-04,
                   1.8343584e-04, 1.5286320e-04, 8.7350403e-05, 3.2756401e-05, 7.2792004e-06,
                   7.2792000e-07};
    float den[] = { 1.0000000e+00, -6.3600154e+00,  1.8665571e+01, -3.3157608e+01,
                    3.9371616e+01, -3.2583000e+01,  1.9000641e+01, -7.6987863e+00,
                    2.0719559e+00, -3.3412316e-01,  2.4495861e-02};

    const int order = 10;
    static float x[order+1] = {0,0,0,0,0,0,0,0,0,0,0};
    static float y[order+1] = {0,0,0,0,0,0,0,0,0,0,0};
    
    for(int i = order; i > 0; i--){
        x[i] = x[i-1];
    }
    x[0] = in;
    
    float x_sum = 0;
    float y_sum = 0;
    for(int i = 0; i < order+1; i++){
        x_sum += (x[i] * num[i]);
    }
    for(int i = 1; i < order+1; i++){
        y_sum += (y[i-1] * den[i]);
    }
    for(int i = order; i > 0; i--){
        y[i] = y[i-1];
    }    
    y[0] = x_sum - y_sum;
    
    return y[0];
        
}

float anti_alias2(float in){
    float num[] = {6.82387827e-06, 1.09182052e-04, 8.18865432e-04, 3.82137182e-03,
                   1.24194585e-02, 2.98067015e-02, 5.46456166e-02, 7.80651718e-02,
                   8.78233165e-02, 7.80651718e-02, 5.46456166e-02, 2.98067015e-02,
                   1.24194585e-02, 3.82137182e-03, 8.18865432e-04, 1.09182052e-04,
                   6.82387827e-06};
    float den[] = { 1.0000000e+00, -2.9343348e+00,  6.0569754e+00, -8.4904690e+00,
                    9.4275398e+00, -8.2493067e+00,  5.9339728e+00, -3.4944806e+00,
                    1.7052743e+00, -6.8370658e-01,  2.2441590e-01, -5.9260190e-02,
                    1.2332326e-02, -1.9465445e-03,  2.1958887e-04, -1.5769092e-05,
                    5.4263586e-07};
    
    static float x[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    static float y[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    
    for(int i = 16; i > 0; i--){
        x[i] = x[i-1];
    }
    x[0] = in;
    
    float x_sum = 0;
    float y_sum = 0;
    for(int i = 0; i < 17; i++){
        x_sum += (x[i] * num[i]);
    }
    for(int i = 1; i < 17; i++){
        y_sum += (y[i-1] * den[i]);
    }
    for(int i = 16; i > 0; i--){
        y[i] = y[i-1];
    }    
    y[0] = x_sum - y_sum;
    
    return y[0];
}

void *audio_thread(void *arg){
    float oversample_frames[441*scanner->oversample_factor];
    float sum_frames[441];
    
    int err;
    int frames_to_deliver;
    
    int last_note = -1; //for modes that are monophonic and use portamento
    int curr_note = -1;

    unsigned char volume = 0;
    float stereo_frames[441*NUM_CHANNELS];
    
    while(is_window_open()){
        snd_pcm_wait(playback_handle, 100);
        frames_to_deliver = snd_pcm_avail_update(playback_handle);
        if ( frames_to_deliver == -EPIPE){
            snd_pcm_prepare(playback_handle);
            printf("Epipe\n");
            continue;
        }
      
        frames_to_deliver = frames_to_deliver > 441 ? 441 : frames_to_deliver;
        memset(sum_frames, 0, 441*sizeof(float));
        //memset(stereo_frames, 0, 882*sizeof(float));
      
      
      
        for(int k = 21; k <= 108; k++){
            //this is going to assume that the midi thread handles the sustaining. ANd will only issue a note off if the sustain is not active
            if(midiNotesPressed[k]){
                volume = midiNotesPressed[k];
                midiNotesPressed[k] = 0;

                scanner->startNote();
                if(scanner->pad_mode){
                    scanner->strike();
                }
	  
                last_note = curr_note;
                curr_note = k;
            }
        }
        for(int k = 21; k <= 108; k++){
            if(midiNotesReleased[k]){
                midiNotesReleased[k] = 0; //lets just pray we dont have race conditions.
                if(k == curr_note){ //activate note release.
                    scanner->release();
                    curr_note = -1;
                    last_note = -1;
                }
            }
        }

        float temp_sample;
        for(int j = 0; j < (frames_to_deliver*scanner->oversample_factor); j++){
            temp_sample = scanner->tick(curr_note, volume);
            oversample_frames[j] = anti_alias2(temp_sample);
        }
        
        int idx = 0;
        float sample_l, sample_r;
        float adsr_gain;
        for(int j = 0; j < frames_to_deliver; j++){
            temp_sample = oversample_frames[idx];
            
            scanner->reverb.tick(temp_sample, temp_sample, sample_l, sample_r);
            adsr_gain = scanner->get_adsr_gain();
            
            sample_l = adsr_gain*compress_audio(sample_l, 100, .05, .01, 0);
            sample_r = adsr_gain*compress_audio(sample_r, 100, .05, .01, 1);
            
            stereo_frames[2*j] = sample_l;
            stereo_frames[2*j + 1] = sample_r;
            idx += scanner->oversample_factor;
        }

      
        /*
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
        */
        while((err = snd_pcm_writei(playback_handle, stereo_frames, frames_to_deliver)) != frames_to_deliver && is_window_open()) {
            snd_pcm_prepare(playback_handle);
            fprintf(stderr, "write to audio interface failed (%s)\n", snd_strerror (err));
        }
    }
    printf("Audio Thread is DEAD\n");
    return NULL;
}


int init_midi(int argc, char *argv[]){
  char *MIDI_DEVICE = argv[1];
  int flags;
  
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
  
  return Controller::init_controller(argc, argv);
}


int del_midi(){
  pthread_join(piano_midi_thread, NULL);
  close(MidiFD);

  pthread_join(m_thread, NULL);
  return Controller::exit_controller();
}

int init_audio(SuperScanner * s){
    scanner = s;
  
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
    
    //memset(&sample, 0, sizeof(sample));    
    return EXIT_SUCCESS;
}

int del_audio(){
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
	if(sustain < 64 && packet[2] >= 64){
	  scanner->strike();
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




/*
 * Dsp Functions
 * Audio Compression
 *
 */

float out_rms[2] = {0,0};
float curr_rms[2] = {0,0}; //lpf rms.
float compress_audio(float in, float attack, float threshold, float ratio, int channel){
    float in_rms = fabs(in); //oh this isnt actually the root-mean-square. oh rip.
    float out;
    
    curr_rms[channel] += (in_rms - curr_rms[channel])/attack; //Look at this stupid ass approximation of the signal amplitude.

    float norm_sample = 0;
    //if(curr_rms != 0)
    norm_sample = in / std::max(curr_rms[channel], 1e-3f);
    

    
    if(curr_rms[channel] > threshold){
        out_rms[channel] = (threshold + ((curr_rms[channel] - threshold)*ratio));
        out = norm_sample*out_rms[channel];
    }
    else{
        out_rms[channel] = curr_rms[channel];
        out = norm_sample*curr_rms[channel];
    }
    
    return out;
}

float get_curr_rms(){
    return curr_rms[0];
}

float get_out_rms(){
    return out_rms[0];
}




