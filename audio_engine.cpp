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

void *audio_thread(void *arg){
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
      
      
      
      for(int k = 21; k <= 108; k++){
	//this is going to assume that the midi thread handles the sustaining. ANd will only issue a note off if the sustain is not active
	if(midiNotesPressed[k]){
	  volume = midiNotesPressed[k];
	  midiNotesPressed[k] = 0;
	  scanner->strike();
	  
	  last_note = curr_note;
	  curr_note = k;
	}
	else if(midiNotesReleased[k]){
	  midiNotesReleased[k] = 0; //lets just pray we dont have race conditions.
	  if(k == curr_note){
	    curr_note = -1;
	    last_note = -1;
	  }
	}
      }

      if(curr_note != -1){
	for(int j = 0; j < frames_to_deliver; j++){
	  sum_frames[j] = scanner->tick(curr_note, volume);
	  stereo_frames[2*j] = sum_frames[j];
	  stereo_frames[2*j + 1] = sum_frames[j];
	}
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
