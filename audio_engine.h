#pragma once

#include <stdio.h>
#include <fcntl.h>
#include <linux/soundcard.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <alsa/asoundlib.h>
#include <math.h>
#include <stdlib.h>
#include <complex.h>
#include <sys/time.h>
#include "controller.h"
#include "super_scanner.h"


#define MIDI_NOTE_ON 0x90
#define MIDI_NOTE_OFF 0x80
#define PEDAL 176
#define PITCH_BEND 224
#define KEYS 88
#define LOWEST_KEY 21
#define SAMPLE_LEN 4410
#define NUM_CHANNELS 2

typedef unsigned char MidiByte; //ugh


int init_audio(SuperScanner *s);
int del_audio();
int init_midi(int argc, char *argv[]);
int del_midi();
void *midi_loop(void *arg);
void *audio_thread(void *arg);


//Dsp functions

float compress_audio(float in, float attack, float threshold, float ratio, int channel);
float anti_alias2(float in);
float anti_alias4(float in);
float anti_alias8(float in);
float remove_dc_bias(float in, int channel);
float get_curr_rms();
float get_out_rms();
