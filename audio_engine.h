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


float synthesize(int n, int t, int s, int volume, int& state);
float synthesize(float freq, int t, int s, int volume, int& state);
float synthesize_portamento(int curr, int last, int t, int s, int volume, int& state);

void load_program(char* filename);
void save_program(char* filename);

int init_alsa();
int exit_alsa();
int init_midi(int argc, char *argv[]);
int exit_midi();
void *midi_loop(void *arg);
void *audio_thread(void *arg);
