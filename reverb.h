#pragma once

#include "controller.h"
#include "math.h"
#include "string.h"
#include <X11/Xlib.h>

#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1024
#define MAX_DELAY 16000

class Reverb{
public:
    Reverb();
    void set_ffwd_rotm(int index, int value);
    void set_fdbk_rotm(int r, int p, int y);
    void set_delay(int index, int delay);
    void set_wet_ratio(int ratio);
    void set_gain(int gain);
    void tick(float l, float r, float &out_l, float &out_r);
    void draw_reverb(Display *dpy, Window w, GC gc);
    void activate(){controller.activate();}
    void update_params();
private:
    float fdbk_rot_matrix[3][3];
    float ffwd_rot_matrix[5][2][2];
    float gain;
    float wet_ratio; //wet vs dryness

    float q[7][MAX_DELAY]; //circular queues for delays
    int q_size[7];
    int q_idx[7];
    int q_tail_idx[7];
    
    Controller controller;
};
