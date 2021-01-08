#include "reverb.h"
#include <stdio.h>

/*
 * Delay Line Reverb
 * Schematic:
 * Inspired by http://msp.ucsd.edu/techniques/latest/book-html/node111.html figure 7.15
 *
 * This is so autistic. I can't believe I did this sober
 *
 * "=" is 2 channel signal
 * "-" is 1 channel signal
 *
 *               /->d1-\    /->d2-\    /->d3-\    /->d4-\
 * (Input) ===>(R1)-----=>(R2)-----=>(R3)-----=>(R4)----=>(FeedForward)
 *
 *                    ______/--d5-----\
 *                   (R5,6,7)--d6----\ \
 *                     ^ ^ ^\--d7---\ \ \
 *                     | | | Feed   | | |
 *                 /---|>+ |  Back  | O-|--\ 
 *  FeedForward ===--->+ | |   Loop | | O---===> Output
 *                     \ \ \---(G)--/ / /
 *                      \ \----(G)---/ /
 *                       \-----(G)----/ 
 *
 */                                     

Reverb::Reverb(){
    for(int i = 0; i < 5; i++){
        set_ffwd_rotm(i, 0);
    }
    set_fdbk_rotm(0,0,0);

    wet_ratio = 0; //completely dry
    gain = .2;
    memset(q, 0, sizeof(q));
    for(int i = 0; i < 7; i++){
        q_tail_idx[i] = 0;
        q_size[i] = 1;
        q_idx[i] = 0;
    }
    
}

void Reverb::set_wet_ratio(int value){
    wet_ratio = value / 128.0;
}

void Reverb::set_gain(int g){
    gain = logf(1 + g / 74.0); //74 scales gain from 0 to .999
    //gain = .8 + .2*(g / 128.0); //this might need to be scaled to a different range. Like .999 to .9 or something.
}

void Reverb::set_delay(int index, int delay){
    //q_size[index] = 64*delay + 1; //q_size goes from 1 to 8128
    q_size[index] = floorf(expf(delay/13.1194)); //q_size goes from 1 to 15999
}

void Reverb::set_ffwd_rotm(int index, int value){

    float angle = M_PI * ((float) value / 127.0);
    ffwd_rot_matrix[index][0][0] =  cosf(angle);
    ffwd_rot_matrix[index][0][1] = -sinf(angle);
    ffwd_rot_matrix[index][1][1] =  cosf(angle);
    ffwd_rot_matrix[index][1][0] =  sinf(angle);
}

void Reverb::set_fdbk_rotm(int value_r, int value_p, int value_y){
    float r = M_PI * ((float) value_r / 254.0);
    float p = M_PI * ((float) value_p / 254.0);
    float y = M_PI * ((float) value_y / 254.0);
    
    r -= M_PI*.25;
    p -= M_PI*.25;
    y -= M_PI*.25;
    
    fdbk_rot_matrix[0][0] =  cosf(y)*cosf(p);
    fdbk_rot_matrix[0][1] = -sinf(y)*cosf(r) + cosf(y)*sinf(p)*sinf(r);
    fdbk_rot_matrix[0][2] =  sinf(y)*sinf(r) + cosf(y)*sinf(p)*cosf(r);

    fdbk_rot_matrix[1][0] =  sinf(y)*cosf(p);
    fdbk_rot_matrix[1][1] =  cosf(y)*cosf(r) + sinf(y)*sinf(p)*sinf(r);
    fdbk_rot_matrix[1][2] = -cosf(y)*sinf(r) + sinf(y)*sinf(p)*cosf(r);

    fdbk_rot_matrix[2][0] = -sinf(p);
    fdbk_rot_matrix[2][1] =  cosf(p)*sinf(r);
    fdbk_rot_matrix[2][2] =  cosf(p)*cosf(r);
}

void Reverb::tick(float l, float r, float &out_l, float &out_r){
    float in_l = l;
    float in_r = r;
    
    for(int i = 0; i < 4; i++){
        l = (ffwd_rot_matrix[i][0][0] * l) + (ffwd_rot_matrix[i][0][1] * r); //this does a matrix multiply
        r = (ffwd_rot_matrix[i][1][0] * l) + (ffwd_rot_matrix[i][1][1] * r); //with a 2x2 and audio input
    
        q[i][q_idx[i]] = r; //delay r.
        r = q[i][(q_idx[i] + 1) % q_size[i]]; //get new r
        q_idx[i] = (q_idx[i] + 1) % q_size[i]; //this is the read index
    }
    
    float fdbk_in[3];
    fdbk_in[0] = q[4][(q_idx[4]+1) % q_size[4]]*gain + l;
    fdbk_in[1] = q[5][(q_idx[5]+1) % q_size[5]]*gain + r;
    fdbk_in[2] = q[6][(q_idx[6]+1) % q_size[6]]*gain;
    
    float fdbk_out[3];
    for(int i = 0; i < 3; i++){
        fdbk_out[i] = 0;
        for(int j = 0; j < 3; j++){
            fdbk_out[i] += fdbk_rot_matrix[i][j]*fdbk_in[j];
        }
    }
    
    for(int i = 4; i < 7; i++){
        q[i][q_idx[i]] = fdbk_out[i-4];
        q_idx[i] = (q_idx[i] + 1) % q_size[i]; //get the next item out of the d5 which is q[0]
    }
    
    out_l = fdbk_in[0];
    out_r = fdbk_in[1];

    l = out_l;
    r = out_r;
    
    out_l = (ffwd_rot_matrix[4][0][0] * l) + (ffwd_rot_matrix[4][0][1] * r); //this does a matrix multiply
    out_r = (ffwd_rot_matrix[4][1][0] * l) + (ffwd_rot_matrix[4][1][1] * r); //with a 2x2 and audio input
    
    out_l = wet_ratio*out_l + (1-wet_ratio)*in_l;
    out_r = wet_ratio*out_r + (1-wet_ratio)*in_r;
}

void Reverb::update_params(){
    for(int i = 0; i < 7; i++){
        set_delay(i, controller.get_slider(i));
    }
    for(int i = 0; i < 4; i++){
        set_ffwd_rotm(i, controller.get_knob(i));
    }
    set_ffwd_rotm(4, controller.get_knob(8));
    
    set_fdbk_rotm(controller.get_knob(4), controller.get_knob(5), controller.get_knob(6));
    set_gain(controller.get_slider(7));
    set_wet_ratio(controller.get_knob(7));
}

void Reverb::draw_reverb(Display *dpy, Window w, GC gc){
    static int one_time = 1;
    if(one_time | controller.has_new_data()){
        one_time = 0;
        update_params();
    }
    //else return; //nothing new to display.
    
    XSetForeground(dpy, gc, 0);
    XFillRectangle(dpy, w, gc, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);

    char line[40];
    int bs = 100;
    int text_offset_x = 10;
    int base_offset = 20;
    int line_offset = 20;
    int block_spacing = bs + 30;
    int text_offset_y = 20;
    
    XSetForeground(dpy, gc, 0xFF);
    XFillRectangle(dpy, w, gc, base_offset, SCREEN_HEIGHT/2, bs, bs);
    XDrawLine(dpy, w, gc, 0, SCREEN_HEIGHT/2 + bs/2 - 5, base_offset, SCREEN_HEIGHT/2 + bs/2 - 5);
    XDrawLine(dpy, w, gc, 0, SCREEN_HEIGHT/2 + bs/2 + 5, base_offset, SCREEN_HEIGHT/2 + bs/2 + 5);
    
    XSetForeground(dpy, gc, 0x00);
    sprintf(line, "Delay: %d", controller.get_slider(0));
    XDrawString(dpy, w, gc, base_offset + text_offset_x, SCREEN_HEIGHT/2 + text_offset_y, line, strlen(line));
    sprintf(line, "Angle: %d", controller.get_knob(0));
    XDrawString(dpy, w, gc, base_offset + text_offset_x, SCREEN_HEIGHT/2 + line_offset + text_offset_y, line, strlen(line));
    
    
    for(int i = 1; i < 4; i++){
        XSetForeground(dpy, gc, 0xFF);
        XDrawLine(dpy, w, gc, base_offset + block_spacing*(i-1) + bs, SCREEN_HEIGHT/2 + bs/2 - 5, base_offset + block_spacing*i, SCREEN_HEIGHT/2 + bs/2 - 5);
        XDrawLine(dpy, w, gc, base_offset + block_spacing*(i-1) + bs, SCREEN_HEIGHT/2 + bs/2 + 5, base_offset + block_spacing*i, SCREEN_HEIGHT/2 + bs/2 + 5);
        XFillRectangle(dpy, w, gc, base_offset + block_spacing*i, SCREEN_HEIGHT/2, bs, bs);
        
        XSetForeground(dpy, gc, 0x00);
        sprintf(line, "Delay: %d", controller.get_slider(i));
        XDrawString(dpy, w, gc, base_offset + text_offset_x + block_spacing*i, SCREEN_HEIGHT/2 + text_offset_y, line, strlen(line));
        sprintf(line, "Angle: %d", controller.get_knob(i));
        XDrawString(dpy, w, gc, base_offset + text_offset_x + block_spacing*i, SCREEN_HEIGHT/2 + text_offset_y + line_offset, line, strlen(line));
    } 

    XSetForeground(dpy, gc, 0xFF);
    XDrawLine(dpy, w, gc, base_offset + block_spacing*3 + bs, SCREEN_HEIGHT/2 + bs/2 - 20, base_offset + block_spacing*4.5, SCREEN_HEIGHT/2 + bs/2 - 20);
    XDrawLine(dpy, w, gc, base_offset + block_spacing*3 + bs, SCREEN_HEIGHT/2 + bs/2 + 20, base_offset + block_spacing*5, SCREEN_HEIGHT/2 + bs/2 + 20);

    int diameter = 41;
    int circle1_offset_x = base_offset + block_spacing*5 - diameter/2;
    int circle1_offset_y = SCREEN_HEIGHT/2 + bs/2 + 20 ;
    
    XFillArc(dpy, w, gc, circle1_offset_x - diameter/2, circle1_offset_y - diameter/2, diameter, diameter, 0, 360*64);
    
    int circle2_offset_x = base_offset + block_spacing*4.5 - diameter/2;
    int circle2_offset_y = SCREEN_HEIGHT/2 + bs/2 - 20;
    XSetForeground(dpy, gc, 0xFF);
    XFillArc(dpy, w, gc, circle2_offset_x - diameter/2, circle2_offset_y - diameter/2, diameter, diameter, 0, 360*64);
    
    int fdbk_i_loop_radius = 360 - block_spacing/2; //d is for diameter //inner loop radius
    int fdbk_l_loop_radius = 360; //d is for diameter
    int fdbk_r_loop_radius = 360 + block_spacing/2; //d is for diameter

    int center_x = circle1_offset_x + fdbk_l_loop_radius;
    int center_y = circle1_offset_y;
    
    XDrawArc(dpy, w, gc, circle1_offset_x + block_spacing/2, circle1_offset_y - fdbk_i_loop_radius, fdbk_i_loop_radius*2, fdbk_i_loop_radius*2, 0, 360*64); //inner loop
    XDrawArc(dpy, w, gc, circle1_offset_x, circle1_offset_y - fdbk_l_loop_radius, fdbk_l_loop_radius*2, fdbk_l_loop_radius*2, 0, 360*64); //left side loop
    XDrawArc(dpy, w, gc, circle1_offset_x - block_spacing/2, circle1_offset_y - fdbk_r_loop_radius, fdbk_r_loop_radius*2, fdbk_r_loop_radius*2, 0, 360*64); //right side loop

    XSetForeground(dpy, gc, 0x00);
    XFillRectangle(dpy, w, gc, circle2_offset_x + 6 - diameter/2, circle2_offset_y - 2, diameter - 12, 5);
    XFillRectangle(dpy, w, gc, circle2_offset_x - 2, circle2_offset_y + 6 - diameter/2, 5, diameter - 12);
    XFillRectangle(dpy, w, gc, circle1_offset_x + 6 - diameter/2, circle1_offset_y - 2, diameter - 12, 5);
    XFillRectangle(dpy, w, gc, circle1_offset_x - 2, circle1_offset_y + 6 - diameter/2, 5, diameter - 12);

    XSetForeground(dpy, gc, 0xFF);
    XPoint points[3];
    int npoints = 3;
    int temp[3] = {fdbk_l_loop_radius, fdbk_r_loop_radius, fdbk_i_loop_radius};
    
    for(int i = 0; i < 3; i++){
        points[0].y = center_y - 32 + temp[i];
        points[0].x = center_x - 10;
        points[1].y = center_y + 28 + temp[i];
        points[1].x = center_x - 10;
        points[2].y = center_y + temp[i] - 5;
        points[2].x = center_x - 60;
        XSetForeground(dpy, gc, 0xFF);
        XFillPolygon(dpy, w, gc, points, npoints, Convex, CoordModeOrigin);

        XFillRectangle(dpy, w, gc, center_x - 10, center_y - 16 + temp[i], 80, 29);
        
        XSetForeground(dpy, gc, 0x00);
        sprintf(line, "Delay %d", controller.get_slider(4+i));
        XDrawString(dpy, w, gc, center_x, center_y + 2 + temp[i], line, strlen(line));
        sprintf(line, "%d", controller.get_slider(7));
        XDrawString(dpy, w, gc, center_x - 32, center_y + temp[i] + 2, line, strlen(line));
    }

    XSetForeground(dpy, gc, 0xFF);
    XFillRectangle(dpy, w, gc, center_x - 60, center_y - temp[2] - block_spacing - 45 , 120, 75 + (block_spacing));
    
    char temp_line[] = "Rotation Matrix";
    XSetForeground(dpy, gc, 0x00);
    XDrawString(dpy, w, gc, center_x - 45, center_y - temp[2] - block_spacing - 25, temp_line, strlen(temp_line));
    sprintf(line, "Roll %d", controller.get_knob(4));
    XDrawString(dpy, w, gc, center_x - 20, center_y - temp[1] + 5, line, strlen(line));
    sprintf(line, "Pitch %d", controller.get_knob(5));
    XDrawString(dpy, w, gc, center_x - 20, center_y - temp[0] + 5, line, strlen(line));
    sprintf(line, "Yaw %d", controller.get_knob(6));
    XDrawString(dpy, w, gc, center_x - 20, center_y - temp[2] + 5, line, strlen(line));

    XSetForeground(dpy, gc, 0xFF);
    XDrawLine(dpy, w, gc, center_x - fdbk_r_loop_radius + diameter/2, center_y - 60 + diameter/2, SCREEN_WIDTH*.8, center_y - 60 + diameter/2);
    XDrawLine(dpy, w, gc, center_x - fdbk_l_loop_radius + diameter/2, center_y - 20 + diameter/2, SCREEN_WIDTH*.8, center_y - 20 + diameter/2);

    XFillRectangle(dpy, w, gc, SCREEN_WIDTH*.8, center_y - 120 + diameter/2, 160, 160);
    XDrawLine(dpy, w, gc, SCREEN_WIDTH*.8 + 160, center_y - 60 + diameter/2, SCREEN_WIDTH, center_y - 60 + diameter/2);
    XDrawLine(dpy, w, gc, SCREEN_WIDTH*.8 + 160, center_y - 20 + diameter/2, SCREEN_WIDTH, center_y - 20 + diameter/2);
    
    char wetvdry[] = "Wet / Dry";
    XSetForeground(dpy, gc, 0x00);
    XDrawString(dpy, w, gc, SCREEN_WIDTH*.8 + 50, center_y - 80 + diameter/2, wetvdry, strlen(wetvdry));
    sprintf(line, "%d", controller.get_knob(7));
    XDrawString(dpy, w, gc, SCREEN_WIDTH*.8 + 70, center_y - 60 + diameter/2, line, strlen(line));    
    
    char reverb_balance[] = "Angle ";
    XDrawString(dpy, w, gc, SCREEN_WIDTH*.8 + 50, center_y - 40 + diameter/2, reverb_balance, strlen(reverb_balance));
    sprintf(line, "%d", controller.get_knob(8));
    XDrawString(dpy, w, gc, SCREEN_WIDTH*.8 + 50, center_y - 20 + diameter/2, line, strlen(line));
    XFlush(dpy);
}
