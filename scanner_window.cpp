#include "scanner_window.h"
#include "super_scanner.h"
#include "audio_engine.h"
#include <algorithm>
#include <ctype.h>
#include <string.h>



Display *dpy;
Window w;
GC gc;
pthread_t w_thread;
static SuperScanner *scanner;

void breakerbreaker(){}

Vector3f scanner_rot = {0,0,0};
Vector3f origin = {0,0,0};
float viewer_angle = 0;
float d_angle = 0;

unsigned get_adsr_color(unsigned color){
    unsigned scaled_color = color*(std::min(1.0f,scanner->adsr_gain)); //adsr_gain is a max of 2.
    return (scaled_color & color) + 0x00FF;
}

void get_string(char* number){ //read the keyboard for a string
    XEvent e;
    char buf[2];
    KeySym ks = 0;
    int count = 0;
    do{
        if(XPending(dpy) > 0){
            XNextEvent(dpy, &e);
            if(e.type == KeyPress){
                XLookupString(&e.xkey, buf, 1, &ks, NULL);
                if(ks == 0xFF0D){
                    number[count] = 0;
                    return;
                }
                number[count] = buf[0];
                XDrawString(dpy, w, gc, (14+count)*8, 100, buf, 1);
                XFlush(dpy);
                count++;
            }
        }
    } while(1);
}

int get_num(){ //read the keyboard for a number
    XEvent e;
    KeySym ks = 0;
    char buf[2] = {0,0};
    char number[128];
    int count = 0;
    int ret = 0;
    do{
        if(XPending(dpy) > 0){
            XNextEvent(dpy, &e);
            if(e.type == KeyPress){
                XLookupString(&e.xkey, buf, 1, &ks, NULL);
                if(isdigit(buf[0])){
                    number[count] = buf[0];
                    count++;
                }
            }
        }
    } while(ks != 0xFF0D);
    number[count] = 0;
    sscanf(number, "%d", &ret);
    return ret;
}

void draw_float(Display *dpy, Window w, GC gc, int x, int y, float num, int decimals){
    char number[10];
    sprintf(number, "%.3f", num);
    XDrawString(dpy, w, gc, x, y, number, strlen(number));
}

void print_keybindings(){
    int num_keys = 16; //number of elements in following array.
    const char *help_msg[] = {
        "? - Keybinding List",
        "m - toggle mono/stereo view",
        "t - Edit scan Path",
        "n - Mass Table",
        "d - Damping Table",
        "c - Connectivity Menu",
        "p - Pause simulation",
        "r - Resume simulation",
        "x - exit",
        "v - View scan table in 2D",
        "o - Edit constrained Nodes",
        "l - Edit equilibrium position table",
        "y - Rotate viewer",
        "i - info screen",
        "a - envelope screen",
        "s - Save program",
        "l - Load program",
        "b - Cool visualization"
    };
    for(int i = 0; i < num_keys; i++){
        XDrawString(dpy, w, gc, 180, 20+(i*16), help_msg[i], strlen(help_msg[i]));
    }
    get_num(); //wait for enter.
}


void draw_visual_menu(Display *dpy, Window w, GC gc, int mono){
    static unsigned count = 0;
    
    Vector3f temp[scanner->scan_len];
    Vector3f start[scanner->scan_len-1];
    Vector3f end[scanner->scan_len-1];

    Matrix3f rot = get_rotation(0, -.2, 0);

    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 8;
    int offset_x = 4;
    for(int i = 0; i < scanner->scan_len; i++){
        temp[i][0] = count*.0001f*(1+count*.1)*(1+count*.1) + offset_x;
        temp[i][1] = 15*(((float)i/(scanner->scan_len - 1.0f)) - .5);
        temp[i][2] = scanner->scan_table[i]*.03 / get_curr_rms();
        
        temp[i] = rot*temp[i];
    }
    for(int i = 0; i < scanner->scan_len-1; i++){
        start[i] = temp[i];
        end[i] = temp[i+1];
    }
    
    unsigned color = 0x00FF00;//get_adsr_color(0xFF0000); //Better color thing here.
    if(mono){
        XSetForeground(dpy, gc, color);
        draw_mono_lines(start, end, scanner->scan_len);
    }
    else{
        XSetForeground(dpy, gc, color);
        draw_stereo_lines(start, end, scanner->scan_len);
    }

    count++;
    if(scanner->has_strike_visual){
        count = 0;
        scanner->has_strike_visual = 0;
        XClearWindow(dpy, w);
    }
}

void handle_visual_menu(Display *dpy, Window w, GC gc, int &menu_id){
    XEvent e;
    KeySym ks = 0;
    char buf[2];
    
    if(XPending(dpy) > 0){
        XNextEvent(dpy, &e);
        switch(e.type){
        case KeyPress:
            XLookupString(&e.xkey, buf, 1, &ks, NULL);
            switch(buf[0]){
            case 'x':
                menu_id = SCANNER_3D_MENU;
                break;
            }
            break;
        }
    }
}


void draw_adsr_menu(Display *dpy, Window w, GC gc, int node_sel_x){
    int num_divs = 8;
    XSegment segs[num_divs];
    unsigned first_release_state = 1 + floorf(scanner->adsr_table_len*.75);
    
    for(int i = 1; i < num_divs; i++){
        segs[i-1].y1 = SCREEN_HEIGHT*(i/(float)num_divs);
        segs[i-1].y2 = segs[i-1].y1;
        segs[i-1].x1 = 0;
        segs[i-1].x2 = SCREEN_WIDTH;
    }
    
    segs[num_divs-1].y1 = 0;
    segs[num_divs-1].y2 = SCREEN_HEIGHT;
    segs[num_divs-1].x1 = SCREEN_WIDTH*scanner->adsr_table[first_release_state][0];
    segs[num_divs-1].x2 = SCREEN_WIDTH*scanner->adsr_table[first_release_state][0];
    
    XPoint points[scanner->adsr_table_len];
    for(unsigned i = 0; i < scanner->adsr_table_len; i++){
        points[i].x = SCREEN_WIDTH*scanner->adsr_table[i][0];
        points[i].y = SCREEN_HEIGHT*(1-scanner->adsr_table[i][1]);
    }

    XSetForeground(dpy, gc, 0xFF0000);
    XDrawSegments(dpy, w, gc, segs, num_divs-1);
    XSetForeground(dpy, gc, 0xFFFF00);
    XDrawSegments(dpy, w, gc, &segs[num_divs-1], 1);
    XSetForeground(dpy, gc, 0xFF);
    XDrawLines(dpy, w, gc, points, scanner->adsr_table_len, CoordModeOrigin);
    XSetForeground(dpy, gc, 0xFF00);
    XFillArc(dpy, w, gc, scanner->adsr_table[node_sel_x][0]*SCREEN_WIDTH, SCREEN_HEIGHT*(1 - scanner->adsr_table[node_sel_x][1]), 5, 5, 0, 360*64);
}

void handle_reverb_menu(Display *dpy, Window w, GC gc, int &menu_id){
    XEvent e;
    KeySym ks = 0;
    char buf[2];
    
    if(XPending(dpy) > 0){
        XNextEvent(dpy, &e);
        switch(e.type){
        case KeyPress:
            XLookupString(&e.xkey, buf, 1, &ks, NULL);
            switch(buf[0]){
            case 'x':
                menu_id = SCANNER_3D_MENU;
                scanner->controller.activate();
                break;
            }
            break;
        }
    }
}

void handle_adsr_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel_x){
    XEvent e;
    KeySym ks = 0;
    char buf[2];
    
    if(XPending(dpy) > 0){
        XNextEvent(dpy, &e);
        switch(e.type){
        case KeyPress:
            XLookupString(&e.xkey, buf, 1, &ks, NULL);
            switch(buf[0]){
            case 'x':
                menu_id = SCANNER_3D_MENU;
                break;
            }
            switch(XLookupKeysym(&e.xkey, 0)){
            case XK_Left:
                node_sel_x = std::max(1, node_sel_x-1); //I like this function.
                break;
            case XK_Right:
                node_sel_x = std::min((int)scanner->adsr_table_len-2, node_sel_x+1); //me gusta
                break;
            }
            break;
        case ButtonPress:
            switch(e.xbutton.button){
            case 1: //left mouse button clicked
                //printf("CLICK\n");
                scanner->adsr_table[node_sel_x][0] = e.xbutton.x_root/(float)SCREEN_WIDTH;
                scanner->adsr_table[node_sel_x][1] = 1 - (e.xbutton.y_root/(float)SCREEN_HEIGHT);
                node_sel_x = std::min((int)scanner->adsr_table_len-2, node_sel_x+1); //me gusta

                //scanner->sort_adsr_table();
                
                break;
            }
            break;
        }
    }
}


void handle_scanner_2d_menu(Display *dpy, Window w, GC gc, int &menu_id){
    XEvent e;
    KeySym ks = 0;
    char buf[2];
    if(XPending(dpy) > 0){
        XNextEvent(dpy, &e);
        if(e.type == KeyPress){
            XLookupString(&e.xkey, buf, 1, &ks, NULL);
            switch(buf[0]){
            case 'x':
                menu_id = SCANNER_3D_MENU;
                break;
            }
        }
    }
}

void draw_scanner_2d_menu(Display *dpy, Window w, GC gc){
    //printf("Scan Len %d\n", scanner->scan_len);
    float interval_x = SCREEN_WIDTH / (scanner->scan_len-1);
    
    XSegment scan_segs[scanner->scan_len];
    for(int i = 0; i < scanner->scan_len-1; i++){
        scan_segs[i].x1 = i*interval_x;
        scan_segs[i].y1 = (SCREEN_HEIGHT/2) + scanner->scan_table[i]/get_curr_rms();
        scan_segs[i].x2 = (i+1)*interval_x;
        scan_segs[i].y2 = (SCREEN_HEIGHT/2) + scanner->scan_table[i+1]/get_curr_rms();
    }

    unsigned color = get_adsr_color(0xFF0000);
    XSetForeground(dpy, gc, color);
    XDrawSegments(dpy, w, gc, scan_segs, scanner->scan_len);
}

void draw_connectivity_menu(Display *dpy, Window w, GC gc, int node_sel_x, int node_sel_y){
  //Draw Upper Triangular Grid.
  int offset_x = 20;
  int offset_y = 20;
  float interval_x = (SCREEN_WIDTH-(offset_x*2))/((float)scanner->num_nodes);
  float interval_y = (SCREEN_HEIGHT-(offset_y*2))/((float)scanner->num_nodes);
  
  XSegment grid_segs[(scanner->num_nodes + 1) * 2];
  
  grid_segs[0].x1 = offset_x;
  grid_segs[0].y1 = offset_y;
  grid_segs[0].x2 = SCREEN_WIDTH-offset_x;
  grid_segs[0].y2 = offset_y;
  
  for(int i = 1; i < (scanner->num_nodes + 1); i++){
    grid_segs[i].x1 = offset_x + (i-1)*interval_x;
    grid_segs[i].y1 = offset_y + i*interval_y;
    grid_segs[i].x2 = SCREEN_WIDTH - offset_x;
    grid_segs[i].y2 = offset_y + i*interval_y;
  }

  int offset_temp = scanner->num_nodes + 1;
  //cols.
  for(int i = 0; i < scanner->num_nodes; i++){
    grid_segs[i+offset_temp].x1 = offset_x + i*interval_x;
    grid_segs[i+offset_temp].y1 = offset_y;
    grid_segs[i+offset_temp].x2 = offset_x + i*interval_x;
    grid_segs[i+offset_temp].y2 = offset_y + (i+1)*interval_y;
  }
  //last column is different
  offset_temp = offset_temp + scanner->num_nodes;
  grid_segs[offset_temp].x1 = offset_x + (scanner->num_nodes)*interval_x;
  grid_segs[offset_temp].y1 = offset_y;
  grid_segs[offset_temp].x2 = offset_x + (scanner->num_nodes)*interval_x;
  grid_segs[offset_temp].y2 = offset_y + (scanner->num_nodes)*interval_y;

  XSetForeground(dpy, gc, 0xFF00);
  XDrawSegments(dpy, w, gc, grid_segs, offset_temp+1);


  int num_len = 10;
  char number[10];
  //display the stiffness values.
  for(int i = 0; i < scanner->num_nodes; i++){
    for(int j = i; j < scanner->num_nodes; j++){
      sprintf(number, "%.3f", scanner->stiffness_matrix[(i*scanner->num_nodes)+j]);
      num_len = strlen(number);
      XDrawString(dpy, w, gc, offset_x-16 + interval_x*(.5+j), offset_y + 8+ interval_y*(.5+i), number, num_len);
    }
  }


  
  XSegment sel_segments[4];
  
  sel_segments[0].x1 = offset_x + (interval_x*node_sel_x);
  sel_segments[0].y1 = offset_y + (interval_y*node_sel_y);
  sel_segments[0].x2 = offset_x + (interval_x*(node_sel_x+1));
  sel_segments[0].y2 = offset_y + (interval_y*node_sel_y);

  sel_segments[1].x1 = offset_x + (interval_x*(node_sel_x+1));
  sel_segments[1].y1 = offset_y + (interval_y*(node_sel_y));
  sel_segments[1].x2 = offset_x + (interval_x*(node_sel_x+1));
  sel_segments[1].y2 = offset_y + (interval_y*(node_sel_y+1));

  sel_segments[2].x1 = offset_x + (interval_x*(node_sel_x+1));
  sel_segments[2].y1 = offset_y + (interval_y*(node_sel_y+1));
  sel_segments[2].x2 = offset_x + (interval_x*(node_sel_x));
  sel_segments[2].y2 = offset_y + (interval_y*(node_sel_y+1));

  sel_segments[3].x1 = offset_x + (interval_x*(node_sel_x));
  sel_segments[3].y1 = offset_y + (interval_y*(node_sel_y+1));
  sel_segments[3].x2 = offset_x + (interval_x*(node_sel_x));
  sel_segments[3].y2 = offset_y + (interval_y*(node_sel_y));
  
  
  XSetForeground(dpy, gc, 0xFF0000);
  XDrawSegments(dpy, w, gc, sel_segments, 4);
}


void handle_connectivity_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel_x, int &node_sel_y){
  XEvent e;
  KeySym ks = 0;
  char buf[2];
  static char number[128];
  static int count = 0;
  float parsed_num = 0;
  float max_value = 20; //random.
  
  
  if(XPending(dpy) > 0){
      XNextEvent(dpy, &e);
      if(e.type == KeyPress){
          XLookupString(&e.xkey, buf, 1, &ks, NULL);

          if(isdigit(buf[0]) || (buf[0] == '.')){
              number[count] = buf[0];
              count++;
          }
          if(ks == 0xFF0D){
              number[count] = 0;
              sscanf(number, "%f", &parsed_num);
              if(parsed_num < max_value){
                  scanner->stiffness_matrix[(node_sel_y*scanner->num_nodes) + node_sel_x] = parsed_num; //have to get the index at the opposite side of the diagonal too.
                  scanner->stiffness_matrix[(node_sel_x*scanner->num_nodes) + node_sel_y] = parsed_num; //maintain symmetric constrain
              }
              count = 0;
          }
      
          switch(buf[0]){
          case 'x':
              scanner->sim_mutex = 0;
              menu_id = SCANNER_3D_MENU;
              break;
          }
          switch(XLookupKeysym(&e.xkey, 0)){
          case XK_Left:
              node_sel_x = std::max(node_sel_y+1, node_sel_x-1); //I like this function.
              count = 0;
              break;
          case XK_Right:
              node_sel_x = std::min(scanner->num_nodes-1, node_sel_x+1); //me gusta
              count = 0;
              break;
          case XK_Down:
              node_sel_y = std::min(node_sel_x-1, node_sel_y+1); //me gusta
              count = 0;
              break;
          case XK_Up:
              node_sel_y = std::max(0, node_sel_y-1); //I like this function.
              count = 0;
              break;
          }
      }
  }  
}




//draw all connections.
void draw_scanner(Display *dpy, Window w, GC gc, int mono){
  int actual_len = 0;
  int num_nodes = scanner->num_nodes;
  Vector3f start[1+(num_nodes*num_nodes/2)]; //throw in a random +1 to avoid off by one errors. in case num_nodes=7, 7*7/2 = 49/2 = 24.5 = 24 which would be one less than needed.
  Vector3f end[1+(num_nodes*num_nodes/2)];
  Vector3f node_pos_rot[scanner->num_nodes];
  
  Matrix3f rot = get_rotation(scanner_rot[0], scanner_rot[1]+viewer_angle, scanner_rot[2]);
  viewer_angle += d_angle;
  
  for(int i = 0; i < num_nodes; i++){ //row
      node_pos_rot[i] = rot*scanner->node_pos[i];
      for(int j = i; j < num_nodes; j++){ //column. //searches only upper Triangluar part.
          if(scanner->stiffness_matrix[(i*num_nodes)+j] > 0){ //connection exists. Add to list.
              start[actual_len] = rot*scanner->node_pos[i];
              end[actual_len] = rot*scanner->node_pos[j];
              actual_len++;
          }
      }
  }

  unsigned color = get_adsr_color(0xFF0000);
  
  if(mono){
      XSetForeground(dpy, gc, color);
      draw_mono_lines(start, end, actual_len);
      XSetForeground(dpy, gc, 0xFF);
      draw_mono_points(node_pos_rot, num_nodes);
  }
  else{
      XSetForeground(dpy, gc, color);
      draw_stereo_lines(start, end, actual_len);
      XSetForeground(dpy, gc, 0xFF);
      draw_stereo_points(node_pos_rot, num_nodes);
  }
}


void handle_scanner_menu(Display *dpy, Window w, GC gc, int &menu_id, int &mono){
    XEvent e;
    char buf[2];
    char filename[100];
    
    //MAIN MENU
    if(XPending(dpy) > 0){
        XNextEvent(dpy, &e);
        if(e.type == KeyPress){
            XLookupString(&e.xkey, buf, 1, NULL, NULL);
            switch(buf[0]){
            case ' ':
                scanner->strike();
                break;
            case 'm':
                mono = !mono;
                break;
            case 't':
                scanner->sim_mutex = 1;
                menu_id = SCAN_PATH_MENU;
                break;
            case 'n':
                scanner->sim_mutex = 1;
                menu_id = NODE_MASS_MENU;
                break;
            case 'd':
                scanner->sim_mutex = 1;
                menu_id = NODE_DAMPING_MENU;
                break;
            case 'c':
                scanner->sim_mutex = 1;
                menu_id = CONNECTIVITY_MENU;
                break;
            case 'p':
                scanner->sim_mutex = 1;
                break;
            case 'r':
                scanner->sim_mutex = 0;
                break;
            case 'x':
                scanner->stop();
                printf("Window Thread, Peacin' out\n");
                break;
            case 'v':
                menu_id = SCANNER_2D_MENU;
                break;
            case 'o':
                scanner->sim_mutex = 1;
                menu_id = CONSTRAINT_MENU;
                break;
            case 'q':
                scanner->sim_mutex = 1;
                menu_id = EQ_POS_MENU;
                break;
            case 'y':
                if(d_angle == 0)
                    d_angle = -.01;
                else
                    d_angle = 0;
                break;
            case 'i':
                menu_id = INFO_MENU;
                break;
            case 'a':
                menu_id = ADSR_MENU;
                break;
            case 'e':
                scanner->reverb.activate();
                menu_id = REVERB_MENU;
                break;
            case 'l':
                XDrawString(dpy, w, gc, 8*8, 100, "Load: ", 6);
                get_string(filename);
                scanner->load(filename);
                break;
            case 's':
                XDrawString(dpy, w, gc, 8*8, 100, "Save: ", 6);
                get_string(filename);
                scanner->save(filename);
                break;
            case 'b': //o for observe.
                menu_id = VISUAL_MENU;
                break;
            case '?':
                print_keybindings();
                break;
            }
            
        }
    }
}

void draw_node_constraint_menu(Display *dpy, Window w, GC gc, int &node_sel){
    XDrawString(dpy, w, gc, SCREEN_WIDTH/2, 20, "Node Constraint Menu", 17);
    draw_scanner_node_menu(dpy, w, gc, node_sel, scanner->constrained_nodes);
}
void draw_node_damping_menu(Display *dpy, Window w, GC gc, int node_sel){
  XDrawString(dpy, w, gc, SCREEN_WIDTH/2, 20, "Node Damping Menu", 17);
  draw_scanner_node_menu(dpy, w, gc, node_sel, scanner->node_damping);
}
void draw_scan_path_menu(Display *dpy, Window w, GC gc, int node_sel){
  XDrawString(dpy, w, gc, SCREEN_WIDTH/2, 20, "Scanner Path Menu", 17);
  draw_scanner_node_menu(dpy, w, gc, node_sel, scanner->scan_path);
}
void draw_node_mass_menu(Display *dpy, Window w, GC gc, int node_sel){
  XDrawString(dpy, w, gc, SCREEN_WIDTH/2, 20, "Node Mass Menu", 14);
  draw_scanner_node_menu(dpy, w, gc, node_sel, scanner->node_mass);
}

void draw_eq_pos_menu(Display *dpy, Window w, GC gc, int node_sel_x, int node_sel_y){
    int actual_len = 0;
    int num_nodes = scanner->num_nodes;
    Vector3f start[1+(num_nodes*num_nodes/2)]; //throw in a random +1 to avoid off by one errors. in case num_nodes=7, 7*7/2 = 49/2 = 24.5 = 24 which would be one less than needed.
    Vector3f end[1+(num_nodes*num_nodes/2)];
    Vector3f node_pos_rot[scanner->num_nodes];
    
    Matrix3f rot = get_rotation(scanner_rot[0], scanner_rot[1] + viewer_angle, scanner_rot[2]);
    viewer_angle += d_angle;
  
    for(int i = 0; i < num_nodes; i++){ //row
        node_pos_rot[i] = rot*scanner->node_pos[i];
        for(int j = i; j < num_nodes; j++){ //column. //searches only upper Triangluar part.
            if(scanner->stiffness_matrix[(i*num_nodes)+j] > 0){ //connection exists. Add to list.
                start[actual_len] = rot*scanner->node_pos[i];
                end[actual_len] = rot*scanner->node_pos[j];
                actual_len++;
            }
        }
    }
    
    XSetForeground(dpy, gc, 0xFF0000);
    draw_mono_lines(start, end, actual_len);
    XSetForeground(dpy, gc, 0xFF00);
    draw_node_labels(node_pos_rot, num_nodes);

    
    float offset_x = 20;
    float interval_x = (SCREEN_WIDTH - (offset_x*2)) / (scanner->scan_len-1);
    float interval_y = 20;
    XSegment sel_segs[4];
    XSegment segments[4 + scanner->scan_len + 1]; //3 rows, num_nodes cols.
    
    sel_segs[0].x1 = interval_x*node_sel_x + offset_x;
    sel_segs[0].y1 = SCREEN_HEIGHT - (interval_y*node_sel_y);
    sel_segs[0].x2 = interval_x*(node_sel_x+1) + offset_x;
    sel_segs[0].y2 = SCREEN_HEIGHT - (interval_y*node_sel_y);

    sel_segs[1].x1 = interval_x*(node_sel_x+1) + offset_x;
    sel_segs[1].y1 = SCREEN_HEIGHT - (interval_y*node_sel_y);
    sel_segs[1].x2 = interval_x*(node_sel_x+1) + offset_x;
    sel_segs[1].y2 = SCREEN_HEIGHT - (interval_y*(node_sel_y+1));

    sel_segs[2].x1 = interval_x*(node_sel_x+1) + offset_x;
    sel_segs[2].y1 = SCREEN_HEIGHT - (interval_y*(node_sel_y+1));
    sel_segs[2].x2 = interval_x*(node_sel_x) + offset_x;
    sel_segs[2].y2 = SCREEN_HEIGHT - (interval_y*(node_sel_y+1));

    sel_segs[3].x1 = interval_x*(node_sel_x) + offset_x;
    sel_segs[3].y1 = SCREEN_HEIGHT - (interval_y*(node_sel_y+1));
    sel_segs[3].x2 = interval_x*(node_sel_x) + offset_x;
    sel_segs[3].y2 = SCREEN_HEIGHT - (interval_y*(node_sel_y));
    
    for(int i = 0; i < 4; i++){
        segments[i].x1 = offset_x;
        segments[i].y1 = SCREEN_HEIGHT - i*interval_y - 1;
        segments[i].x2 = SCREEN_WIDTH - offset_x;
        segments[i].y2 = SCREEN_HEIGHT - i*interval_y - 1;
    }

    for(int i = 0; i < scanner->scan_len; i++){
        segments[i+4].x1 = (interval_x * i) + offset_x;
        segments[i+4].y1 = SCREEN_HEIGHT - (interval_y*3) - 1;
        segments[i+4].x2 = (interval_x * i) + offset_x;
        segments[i+4].y2 = SCREEN_HEIGHT;
    }
    

    int num_len;
    char num[12];
    for(int i = 0; i < scanner->num_nodes; i++){
        for(int j = 0; j < 3; j++){
            sprintf(num, "%.1f", scanner->node_eq_pos[i][2-j]);
            num_len = strlen(num);
            XDrawString(dpy, w, gc, (i+.5)*interval_x + offset_x, SCREEN_HEIGHT - ((j+.5)*interval_y), num, num_len);
        }
    }
    
    
    XSetForeground(dpy, gc, 0xFF0000);
    XDrawSegments(dpy, w, gc, sel_segs, 4);
    
    XSetForeground(dpy, gc, 0xFF00);
    XDrawSegments(dpy, w, gc, segments, 4 + scanner->num_nodes + 1);
    
}

void draw_scanner_node_menu(Display *dpy, Window w, GC gc, int node_sel, int *params){
  //Draw 1/2 size mono scanner. Label Nodes.
  int actual_len = 0;
  int num_nodes = scanner->num_nodes;
  Vector3f start[1+(num_nodes*num_nodes/2)]; //throw in a random +1 to avoid off by one errors. in case num_nodes=7, 7*7/2 = 49/2 = 24.5 = 24 which would be one less than needed.
  Vector3f end[1+(num_nodes*num_nodes/2)];
  Vector3f node_pos_rot[scanner->num_nodes];
  
  Matrix3f rot = get_rotation(scanner_rot[0], scanner_rot[1] + viewer_angle, scanner_rot[2]);
  viewer_angle += d_angle;
  
  for(int i = 0; i < num_nodes; i++){ //row
    node_pos_rot[i] = rot*scanner->node_pos[i];
    for(int j = i; j < num_nodes; j++){ //column. //searches only upper Triangluar part.
      if(scanner->stiffness_matrix[(i*num_nodes)+j] > 0){ //connection exists. Add to list.
          start[actual_len] = rot*scanner->node_pos[i];
          end[actual_len] = rot*scanner->node_pos[j];
          actual_len++;
      }
    }
  }

  XSetForeground(dpy, gc, 0xFF0000);
  draw_mono_lines(start, end, actual_len);
  XSetForeground(dpy, gc, 0xFF00);
  draw_node_labels(node_pos_rot, num_nodes);

  draw_text_boxes(params);
  draw_select_box(node_sel);
}



void draw_select_box(int node_sel){
  XSegment sel_segments[4];
  float interval = (SCREEN_WIDTH-40.0)/scanner->scan_len;
  sel_segments[0].x1 = 20 + (interval*node_sel);
  sel_segments[0].y1 = SCREEN_HEIGHT - 20;
  sel_segments[0].x2 = 20 + (interval*node_sel);
  sel_segments[0].y2 = SCREEN_HEIGHT - 40;

  sel_segments[1].x1 = 20 + (interval*(node_sel+1));
  sel_segments[1].y1 = SCREEN_HEIGHT - 20;
  sel_segments[1].x2 = 20 + (interval*(node_sel+1));
  sel_segments[1].y2 = SCREEN_HEIGHT - 40;

  sel_segments[2].x1 = 20 + (interval*(node_sel+1));
  sel_segments[2].y1 = SCREEN_HEIGHT - 20;
  sel_segments[2].x2 = 20 + (interval*node_sel);
  sel_segments[2].y2 = SCREEN_HEIGHT - 20;

  sel_segments[3].x1 = 20 + (interval*(node_sel+1));
  sel_segments[3].y1 = SCREEN_HEIGHT - 40;
  sel_segments[3].x2 = 20 + (interval*node_sel);
  sel_segments[3].y2 = SCREEN_HEIGHT - 40;

  XSetForeground(dpy, gc, 0xFF0000);  
  XDrawSegments(dpy, w, gc, sel_segments, 4);
}

void draw_text_boxes(int * params){
  //Draw Scan Path
  int num_segs = 4 + scanner->scan_len - 1;
  XSegment segments[num_segs];
  
  //Left segment
  segments[0].x1 = 20;
  segments[0].y1 = SCREEN_HEIGHT-20;
  segments[0].x2 = 20;
  segments[0].y2 = SCREEN_HEIGHT-40;

  //Right segment
  segments[1].x1 = SCREEN_WIDTH-20;
  segments[1].y1 = SCREEN_HEIGHT-20;
  segments[1].x2 = SCREEN_WIDTH-20;
  segments[1].y2 = SCREEN_HEIGHT-40;
  
  //Top
  segments[2].x1 = 20;
  segments[2].y1 = SCREEN_HEIGHT-40;
  segments[2].x2 = SCREEN_WIDTH-20;
  segments[2].y2 = SCREEN_HEIGHT-40;

  //Bottom
  segments[3].x1 = 20;
  segments[3].y1 = SCREEN_HEIGHT-20;
  segments[3].x2 = SCREEN_WIDTH-20;
  segments[3].y2 = SCREEN_HEIGHT-20;
  
  
  int num;
  char snum[4];
  int num_len;
  float interval = (SCREEN_WIDTH-40.0)/scanner->scan_len;
  for(int i = 0; i < scanner->scan_len; i++){
    segments[4+i].x1 = 20 + (interval*(1+i));
    segments[4+i].y1 = SCREEN_HEIGHT - 20;
    segments[4+i].x2 = 20 + (interval*(1+i));
    segments[4+i].y2 = SCREEN_HEIGHT - 40;
    
    num = params[i];
    sprintf(snum, "%d", num);
    num_len = 0;
    if(num < 10)
      num_len = 1;
    else if(num < 100)
      num_len = 2;
    else if(num < 1000)
      num_len = 3;
    
    XDrawString(dpy, w, gc, 22 + (interval*i), SCREEN_HEIGHT-22, snum, num_len);
  }  
  
  XDrawSegments(dpy, w, gc, segments, num_segs);
}

void handle_eq_pos_menu(Display *dpy, Window w, GC gc, int &node_sel_x, int &node_sel_y, int &menu_id){
    XEvent e;
    KeySym ks = 0;
    char buf[2];
    static char number[128];
    static int count = 0;
    float parsed_num = 0;
    float max_value = 100; //random.
    
  
    if(XPending(dpy) > 0){
        XNextEvent(dpy, &e);
        if(e.type == KeyPress){
            XLookupString(&e.xkey, buf, 1, &ks, NULL);
            
            if(isdigit(buf[0]) || (buf[0] == '.')){
                number[count] = buf[0];
                count++;
            }
            if(ks == 0xFF0D){
                number[count] = 0;
                sscanf(number, "%f", &parsed_num);
                if(parsed_num < max_value){
                    scanner->node_eq_pos[node_sel_x][2-node_sel_y] = parsed_num;
                }
                count = 0;
            }
            
            switch(buf[0]){
            case 'x':
                scanner->sim_mutex = 0;
                menu_id = SCANNER_3D_MENU;
                break;
            }
            switch(XLookupKeysym(&e.xkey, 0)){
            case XK_Left:
                node_sel_x = std::max(0, node_sel_x-1); //I like this function.
                count = 0;
                break;
            case XK_Right:
                node_sel_x = std::min(scanner->num_nodes-1, node_sel_x+1); //me gusta
                count = 0;
                break;
            case XK_Down:
                node_sel_y = std::max(0, node_sel_y-1); //I like this function.
                count = 0;
                break;
            case XK_Up:
                node_sel_y = std::min(2, node_sel_y+1); //me gusta
                count = 0;
                break;
            }
        }
    }  
}

void handle_node_constraint_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel){
  handle_scanner_node_menu(dpy, w, gc, menu_id, node_sel, scanner->constrained_nodes, 100);
}
void handle_node_damping_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel){
  handle_scanner_node_menu(dpy, w, gc, menu_id, node_sel, scanner->node_damping, 100);
}
void handle_node_mass_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel){
  handle_scanner_node_menu(dpy, w, gc, menu_id, node_sel, scanner->node_mass, 10);
}
void handle_scan_path_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel){
  handle_scanner_node_menu(dpy, w, gc, menu_id, node_sel, scanner->scan_path, scanner->scan_len);
}
void handle_scanner_node_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel, int *params, int max_value){
  XEvent e;
  KeySym ks = 0;
  char buf[2];
  static char number[128];
  static int count = 0;
  int parsed_num = 0;
  
  if(XPending(dpy) > 0){
    XNextEvent(dpy, &e);
    if(e.type == KeyPress){
      XLookupString(&e.xkey, buf, 1, &ks, NULL);

      if(isdigit(buf[0])){
	number[count] = buf[0];
	count++;
      }
      if(ks == 0xFF0D){
	number[count] = 0;
	sscanf(number, "%d", &parsed_num);
	if(parsed_num < max_value){
	  params[node_sel] = parsed_num;
	}
	count = 0;
      }
      
      switch(buf[0]){
      case 'x':
	scanner->sim_mutex = 0;
	menu_id = SCANNER_3D_MENU;
	break;
      }
      switch(XLookupKeysym(&e.xkey, 0)){
      case XK_Left:
	node_sel = std::max(0, node_sel-1); //I like this function.
	count = 0;
	break;
      case XK_Right:
	node_sel = std::min(scanner->num_nodes-1, node_sel+1); //me gusta
	count = 0;
	break;
      case XK_Up:
	params[node_sel] = std::min(params[node_sel]+1, scanner->num_nodes);
	break;
      case XK_Down:
	params[node_sel] = std::max(params[node_sel]-1, 0);
	break;
      }
    }
  }
}

void handle_info_menu(Display *dpy, Window w, GC gc, int &menu_id){
    XEvent e;
    KeySym ks = 0;
    char buf[2];
    if(XPending(dpy) > 0){
        XNextEvent(dpy, &e);
        if(e.type == KeyPress){
            XLookupString(&e.xkey, buf, 1, &ks, NULL);
            switch(buf[0]){
            case 'x':
                menu_id = SCANNER_3D_MENU;
                break;
            }
        }
    }

}

void draw_info_menu(Display *dpy, Window w, GC gc){
    draw_scanner(dpy, w, gc, 1); //nice.
    
    const int num_segments = 28;
    XSegment segments[num_segments];

    XSetForeground(dpy, gc, 0xFF);

    int pos_table_x = 100;
    int pos_table_y = SCREEN_HEIGHT - 120;
    int pos_table_height = 36;
    int pos_table_width = 140;
    //this is the position and orientation table.
    //top rectangle.
    segments[0].x1 = pos_table_x;
    segments[0].y1 = pos_table_y;
    segments[0].x2 = pos_table_x + pos_table_width;
    segments[0].y2 = pos_table_y;

    segments[1].x1 = pos_table_x + pos_table_width;
    segments[1].y1 = pos_table_y;
    segments[1].x2 = pos_table_x + pos_table_width;
    segments[1].y2 = pos_table_y + 13+12+pos_table_height;
        
    segments[2].x1 = pos_table_x;
    segments[2].y1 = pos_table_y + 13;
    segments[2].x2 = pos_table_x + pos_table_width;
    segments[2].y2 = pos_table_y + 13;
    
    segments[3].x1 = pos_table_x;
    segments[3].y1 = pos_table_y;
    segments[3].x2 = pos_table_x;
    segments[3].y2 = pos_table_y + 13+12+pos_table_height;

    segments[4].x1 = pos_table_x - 10;
    segments[4].y1 = pos_table_y + 13 + 12;
    segments[4].x2 = pos_table_x - 10;
    segments[4].y2 = pos_table_y + 13+12+pos_table_height;

    segments[5].x1 = pos_table_x - 10;
    segments[5].y1 = pos_table_y + 13+12+pos_table_height;
    segments[5].x2 = pos_table_x + pos_table_width;
    segments[5].y2 = pos_table_y + 13+12+pos_table_height;

    segments[6].x1 = pos_table_x - 10;
    segments[6].y1 = pos_table_y + 13 + 12+(pos_table_height/3);
    segments[6].x2 = pos_table_x + pos_table_width;
    segments[6].y2 = pos_table_y + 13 + 12+(pos_table_height/3);

    segments[7].x1 = pos_table_x - 10;
    segments[7].y1 = pos_table_y + 13 + 12+(2*pos_table_height/3);
    segments[7].x2 = pos_table_x + pos_table_width;
    segments[7].y2 = pos_table_y + 13 + 12+(2*pos_table_height/3);

    segments[8].x1 = pos_table_x - 10;
    segments[8].y1 = pos_table_y + 13+12;
    segments[8].x2 = pos_table_x + pos_table_width;
    segments[8].y2 = pos_table_y + 13+12;

    segments[9].x1 = pos_table_x + (pos_table_width/2);
    segments[9].y1 = pos_table_y + 13;
    segments[9].x2 = pos_table_x + (pos_table_width/2);
    segments[9].y2 = pos_table_y + 13+12+pos_table_height;
    
    XDrawString(dpy, w, gc, pos_table_x + (pos_table_width/2) - (72/2), SCREEN_HEIGHT - 109, "Camera Frame", 12);
    XDrawString(dpy, w, gc, pos_table_x + (pos_table_width/4) - (48/2), pos_table_y + 13+11, "Position", 8);
    XDrawString(dpy, w, gc, pos_table_x + (3*pos_table_width/4) - (48/2), pos_table_y + 13+11, "Rotation", 8);
    XDrawString(dpy, w, gc, pos_table_x - 10 + 3, pos_table_y + 13+12-1 + (pos_table_height/3), "X", 1);
    XDrawString(dpy, w, gc, pos_table_x - 10 + 3, pos_table_y + 13+12-1 + (2*pos_table_height/3), "Y", 1);
    XDrawString(dpy, w, gc, pos_table_x - 10 + 3, pos_table_y + 13+12-1 + pos_table_height, "Z", 1);
    
    draw_float(dpy, w, gc, pos_table_x+8, pos_table_y + 13+12-1 + (pos_table_height/3), origin[0], 2);
    draw_float(dpy, w, gc, pos_table_x+8, pos_table_y + 13+12-1 + (2*pos_table_height/3), origin[1], 2);
    draw_float(dpy, w, gc, pos_table_x+8, pos_table_y + 13+12-1 + pos_table_height, origin[2], 2);
    
    draw_float(dpy, w, gc, pos_table_x+8+(pos_table_width/2), pos_table_y + 13+12-1 + (pos_table_height/3), scanner_rot[0], 2);
    draw_float(dpy, w, gc, pos_table_x+8+(pos_table_width/2), pos_table_y + 13+12-1 + (2*pos_table_height/3), scanner_rot[1], 2);
    draw_float(dpy, w, gc, pos_table_x+8+(pos_table_width/2), pos_table_y + 13+12-1 + pos_table_height, scanner_rot[2], 2);
    
    int bias_table_x = 300;
    int bias_table_y = pos_table_y;
    int bias_table_width = 400;
    int bias_table_height = 24;
    
    segments[10].x1 = bias_table_x;
    segments[10].y1 = bias_table_y;
    segments[10].x2 = bias_table_x + bias_table_width;
    segments[10].y2 = bias_table_y;
    
    segments[11].x1 = bias_table_x + bias_table_width;
    segments[11].y1 = bias_table_y;
    segments[11].x2 = bias_table_x + bias_table_width;
    segments[11].y2 = bias_table_y + 12 + bias_table_height;
    
    segments[12].x1 = bias_table_x + bias_table_width;
    segments[12].y1 = bias_table_y + 12;
    segments[12].x2 = bias_table_x;
    segments[12].y2 = bias_table_y + 12;
    
    segments[13].x1 = bias_table_x;
    segments[13].y1 = bias_table_y + 12 + bias_table_height;
    segments[13].x2 = bias_table_x;
    segments[13].y2 = bias_table_y;

    segments[14].x1 = bias_table_x;
    segments[14].y1 = bias_table_y + 12 + bias_table_height;
    segments[14].x2 = bias_table_x + bias_table_width;
    segments[14].y2 = bias_table_y + 12 + bias_table_height;

    segments[15].x1 = bias_table_x;
    segments[15].y1 = bias_table_y + 12 + (bias_table_height/2);
    segments[15].x2 = bias_table_x + bias_table_width;
    segments[15].y2 = bias_table_y + 12 + (bias_table_height/2);

    segments[16].x1 = bias_table_x + bias_table_width/2;
    segments[16].y1 = bias_table_y;
    segments[16].x2 = bias_table_x + bias_table_width/2;
    segments[16].y2 = bias_table_y + 12;
    
    XDrawString(dpy, w, gc, bias_table_x + (bias_table_width*.25) - (4*6/2), bias_table_y + 11, "Bias", 4);
    XDrawString(dpy, w, gc, bias_table_x + (bias_table_width*.75) - (7*6/2), bias_table_y + 11, "Release", 7);
    
    segments[17].x1 = bias_table_x + bias_table_width/6.0;
    segments[17].y1 = bias_table_y + 12;
    segments[17].x2 = bias_table_x + bias_table_width/6.0;
    segments[17].y2 = bias_table_y + 12 + bias_table_height;
    
    segments[18].x1 = bias_table_x + 2*bias_table_width/6.0;
    segments[18].y1 = bias_table_y + 12;
    segments[18].x2 = bias_table_x + 2*bias_table_width/6.0;
    segments[18].y2 = bias_table_y + 12 + bias_table_height;

    segments[19].x1 = bias_table_x + 3*bias_table_width/6.0;
    segments[19].y1 = bias_table_y + 12;
    segments[19].x2 = bias_table_x + 3*bias_table_width/6.0;
    segments[19].y2 = bias_table_y + 12 + bias_table_height;

    segments[20].x1 = bias_table_x + bias_table_width*(.5+(1/4.0));
    segments[20].y1 = bias_table_y + 12;
    segments[20].x2 = bias_table_x + bias_table_width*(.5+(1/4.0));
    segments[20].y2 = bias_table_y + 12 + bias_table_height;

    
    XDrawString(dpy, w, gc, bias_table_x + (0.5*bias_table_width/6.0) - (4*6/2), bias_table_y + 12+11, "Mass", 4);
    XDrawString(dpy, w, gc, bias_table_x + (1.5*bias_table_width/6.0) - ((7*6)/2), bias_table_y + 12+11, "Damping", 7);
    XDrawString(dpy, w, gc, bias_table_x + (2.5*bias_table_width/6.0) - ((9*6)/2), bias_table_y + 12+11, "Stiffness", 9);
    
    XDrawString(dpy, w, gc, bias_table_x + (bias_table_width*(.5+(0.5/4.0))) - ((7*6)/2), bias_table_y + 12+11, "Damping", 7);
    XDrawString(dpy, w, gc, bias_table_x + (bias_table_width*(.5+(1.5/4.0))) - ((9*6)/2), bias_table_y + 12+11, "Stiffness", 9);

    draw_float(dpy, w, gc, bias_table_x + (0.5*bias_table_width/6.0) - (4*6/2), bias_table_y + 24+11, scanner->mass_bias, 3);
    draw_float(dpy, w, gc, bias_table_x + (1.5*bias_table_width/6.0) - ((7*6)/2), bias_table_y + 24+11, scanner->damping_bias, 3);
    draw_float(dpy, w, gc, bias_table_x + (2.5*bias_table_width/6.0) - ((9*6)/2), bias_table_y + 24+11, scanner->stiffness_bias, 3);
    
    draw_float(dpy, w, gc, bias_table_x + (bias_table_width*(.5+(0.5/4.0))) - ((7*6)/2), bias_table_y + 24+11, scanner->release_damping, 3);
    draw_float(dpy, w, gc, bias_table_x + (bias_table_width*(.5+(1.5/4.0))) - ((9*6)/2), bias_table_y + 24+11, scanner->release_stiffness, 3);

    
    int other_table_x = 740;
    int other_table_y = pos_table_y;
    int other_table_width = 360;
    int other_table_height = 26;
    
    segments[21].x1 = other_table_x;
    segments[21].y1 = other_table_y;
    segments[21].x2 = other_table_x + other_table_width;
    segments[21].y2 = other_table_y;

    segments[22].x1 = other_table_x + other_table_width;
    segments[22].y1 = other_table_y;
    segments[22].x2 = other_table_x + other_table_width;
    segments[22].y2 = other_table_y + other_table_height;
    
    segments[23].x1 = other_table_x;
    segments[23].y1 = other_table_y;
    segments[23].x2 = other_table_x;
    segments[23].y2 = other_table_y + other_table_height;
    
    segments[24].x1 = other_table_x;
    segments[24].y1 = other_table_y + other_table_height;
    segments[24].x2 = other_table_x + other_table_width;
    segments[24].y2 = other_table_y + other_table_height;

    segments[25].x1 = other_table_x;
    segments[25].y1 = other_table_y + other_table_height/2;
    segments[25].x2 = other_table_x + other_table_width;
    segments[25].y2 = other_table_y + other_table_height/2;
    
    segments[26].x1 = other_table_x + other_table_width/3;
    segments[26].y1 = other_table_y;
    segments[26].x2 = other_table_x + other_table_width/3;
    segments[26].y2 = other_table_y + other_table_height;

    segments[27].x1 = other_table_x + 2*other_table_width/3;
    segments[27].y1 = other_table_y;
    segments[27].x2 = other_table_x + 2*other_table_width/3;
    segments[27].y2 = other_table_y + other_table_height;

    XDrawString(dpy, w, gc, other_table_x + (other_table_width*(0.5/3.0)) - ((6*6)/2), other_table_y + 11, "Volume", 6);
    XDrawString(dpy, w, gc, other_table_x + (other_table_width*(1.5/3.0)) - ((6*6)/2), other_table_y + 11, "Hammer", 6);
    XDrawString(dpy, w, gc, other_table_x + (other_table_width*(2.5/3.0)) - ((19*6)/2), other_table_y + 11, "Restoring Stiffness", 19);
    
    draw_float(dpy, w, gc, other_table_x + (other_table_width*(0.5/3.0)) - ((6*6)/2), other_table_y + 24, scanner->m_volume, 3);
    draw_float(dpy, w, gc, other_table_x + (other_table_width*(1.5/3.0)) - ((6*6)/2), other_table_y + 24, scanner->hammer_num, 3);
    draw_float(dpy, w, gc, other_table_x + (other_table_width*(2.5/3.0)) - ((19*6)/2), other_table_y + 24, scanner->restoring_stiffness, 3);
    
    XDrawSegments(dpy, w, gc, segments, num_segments);
    
    int select = scanner->scan_method/10;
    switch(select){
    case 0:
        XDrawString(dpy, w, gc, 1120, other_table_y, "Scan Path Method: Z Pos", 23);
        break;
    case 1:
        XDrawString(dpy, w, gc, 1120, other_table_y, "Scan Path Method: Displacement Norm", 35);
        break;
    case 2:
        XDrawString(dpy, w, gc, 1120, other_table_y, "Scan Path Method: Displacement Sum", 34);
        break;
    }

    if(scanner->pad_mode){
        XDrawString(dpy, w, gc, 1120, other_table_y+10, "Pad Mode On", 11);
    }
    else{
        XDrawString(dpy, w, gc, 1120, other_table_y+10, "Pad Mode Off", 12);
    }
}


void* window_thread(void*){
  int menu_id = SCANNER_3D_MENU;
  int mono = 1;
  int node_sel = 0;
  
  int node_sel_x = 0; //Connectivity matrix
  int node_sel_y = 0;
  
  int flag = 0;
  int should_clear = 1;
  
  XSetBackground(dpy, gc, 0);
  while(is_window_open()){
    if(should_clear){
        XClearWindow(dpy, w);
    }
    switch(menu_id){
    case SCANNER_3D_MENU: //Main Menu Section=============================================
        draw_scanner(dpy, w, gc, mono);
        handle_scanner_menu(dpy, w, gc, menu_id, mono);
        flag = 1;
        should_clear = 1;
        break;
    case SCAN_PATH_MENU: //Scan Path Selection Menu=================================
        draw_scan_path_menu(dpy, w, gc, node_sel);
        handle_scan_path_menu(dpy, w, gc, menu_id, node_sel);
        break;
    case NODE_MASS_MENU: //Node Mass Menu=================================
        draw_node_mass_menu(dpy, w, gc, node_sel);
        handle_node_mass_menu(dpy, w, gc, menu_id, node_sel);
        break;
    case NODE_DAMPING_MENU: //Node Mass Menu=================================
        draw_node_damping_menu(dpy, w, gc, node_sel);
        handle_node_damping_menu(dpy, w, gc, menu_id, node_sel);
        break;
    case CONNECTIVITY_MENU: //Connection Matrix Menu=================================
        if(flag){
            node_sel_x = 0;
            node_sel_y = 0;
            flag = 0;
        }
        draw_connectivity_menu(dpy, w, gc, node_sel_x, node_sel_y);
        handle_connectivity_menu(dpy, w, gc, menu_id, node_sel_x, node_sel_y);
        break;
    case SCANNER_2D_MENU:
        draw_scanner_2d_menu(dpy, w, gc);
        handle_scanner_2d_menu(dpy, w, gc, menu_id);
        break;
    case CONSTRAINT_MENU:
        draw_node_constraint_menu(dpy, w, gc, node_sel);
        handle_node_constraint_menu(dpy, w, gc, menu_id, node_sel);
        break;
    case EQ_POS_MENU:
        if(flag){
            node_sel_x = 0;
            node_sel_y = 0;
            flag = 0;
        }
        draw_eq_pos_menu(dpy, w, gc, node_sel_x, node_sel_y);
        handle_eq_pos_menu(dpy, w, gc, node_sel_x, node_sel_y, menu_id);
        break;
    case INFO_MENU:
        draw_info_menu(dpy, w, gc);
        handle_info_menu(dpy, w, gc, menu_id);
        break;
    case ADSR_MENU:
        if(flag){
            node_sel_x = 1;
            flag = 0;
        }
        draw_adsr_menu(dpy, w, gc, node_sel_x);
        handle_adsr_menu(dpy, w, gc, menu_id, node_sel_x);
        break;
    case REVERB_MENU:
        scanner->reverb.draw_reverb(dpy, w, gc);
        handle_reverb_menu(dpy, w, gc, menu_id);
        break;
    case VISUAL_MENU:
        if(flag){
            flag = 0;
            should_clear = 0;
        }
        draw_visual_menu(dpy, w, gc, mono);
        handle_visual_menu(dpy, w, gc, menu_id);
        break;
    default: //Never get here. Pls.
        break;
    }

    if(menu_id == REVERB_MENU){
        scanner->reverb.update_params();
    }
    else{
        scanner->update_params();    
    }
    
    origin = scanner->origin;
    scanner_rot = scanner->scanner_rot;
    
    XFlush(dpy);
    usleep(10000); //update 100 times a second.
  }
  return 0;
}


void init_window(SuperScanner *s){
    scanner = s;
  
    dpy = XOpenDisplay(0);
    w = XCreateSimpleWindow(dpy, DefaultRootWindow(dpy), 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, 0, 0, 0);

    Atom wm_state   = XInternAtom (dpy, "_NET_WM_STATE", true );
    Atom wm_fullscreen = XInternAtom (dpy, "_NET_WM_STATE_FULLSCREEN", true );
    XChangeProperty(dpy, w, wm_state, XA_ATOM, 32, PropModeReplace, (unsigned char *)&wm_fullscreen, 1);
    
    XSelectInput(dpy, w, StructureNotifyMask | ExposureMask | KeyPressMask | ButtonPressMask);
    XClearWindow(dpy, w);
    XMapWindow(dpy, w);
    gc = XCreateGC(dpy, w, 0, 0);
    
    XEvent e;
    do{
        XNextEvent(dpy, &e);        
    } while(e.type != MapNotify);

    pthread_create(&w_thread, NULL, &window_thread, NULL);    
}



void del_window(){
    pthread_join(w_thread, 0);
    XDestroyWindow(dpy, w);
    XCloseDisplay(dpy);
    return;
}



Matrix3f get_rotation(float x, float y, float z){
    Matrix3f rotx; rotx << 1,0,0, 0,cosf(x),-sinf(x), 0,sinf(x),cosf(x);
    Matrix3f roty; roty << cosf(y),0,sinf(y), 0,1,0, -sinf(y),0,cosf(y);
    Matrix3f rotz; rotz << cosf(z),-sinf(z),0, sinf(z),cosf(z),0, 0,0,1;

    return rotx*roty*rotz;
}



void draw_node_labels(Vector3f *node_pos, int num_nodes){
  ProjectedPoint pp; 
  MonoPixel mp;
  XPoint xpoints[num_nodes];
  for(int i = 0; i < num_nodes; i++){
    pp = project_mono_point(node_pos[i], origin);
    mp = projection_to_mono_pixel(pp);
    
    xpoints[i].x = mp.x;
    xpoints[i].y = mp.y;
  }

  XSetForeground(dpy, gc, 0xFF);
  XDrawPoints(dpy, w, gc, xpoints, num_nodes, CoordModeOrigin);

  int num_len;
  char num[20]; //limited to 3 digit numbers. Plz dont make me do more.
  memset(num, 0, 4);

  for(int i = 0; i < num_nodes; i++){
    sprintf(num, "%d", i);
    //I Don't care if there is a better way to do this. Its good enough.
    num_len = strlen(num);
    XDrawString(dpy, w, gc, xpoints[i].x, xpoints[i].y, num, num_len);
  }
}


ProjectedPoint project_mono_point(Vector3f point_v, Vector3f view_v){
  ProjectedPoint pp;
  
  pp.theta = (point_v[1] - view_v[1])/( point_v[0] - view_v[0]); //atan2f(point_v[1] - view_v[1], point_v[0] - view_v[0]);
  pp.phi = (point_v[2] - view_v[2])/( point_v[0] - view_v[0]); //atan2f(point_v[2] - view_v[2], point_v[0] - view_v[0]);

  return pp;
}


MonoPixel projection_to_mono_pixel(ProjectedPoint pp){
  MonoPixel mp;
  float theta_max = 2;
  float phi_max = 2;

  float scale_x = (SCREEN_WIDTH/2)/theta_max;
  float scale_y = (SCREEN_HEIGHT/2)/phi_max;

  if(abs(pp.theta) > (theta_max + EPS) || abs(pp.phi) > (phi_max + EPS)){
    mp.x = -1;
    mp.y = -1;
  }
  else{
    mp.x = floor(pp.theta*scale_x + SCREEN_WIDTH/2);
    mp.y = floor(-pp.phi*scale_y + SCREEN_HEIGHT/2);
  }
  return mp;
}


void draw_mono_lines(Vector3f *start_point, Vector3f *end_point, int len){
  ProjectedPoint pp_start;
  ProjectedPoint pp_end;

  MonoPixel mp_start;
  MonoPixel mp_end;
  XSegment segments[len];
  
  for(int i = 0; i < len; i++){
    pp_start = project_mono_point(start_point[i], origin);
    pp_end = project_mono_point(end_point[i], origin);
    
    mp_start = projection_to_mono_pixel(pp_start);
    mp_end = projection_to_mono_pixel(pp_end);

    if((mp_start.x != -1) && (mp_end.x != -1) && ((start_point[i][0] - origin[0]) > 0) && ((end_point[i][0] - origin[0]) > 0)){
        segments[i].x1 = mp_start.x;
        segments[i].y1 = mp_start.y;
        segments[i].x2 = mp_end.x;
        segments[i].y2 = mp_end.y;
    }
    else{
        segments[i].x1 = -1;
        segments[i].y1 = -1;
        segments[i].x2 = -1;
        segments[i].y2 = -1;
    }
  }
  
  XDrawSegments(dpy, w, gc, segments, len);
}



void draw_mono_points(Vector3f *point, int len){
  ProjectedPoint pp; 
  MonoPixel mp;
  XPoint xpoints[len];
  for(int i = 0; i < len; i++){
    pp = project_mono_point(point[i], origin);
    mp = projection_to_mono_pixel(pp);
    if((point[i][0] - origin[0]) > 0){
        xpoints[i].x = mp.x;
        xpoints[i].y = mp.y;        
    }
    else{
        xpoints[i].x = -1;
        xpoints[i].y = -1;
    }
  }

  XDrawPoints(dpy, w, gc, xpoints, len, CoordModeOrigin);  
}




ProjectedStereoPoints project_stereo_point(Vector3f point_v, Vector3f view_v){
    Point3D view;
    Point3D point;
    ProjectedStereoPoints pp;

    view.x = view_v[0];
    view.y = view_v[1];
    view.z = view_v[2];
    
    point.x = point_v[0];
    point.y = point_v[1];
    point.z = point_v[2];
    
    pp.theta_l = (point.y - view.y)/(point.x - view.x);//atan2f(point.y - view.y, point.x - view.x);
    pp.theta_r = (point.y - view.y - STEREO_DISPARITY)/(point.x - view.x); //atan2f(point.y - view.y - STEREO_DISPARITY, point.x - view.x);
    pp.phi_l = (point.z - view.z)/(point.x - view.x); //atan2f(point.z - view.z, point.x - view.x);
    pp.phi_r = (point.z - view.z)/(point.x - view.x); //atan2f(point.z - view.z, point.x - view.x);

    return pp;
}

StereoPixels projection_to_stereo_pixels(ProjectedStereoPoints pp){
    StereoPixels sp;
    float theta_max = 2;
    float phi_max = 2;
    float scale_x = (SCREEN_WIDTH/4)/theta_max;
    float scale_y = (SCREEN_HEIGHT/2)/phi_max;

//    printf("angle %f %f\n", pp.theta_l, pp.phi_l);
    if(abs(pp.theta_l) > (theta_max + EPS) || abs(pp.phi_l) > (phi_max + EPS)){
        sp.xl = -1;
        sp.yl = -1;
    }
    else{
        sp.xl = floor(pp.theta_l*scale_x + SCREEN_WIDTH/4);
        sp.yl = floor(-pp.phi_l*scale_y + SCREEN_HEIGHT/2);
    }
    
    if(abs(pp.theta_r) > (theta_max + EPS) || abs(pp.phi_r) > (phi_max + EPS)){
        sp.xr = -1;
        sp.yr = -1;
    }
    else{
        sp.xr = floor(pp.theta_r*scale_x + 3*SCREEN_WIDTH/4);
        sp.yr = floor(-pp.phi_r*scale_y + SCREEN_HEIGHT/2);
    }
    return sp;
}

void draw_stereo_lines(Vector3f *start_point, Vector3f *end_point, int len){
  ProjectedStereoPoints pp_start;
  ProjectedStereoPoints pp_end;

  StereoPixels sp_start;
  StereoPixels sp_end;
  XSegment segments[2*len];
  
  for(int i = 0; i < len; i++){
    pp_start = project_stereo_point(start_point[i], origin);
    pp_end = project_stereo_point(end_point[i], origin);
    sp_start = projection_to_stereo_pixels(pp_start);
    sp_end = projection_to_stereo_pixels(pp_end);

    if((sp_start.xl != -1) && (sp_end.xl != -1) && (sp_start.xr != -1) && (sp_end.xr != -1) &&
       ((start_point[i][0] - origin[0]) > 0) && ((end_point[i][0] - origin[0]) > 0)){
        segments[2*i].x1 = sp_start.xl;
        segments[2*i].y1 = sp_start.yl;
        segments[2*i].x2 = sp_end.xl;
        segments[2*i].y2 = sp_end.yl;
        
        segments[(2*i)+1].x1 = sp_start.xr;
        segments[(2*i)+1].y1 = sp_start.yr;
        segments[(2*i)+1].x2 = sp_end.xr;
        segments[(2*i)+1].y2 = sp_end.yr;
    }
    else{
        segments[2*i].x1 = -1;
        segments[2*i].y1 = -1;
        segments[2*i].x2 = -1;
        segments[2*i].y2 = -1;
        
        segments[(2*i)+1].x1 = -1;
        segments[(2*i)+1].y1 = -1;
        segments[(2*i)+1].x2 = -1;
        segments[(2*i)+1].y2 = -1;
    }
    //    printf("Segment %d %d %d %d\n", segments[2*i].x1, segments[2*i].y1, segments[2*i].x2, segments[2*i].y2); //LEFTOFF
  }
  
  XDrawSegments(dpy, w, gc, segments, 2*len);
}

void draw_stereo_points(Vector3f *points, int len){
  ProjectedStereoPoints pp; 
  StereoPixels sp;
  XPoint xpoints[2*len];
  for(int i = 0; i < len; i++){
    pp = project_stereo_point(points[i], origin);
    sp = projection_to_stereo_pixels(pp);

    if((points[i][0] - origin[0]) > 0){
        xpoints[(2*i)].x = sp.xl;
        xpoints[(2*i)].y = sp.yl;
        xpoints[(2*i)+1].x = sp.xr;
        xpoints[(2*i)+1].y = sp.yr;
    }
  }

  XDrawPoints(dpy, w, gc, xpoints, 2*len, CoordModeOrigin);
}

void draw_stereo_line(Vector3f start_point, Vector3f end_point){
    //should be a line sloping to the top right.

    ProjectedStereoPoints pp_start = project_stereo_point(start_point, origin);
    ProjectedStereoPoints pp_end = project_stereo_point(end_point, origin);
    
    StereoPixels sp_start = projection_to_stereo_pixels(pp_start);
    StereoPixels sp_end = projection_to_stereo_pixels(pp_end);

    if(((start_point[0] - origin[0]) > 0) && ((end_point[0] - origin[0]) > 0)){
        if(!(sp_start.xl == -1 || sp_end.xl == -1))
            XDrawLine(dpy, w, gc, sp_start.xl, sp_start.yl, sp_end.xl, sp_end.yl);
        if(!(sp_start.xr == -1 || sp_end.xr == -1))
            XDrawLine(dpy, w, gc, sp_start.xr, sp_start.yr, sp_end.xr, sp_end.yr);
    }
}

void draw_stereo_point(Vector3f point){
    ProjectedStereoPoints pp = project_stereo_point(point, origin);
    StereoPixels sp = projection_to_stereo_pixels(pp);
    if((point[0] - origin[0]) > 0){
        XDrawPoint(dpy, w, gc, sp.xl, sp.yl);
        XDrawPoint(dpy, w, gc, sp.xr, sp.yr);
    }
}

void draw_stereo_sphere(Sphere sphere){
    ProjectedStereoPoints pp = project_stereo_point(sphere.pos, origin);
    StereoPixels sp = projection_to_stereo_pixels(pp);
    Vector3f temp(sphere.pos);
    float radius_angle = (sphere.radius)/(temp.norm())*128; //atan2f(sphere.radius, temp.norm())*128;
    XFillArc(dpy, w, gc, sp.xl, sp.yl, radius_angle, radius_angle, 0, 360*64);
    XFillArc(dpy, w, gc, sp.xr, sp.yr, radius_angle, radius_angle, 0, 360*64);
}

int rainbow(int c){
  static int red = 0x00;
  static int green = 0x00;
  static int blue = 0xFF;
  static int state = 0;
  static int counter = 0;
  
  counter++;
  if(counter < c){
      int out = 0;
      out = (red << 16) | (blue << 8) | green;
      return out;
  }
  
  counter = 0;

  switch(state){
  case 0:
      green++;
      if(green == 0x8F) state = 1;
      break;
  case 1:
      blue--;
      if(blue == 0x00) state = 2;
      break;
  case 2:
      blue++;
      if(blue == 0x8F) state = 3;
      break;
  case 3:
      green--;
      if(green == 0x00) state = 0;
      break;
  }

  int out = 0;
  out = (red << 16) | (green << 8) | blue;
  return out;
  
}


void lorenz_ode(float *X, float *Xd){
    float sigma = 10;
    float beta = 8/3;
    float row = 22;
    
    Xd[0] = sigma*(X[1]-X[0]);
    Xd[1] = X[0]*(row-X[2])-X[1];
    Xd[2] = X[0]*X[1]-beta*X[2];
}
