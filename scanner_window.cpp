#include "scanner_window.h"
#include "super_scanner.h"
#include <algorithm>


Display *dpy;
Window w;
GC gc;
pthread_t w_thread;
static SuperScanner *scanner;

void breakerbreaker(){}

Vector3f origin = {-3,8,0};
float viewer_angle = -.1;


//draw all connections.
void draw_scanner(Display *dpy, Window w, GC gc, int mono){
  int actual_len = 0;
  int num_nodes = scanner->num_nodes;
  Vector3f start[1+(num_nodes*num_nodes/2)]; //throw in a random +1 to avoid off by one errors. in case num_nodes=7, 7*7/2 = 49/2 = 24.5 = 24 which would be one less than needed.
  Vector3f end[1+(num_nodes*num_nodes/2)];
  Vector3f node_pos_rot[scanner->num_nodes];
  
  Matrix3f rot = get_rotation(0, viewer_angle, 0);
  
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

  if(mono){
    draw_mono_lines(start, end, actual_len);
    draw_mono_points(node_pos_rot, num_nodes);
  }
  else{
    draw_stereo_lines(start, end, actual_len);
    draw_stereo_points(node_pos_rot, num_nodes);
  }
}


void handle_scanner_menu(Display *dpy, Window w, GC gc, int &menu_id, int &mono){
  XEvent e;
  char buf[2];

  if(XPending(dpy) > 0){
    XNextEvent(dpy, &e);
    if(e.type == KeyPress){
      XLookupString(&e.xkey, buf, 1, NULL, NULL);
      switch(buf[0]){
      case 'm':
	mono = !mono;
	break;
      case 's':
	scanner->sim_mutex = 1;
	menu_id = SCAN_PATH_MENU;
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
      }
    }
  }
}



void draw_scan_path_menu(Display *dpy, Window w, GC gc, int node_sel){
  //Draw 1/2 size mono scanner. Label Nodes.
  int actual_len = 0;
  int num_nodes = scanner->num_nodes;
  Vector3f start[1+(num_nodes*num_nodes/2)]; //throw in a random +1 to avoid off by one errors. in case num_nodes=7, 7*7/2 = 49/2 = 24.5 = 24 which would be one less than needed.
  Vector3f end[1+(num_nodes*num_nodes/2)];
  Vector3f node_pos_rot[scanner->num_nodes];
  
  Matrix3f rot = get_rotation(0, viewer_angle, 0);
  
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
  
  draw_mono_lines(start, end, actual_len);
  draw_node_labels(node_pos_rot, num_nodes);
  
  
  //Draw Scan Path
  int num_segs = 4 + scanner->num_nodes - 1;
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
  
  
  float interval = (SCREEN_WIDTH-40.0)/(scanner->num_nodes-1);
  for(int i = 1; i < scanner->num_nodes-1; i++){
    segments[4+i].x1 = 20 + (interval*i);
    segments[4+i].y1 = SCREEN_HEIGHT - 20;
    segments[4+i].x2 = 20 + (interval*i);
    segments[4+i].y2 = SCREEN_HEIGHT - 40;
  }

  XSegment sel_segments[4];
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
  sel_segments[2].x2 = 20 + (interval*(node_sel+1));
  sel_segments[2].y2 = SCREEN_HEIGHT - 40;
  
  
  XDrawSegments(dpy, w, gc, segments, num_segs);
  
  XSetForeground(dpy, gc, 0xFF0000);  
  XDrawSegments(dpy, w, gc, sel_segments, 4);
}


void handle_scan_path_menu(Display *dpy, Window w, GC gc, int &menu_id, int &node_sel){
  XEvent e;
  char buf[2];

  if(XPending(dpy) > 0){
    XNextEvent(dpy, &e);
    if(e.type == KeyPress){
      XLookupString(&e.xkey, buf, 1, NULL, NULL);
      switch(buf[0]){
      case 'x':
	scanner->sim_mutex = 0;
	menu_id = SCANNER_3D_MENU;
	break;
      }
      switch(e.xkey.keycode){
      case XK_Left:
	node_sel = std::min(0, node_sel-1); //I like this function.
	break;
      case XK_Right:
	node_sel = std::max(scanner->num_nodes-1, node_sel+1); //me gusta
	break;
      }
    }
  }
}


void* window_thread(void*){
  int menu_id = SCANNER_3D_MENU;
  int mono = 1;
  int node_sel = 0;
  
  XSetBackground(dpy, gc, 0);
  while(is_window_open()){
    XClearWindow(dpy, w);
    switch(menu_id){
    case SCANNER_3D_MENU: //Main Menu Section=============================================
      draw_scanner(dpy, w, gc, mono);
      handle_scanner_menu(dpy, w, gc, menu_id, mono);      

      break;
      
    case SCAN_PATH_MENU: //Scan Path Selection Menu=================================
      draw_scan_path_menu(dpy, w, gc, node_sel);
      handle_scan_path_menu(dpy, w, gc, menu_id, node_sel);
      
      break;
     
    default: //Never get here. Pls.
      break;
    }
    scanner->update_params();
    
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
    
    XSelectInput(dpy, w, StructureNotifyMask | ExposureMask | KeyPressMask);
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
  char num[4]; //limited to 3 digit numbers. Plz dont make me do more.
  memset(num, 0, 4);

  for(int i = 0; i < num_nodes; i++){
    sprintf(num, "%d", i);
    //I Don't care if there is a better way to do this. Its good enough.
    if(i < 10)
      num_len = 1;
    else if(i < 100)
      num_len = 2;
    else if(i < 1000)
      num_len = 3;
    
    XDrawString(dpy, w, gc, xpoints[i].x, xpoints[i].y, num, num_len);
  }
}


ProjectedPoint project_mono_point(Vector3f point_v, Vector3f view_v){
  ProjectedPoint pp;
  
  pp.theta = atan2f(point_v[1] - view_v[1], point_v[0] - view_v[0]);
  pp.phi = atan2f(point_v[2] - view_v[2], point_v[0] - view_v[0]);

  return pp;
}


MonoPixel projection_to_mono_pixel(ProjectedPoint pp){
  MonoPixel mp;
  float theta_max = M_PI*.7;
  float phi_max = M_PI*.7;

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
    
    segments[i].x1 = mp_start.x;
    segments[i].y1 = mp_start.y;
    segments[i].x2 = mp_end.x;
    segments[i].y2 = mp_end.y;
  }
  
  XSetForeground(dpy, gc, 0xFF0000);
  XDrawSegments(dpy, w, gc, segments, len);
}



void draw_mono_points(Vector3f *point, int len){
  ProjectedPoint pp; 
  MonoPixel mp;
  XPoint xpoints[len];
  for(int i = 0; i < len; i++){
    pp = project_mono_point(point[i], origin);
    mp = projection_to_mono_pixel(pp);
    xpoints[i].x = mp.x;
    xpoints[i].y = mp.y;
  }

  XSetForeground(dpy, gc, 0xFF);
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
    
    pp.theta_l = atan2f(point.y - view.y, point.x - view.x);
    pp.theta_r = atan2f(point.y - view.y - STEREO_DISPARITY, point.x - view.x);
    pp.phi_l = atan2f(point.z - view.z, point.x - view.x);
    pp.phi_r = atan2f(point.z - view.z, point.x - view.x);

    return pp;
}

StereoPixels projection_to_stereo_pixels(ProjectedStereoPoints pp){
    StereoPixels sp;
    float theta_max = M_PI*.7;
    float phi_max = M_PI*.7;
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
    
    segments[2*i].x1 = sp_start.xl;
    segments[2*i].y1 = sp_start.yl;
    segments[2*i].x2 = sp_end.xl;
    segments[2*i].y2 = sp_end.yl;

    segments[(2*i)+1].x1 = sp_start.xr;
    segments[(2*i)+1].y1 = sp_start.yr;
    segments[(2*i)+1].x2 = sp_end.xr;
    segments[(2*i)+1].y2 = sp_end.yr;

    //    printf("Segment %d %d %d %d\n", segments[2*i].x1, segments[2*i].y1, segments[2*i].x2, segments[2*i].y2); //LEFTOFF
  }
  
  XSetForeground(dpy, gc, 0xFF0000);
  XDrawSegments(dpy, w, gc, segments, 2*len);
}

void draw_stereo_points(Vector3f *points, int len){
  ProjectedStereoPoints pp; 
  StereoPixels sp;
  XPoint xpoints[2*len];
  for(int i = 0; i < len; i++){
    pp = project_stereo_point(points[i], origin);
    sp = projection_to_stereo_pixels(pp);
    xpoints[(2*i)].x = sp.xl;
    xpoints[(2*i)].y = sp.yl;
    xpoints[(2*i)+1].x = sp.xr;
    xpoints[(2*i)+1].y = sp.yr;
  }

  XSetForeground(dpy, gc, 0xFF);
  XDrawPoints(dpy, w, gc, xpoints, 2*len, CoordModeOrigin);
}

void draw_stereo_line(Vector3f start_point, Vector3f end_point){
    //should be a line sloping to the top right.

    ProjectedStereoPoints pp_start = project_stereo_point(start_point, origin);
    ProjectedStereoPoints pp_end = project_stereo_point(end_point, origin);
    
    StereoPixels sp_start = projection_to_stereo_pixels(pp_start);
    StereoPixels sp_end = projection_to_stereo_pixels(pp_end);

    if(!(sp_start.xl == -1 || sp_end.xl == -1))
        XDrawLine(dpy, w, gc, sp_start.xl, sp_start.yl, sp_end.xl, sp_end.yl);
    if(!(sp_start.xr == -1 || sp_end.xr == -1))
        XDrawLine(dpy, w, gc, sp_start.xr, sp_start.yr, sp_end.xr, sp_end.yr);
}

void draw_stereo_point(Vector3f point){
    ProjectedStereoPoints pp = project_stereo_point(point, origin);
    StereoPixels sp = projection_to_stereo_pixels(pp);
    XDrawPoint(dpy, w, gc, sp.xl, sp.yl);
    XDrawPoint(dpy, w, gc, sp.xr, sp.yr);
}

void draw_stereo_sphere(Sphere sphere){
    ProjectedStereoPoints pp = project_stereo_point(sphere.pos, origin);
    StereoPixels sp = projection_to_stereo_pixels(pp);
    Vector3f temp(sphere.pos);
    float radius_angle = atan2f(sphere.radius, temp.norm())*128;
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
