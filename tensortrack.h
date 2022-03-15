/*
 * tracking camera using efficientdet_lite
 * Copyright (C) 2019-2022 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */



#ifndef TENSORTRACK_H
#define TENSORTRACK_H



// type of servo board
//#define USE_ATMEGA // atmega at 16Mhz with USB dongle
#define USE_PIC // 18f14k50 at 48Mhz with integrated USB

#define USE_SERVER



#ifdef USE_ATMEGA
// PWM limits
    #define MIN_PWM 1000 
    #define MAX_PWM 32700
// PWM allowed beyond starting point for tracking
    #define TILT_MAG 1500
    #define PAN_MAG 5000
// starting values
    #define PAN0 18676
    #define TILT0 21076
#endif

#ifdef USE_PIC
    #define MIN_PWM 733
    #define MAX_PWM 24000 
    #define TILT_MAG 1100
    #define PAN_MAG 3700
    #define PAN0 14007
    #define TILT0 15807
#endif

#define TEXTLEN 1024

#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define TO_MS(x) ((x).tv_sec * 1000 + (x).tv_usec / 1000)

// different parameters for different lenses
#define LENS_15 0
#define LENS_28 1
#define LENS_50 2
#define TOTAL_LENSES 3

#define STARTUP 0
#define CONFIGURING 1
#define TRACKING 2

// raw PWM values
extern float pan;
extern float tilt;
extern float start_pan;
extern float start_tilt;
extern int pan_sign;
extern int tilt_sign;
extern int lens;
extern int landscape;
extern int current_operation;

// error bits
#define VIDEO_DEVICE_ERROR 1
#define VIDEO_BUFFER_ERROR 2
#define SERVO_ERROR 4
extern uint8_t error_flags;
extern pthread_mutex_t www_mutex;
extern int server_output;
// temporary in the encoded size
extern uint8_t *server_video;

typedef struct
{
// manual PWM step
    int pan_step;
    int tilt_step;
// PID gain converts a percent into a PWM step
    float x_gain;
    float y_gain;
// Limit of PWM changes
    int max_tilt_change;
    int max_pan_change;
// deadband in percent
    float deadband;
// top_y in percent
    float top_y;
// height of box in percent to track center instead of head position
    int min_h;
} lens_t;

extern lens_t lenses[TOTAL_LENSES];

// send preview video to the GUI or server
void draw_video(unsigned char *src, 
    int dst_x,
    int dst_y,
    int dst_w,
    int dst_h,
    int src_w,
    int src_h,
    int src_rowspan);
void init_gui();
void save_defaults();
void write_servos(int use_pwm_limits);
void stop_servos();
void send_error();
void init_server();
void send_vijeo_fifo(uint8_t *data, int bytes);



#endif


