/*
 * face tracking camera
 * Copyright (C) 2022 Adam Williams <broadcast at earthling dot net>
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



#ifndef TRUCKCAM_H
#define TRUCKCAM_H

#include <sys/time.h>

#define TEXTLEN 1024

// raspberry pi SPI
#define USE_PI

// video from HDMI
//#define RAW_W 1920
//#define RAW_H 1080
// video from generalplus
#define RAW_W 1280
#define RAW_H 720
// cropped section to scan
// #define CROP_X 240
// #define CROP_Y 0
// #define CROP_W 1440
// #define CROP_H 720
#define CROP_X 0
#define CROP_Y 0
#define CROP_W 1280
#define CROP_H 720
// scaled image to scan
// smaller is faster
#define SCALED_W 640
#define SCALED_H 360
// max frame rate + 1 to limit the preview bandwidth
// minimum frame rate is required to prevent wifi stuttering
#define FPS 11

// use face size instead of recognition model
#define USE_SIZE


// use ffmpeg for encoding preview
//#define USE_FFMPEG

#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define TO_MS(x) ((x).tv_sec * 1000 + (x).tv_usec / 1000)

// points to either ffmpeg input or the web socket
extern int server_output;

extern int current_operation;
#define IDLE 0
#define TRACKING 1
extern int face_position;
#define FACE_LEFT 0
#define FACE_CENTER 1
#define FACE_RIGHT 2

extern int deadband;
extern int speed;
extern int xy_radius;
extern int size_radius;
extern int color_radius;
extern struct timespec settings_time;

extern uint8_t error_flags;
#define VIDEO_DEVICE_ERROR 1
#define VIDEO_BUFFER_ERROR 2

// packet type
#define VIJEO 0x00
#define STATUS 0x01

void send_error();
void init_server();
void send_vijeo_fifo(uint8_t *data, int bytes);
void save_defaults();
void dump_settings();




#endif









