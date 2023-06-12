/*
 * Tracking camera using body_25 & facenet
 *
 * Copyright (C) 2023 Adam Williams <broadcast at earthling dot net>
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

#ifndef TRUCKCAM_TRT_H
#define TRUCKCAM_TRT_H


#include <stdint.h>

extern int current_operation;
extern uint8_t error_flags;


// dimensions injested from the camera must match the decoded JPEG size
#define RAW_W 1280
#define RAW_H 720

// face tracker dimensions
// must regenerate the .engine files if this changes
#define SCAN_W 1280
#define SCAN_H 720

// preview on server
#define SERVER_W 640
#define SERVER_H 360

#define HEADER_SIZE 8
#define MAX_JPEG 0x100000

#define TEXTLEN 1024

#define IDLE 0
#define TRACKING 1

#define VIDEO_DEVICE_ERROR 1
#define VIDEO_BUFFER_ERROR 2
#define SERVO_ERROR 4

#define KNOWN_PATH "known"
#define MAX_FACES 5

#define INPUT_IMAGES 2
extern uint8_t **input_rows[INPUT_IMAGES];
extern int current_input2;

void send_error();
void init_server();
void send_vijeo(int current_input, int keypoint_size);


#endif




