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

// this runs on a raspberry pi 4B


// To build it:
// 
// ./make tensortrack
// 
// To run it:
// 
// LD_LIBRARY_PATH=/usr/local/lib:. ./tensortrack


// the earlier trackers using LIDAR & difference keying
// are in the gimbal directory as motion.c cam_lidar.c cam.c
// the servo driver is in the servos/ directory

#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"
#include "tensorflow/lite/builtin_op_data.h"

#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>
#include <linux/videodev2.h>
#include <queue>
#include <vector>

extern "C"
{
#include <uuid/uuid.h>
}

#include "tensortrack.h"

// capture raw video for debugging or offline training
//#define CAPTURE

// Read uncompressed HDMI.  Slows framerate to 5fps
//#define RAW_HDMI





// Number of humans in the scene
#define MAX_HITS 2
// score required to get a hit
#define THRESHOLD 0.5



#define MODEL "efficientlion0.tflite.1000.300epoch"


// video from HDMI
#define RAW_W 1920
#define RAW_H 1080
// cropping in landscape mode
#define CROP_W 1080
#define CROP_H 1080
// preview in landscape mode
#define PREVIEW_W 640
#define PREVIEW_H 360
// cropping in portrait mode for inferrence
#define P_CROP_W 1080
#define P_CROP_H 1620
// max frame rate to limit the preview bandwidth
#define FPS 11
#define SCANNING0 0
#define SCANNING1 1
#define SCANNING2 2
#define LOCKED 4
int crop_state = SCANNING0;
int crop_counter = 0;
#define CROP_TIMEOUT 0

// current position of cropped section, derived from the crop state
int window_x = 0;
// previous position of cropped section
int prev_window_x = 0;

#define HDMI_BUFFERS 2
#define BUFSIZE2 0x400000
unsigned char *mmap_buffer[HDMI_BUFFERS];
// compressed frame from the hardware
unsigned char reader_buffer3[BUFSIZE2];
// size of the frame in reader_buffer3
int frame_size;
struct timespec fps_time1;
static pthread_mutex_t frame_lock;
static sem_t frame_ready_sema;
int hdmi_fd = -1;
int current_hdmi = -1;

// input size for the neural network
int input_w;
int input_h;

#define INIT_PROFILE \
    struct timespec profile_time1; \
    struct timespec profile_time2; \
    double delta; \
    clock_gettime(CLOCK_MONOTONIC, &profile_time1);


#define UPDATE_PROFILE \
    clock_gettime(CLOCK_MONOTONIC, &profile_time2); \
    delta =  \
        (double)((profile_time2.tv_sec * 1000 + profile_time2.tv_nsec / 1000000) - \
        (profile_time1.tv_sec * 1000 + profile_time1.tv_nsec / 1000000)) / 1000; \
    profile_time1 = profile_time2;

lens_t lenses[] = 
{
// 17mm
    { 
        150,  // pan_step
        150,  // tilt_step
        50,   // x_gain
        30,   // y_gain
        50,   // max_tilt_change
        50,   // max_pan_change
        5,    // deadband
        7,    // top_y.  If top is above this, tilt down.
        75,   // min_h.  If box is smaller than this, track center instead of top.
    },
};


// scale from pixels to percent of screen size
#define TO_PERCENT_Y(y) ((y) * 100 / height)
#define TO_PERCENT_X(x) ((x) * 100 / width)
// scale from percent of screen size to pixels
#define FROM_PERCENT_Y(y) ((y) * height / 100)
#define FROM_PERCENT_X(x) ((x) * width / 100)





// raw PWM values
float pan = PAN0;
float tilt = TILT0;
float start_pan = pan;
float start_tilt = tilt;
int pan_sign = 1;
int tilt_sign = 1;
int lens = LENS_15;
int landscape = 1;

static int servo_fd = -1;
static int frames = 0;
static FILE *ffmpeg_fd = 0;

int current_operation = STARTUP;
uint8_t error_flags = 0xff;
// show the alert for this amount of time
struct timespec settings_time = { 0 };

int init_serial(const char *path)
{
	struct termios term;
    int verbose = 0;

	if(verbose) printf("init_serial %d: opening %s\n", __LINE__, path);

// Initialize serial port
	int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		if(verbose) printf("init_serial %d: path=%s: %s\n", __LINE__, path, strerror(errno));
		return -1;
	}
	
	if (tcgetattr(fd, &term))
	{
		if(verbose) printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}


/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
	tcflush(fd, TCIOFLUSH);
	cfsetispeed(&term, B115200);
	cfsetospeed(&term, B115200);
//	term.c_iflag = IGNBRK;
	term.c_iflag = 0;
	term.c_oflag = 0;
	term.c_lflag = 0;
//	term.c_cflag &= ~(PARENB | PARODD | CRTSCTS | CSTOPB | CSIZE);
//	term.c_cflag |= CS8;
	term.c_cc[VTIME] = 1;
	term.c_cc[VMIN] = 1;
/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
	if(tcsetattr(fd, TCSANOW, &term))
	{
		if(verbose) printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

	printf("init_serial %d: opened %s\n", __LINE__, path);
	return fd;
}


void* servo_reader(void *ptr)
{
//    printf("servo_reader %d\n", __LINE__);
    int servos = 0;

    while(1)
    {
// open the device
        if(servo_fd < 0)
        {
#ifdef USE_ATMEGA
	        servo_fd = init_serial("/dev/ttyUSB0");
	        if(servo_fd < 0) servo_fd = init_serial("/dev/ttyUSB1");
	        if(servo_fd < 0) servo_fd = init_serial("/dev/ttyUSB2");
#endif

#ifdef USE_PIC
	        servo_fd = init_serial("/dev/ttyACM0");
	        if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM1");
	        if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM2");
#endif
        }

        if(servo_fd >= 0)
        {
            if((error_flags & SERVO_ERROR))
            {
                printf("servo_reader %d: servos found\n", __LINE__);
            }
            error_flags &= ~SERVO_ERROR;
            send_error();

            uint8_t buffer;
            while(1)
            {
                int bytes_read = read(servo_fd, &buffer, 1);
                if(bytes_read <= 0)
                {
                    printf("servo_reader %d: servos unplugged\n", __LINE__);
                    close(servo_fd);
                    servo_fd = -1;
                    break;
                }
            }
            //printf("%c", buffer);
            //fflush(stdout);
        }
        else
        {
            if(!(error_flags & SERVO_ERROR))
            {
                printf("servo_reader %d: servos not found\n", __LINE__);
            }
            error_flags |= SERVO_ERROR;
            send_error();
            sleep(1);
        }
    }
}

int init_servos()
{
    pthread_t x;
	pthread_create(&x, 
		0, 
		servo_reader, 
		0);

    return 0;
}

void write_servos(int use_pwm_limits)
{
	if(servo_fd >= 0)
	{
#define SYNC_CODE0 0xff
#define SYNC_CODE1 0x2d
#define SYNC_CODE2 0xd4
#define SYNC_CODE3 0xe5
#define BUFFER_SIZE 8

// limits are absolute PWM limits
        if(use_pwm_limits)
        {
            CLAMP(pan, MIN_PWM, MAX_PWM);
            CLAMP(tilt, MIN_PWM, MAX_PWM);
        }
        else
// limits are relative to the starting position
        {
            CLAMP(pan, start_pan - PAN_MAG, start_pan + PAN_MAG);
            CLAMP(tilt, start_tilt - TILT_MAG, start_tilt + TILT_MAG);
        }

        uint16_t pan_i = (uint16_t)pan;
        uint16_t tilt_i = (uint16_t)tilt;
		char buffer[BUFFER_SIZE];
        buffer[0] = SYNC_CODE0;
        buffer[1] = SYNC_CODE1;
        buffer[2] = SYNC_CODE2;
        buffer[3] = SYNC_CODE3;
        buffer[4] = pan_i;
        buffer[5] = pan_i >> 8;
        buffer[6] = tilt_i;
        buffer[7] = tilt_i >> 8;

//printf("write_servos %d %d %d\n", __LINE__, pan_i,  tilt_i);
		int temp = write(servo_fd, buffer, BUFFER_SIZE);
	}
}

void stop_servos()
{
	if(servo_fd >= 0)
	{
#define SYNC_CODE0 0xff
#define SYNC_CODE1 0x2d
#define SYNC_CODE2 0xd4
#define SYNC_CODE3 0xe5
#define BUFFER_SIZE 8
		char buffer[BUFFER_SIZE];
        buffer[0] = SYNC_CODE0;
        buffer[1] = SYNC_CODE1;
        buffer[2] = SYNC_CODE2;
        buffer[3] = SYNC_CODE3;
        buffer[4] = 0;
        buffer[5] = 0;
        buffer[6] = 0;
        buffer[7] = 0;

// write it a few times to defeat UART initialization glitches
		int temp = write(servo_fd, buffer, BUFFER_SIZE);
        temp = write(servo_fd, buffer, BUFFER_SIZE);
        temp = write(servo_fd, buffer, BUFFER_SIZE);
        temp = write(servo_fd, buffer, BUFFER_SIZE);
    }    
}



void load_defaults()
{
    char *home = getenv("HOME");
    char string[TEXTLEN];
    sprintf(string, "%s/.tracker.rc", home);
    FILE *fd = fopen(string, "r");
    
    if(!fd)
    {
        printf("load_defaults %d: Couldn't open %s for reading\n", 
            __LINE__, 
            string);
        return;
    }
    
    while(!feof(fd))
    {
        if(!fgets(string, TEXTLEN, fd)) break;
// get 1st non whitespace character
        char *key = string;
        while((*key == ' ' ||
            *key == '\t' ||
            *key == '\n') && 
            *key != 0)
        {
            key++;
        }

// comment or empty
        if(*key == '#' || *key == 0)
        {
            continue;
        }
        
// get start of value
        char *value = key;
        while(*value != ' ' && 
            *value != '\t' && 
            *value != '\n' && 
            *value != 0)
        {
            value++;
        }


        while((*value == ' ' ||
            *value == '\t' ||
            *value == '\n') && 
            *value != 0)
        {
            *value = 0;
            value++;
        }

        if(*value == 0)
        {
// no value given
            continue;
        }

// delete the newline
        char *end = value;
        while(*end != '\n' && 
            *end != 0)
        {
            end++;
        }
        
        if(*end == '\n')
        {
            *end = 0;
        }
        
        printf("load_defaults %d key='%s' value='%s'\n", __LINE__, key, value);
        if(!strcasecmp(key, "PAN"))
        {
            start_pan = pan = atof(value);
        }
        else
        if(!strcasecmp(key, "TILT"))
        {
            start_tilt = tilt = atof(value);
        }
        else
        if(!strcasecmp(key, "PAN_SIGN"))
        {
            pan_sign = atoi(value);
        }
        else
        if(!strcasecmp(key, "TILT_SIGN"))
        {
            tilt_sign = atoi(value);
        }        
        else
//         if(!strcasecmp(key, "LENS"))
//         {
//             lens = atoi(value);
//         }        
//         else
        if(!strcasecmp(key, "LANDSCAPE"))
        {
            landscape = atoi(value);
        }        
    }

    fclose(fd);
}

void save_defaults()
{
    char *home = getenv("HOME");
    char string[TEXTLEN];
    sprintf(string, "%s/.tracker.rc", home);
    FILE *fd = fopen(string, "w");

    if(!fd)
    {
        printf("save_defaults %d: Couldn't open %s for writing\n", 
            __LINE__, 
            string);
        return;
    }

    fprintf(fd, "PAN %d\n", (int)start_pan);
    fprintf(fd, "TILT %d\n", (int)start_tilt);
    fprintf(fd, "PAN_SIGN %d\n", pan_sign);
    fprintf(fd, "TILT_SIGN %d\n", tilt_sign);
//    fprintf(fd, "LENS %d\n", lens);
    fprintf(fd, "LANDSCAPE %d\n", landscape);

    fclose(fd);
}







// probe for the video device
int open_hdmi(int verbose)
{
    std::vector<char*> paths;
    char string[TEXTLEN];
// this causes a SIGCHLD
    FILE *fd = popen("ls /dev/video*", "r");
    while(!feof(fd))
    {
        char *result = fgets(string, TEXTLEN, fd);
        if(!result)
        {
            break;
        }
// strip the newlines
        while(strlen(result) > 1 &&
            result[strlen(result) - 1] == '\n')
        {
            result[strlen(result) - 1] = 0;
        }
        paths.push_back(strdup(result));
    }
    fclose(fd);

    int fd2 = -1;
    for(int i = 0; i < paths.size(); i++)
    {
        current_hdmi++;
        if(current_hdmi >= paths.size())
        {
            current_hdmi = 0;
        }

        fd2 = open(paths.at(current_hdmi), O_RDWR);
        if(fd2 < 0)
        {
            continue;
        }

// test the W & H to find the right camera
        struct v4l2_format v4l2_params;
        v4l2_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd2, VIDIOC_G_FMT, &v4l2_params);
// printf("open_hdmi %d %s %d %d\n", 
// __LINE__, 
// paths.at(i),
// v4l2_params.fmt.pix.width, 
// v4l2_params.fmt.pix.height);

        if(verbose)
        {
            printf("open_hdmi %d: %s format=%c%c%c%c w=%d h=%d\n",
                __LINE__,
                paths.at(current_hdmi),
                v4l2_params.fmt.pix.pixelformat & 0xff,
                (v4l2_params.fmt.pix.pixelformat >> 8) & 0xff,
                (v4l2_params.fmt.pix.pixelformat >> 16) & 0xff,
                (v4l2_params.fmt.pix.pixelformat >> 24) & 0xff,
                v4l2_params.fmt.pix.width,
                v4l2_params.fmt.pix.height);
        }

        if(v4l2_params.fmt.pix.width != RAW_W ||
            v4l2_params.fmt.pix.height != RAW_H)
        {
            close(fd2);
            fd2 = -1;
        }
        
        
// probe picture controls
        if(fd2 >= 0)
        {
		    for(int j = V4L2_CID_BASE; j < V4L2_CID_LASTP1; j++)
		    {
                struct v4l2_queryctrl arg;
			    bzero(&arg, sizeof(arg));
			    arg.id = j;
                if(!ioctl(fd2, VIDIOC_QUERYCTRL, &arg))
                {
                    printf("open_hdmi %d: id=%x min=%d max=%d step=%d default=%d %s\n", 
                        __LINE__, 
                        arg.id, 
                        arg.minimum,
                        arg.maximum,
                        arg.step,
                        arg.default_value,
                        arg.name);
                }
            }

        }



        if(fd2 >= 0)
        {
// codec
            struct v4l2_format v4l2_params;
            v4l2_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd2, VIDIOC_G_FMT, &v4l2_params);

// codec only works for HDMI
#ifdef RAW_HDMI
            v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#else
            v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
#endif
            if(ioctl(fd2, VIDIOC_S_FMT, &v4l2_params) < 0)
            {
//              printf("open_hdmi %d: VIDIOC_S_FMT failed\n",
//                  __LINE__);
            }
        }


        if(fd2 >= 0)
        {
            struct v4l2_requestbuffers requestbuffers;
            requestbuffers.count = HDMI_BUFFERS;
            requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            requestbuffers.memory = V4L2_MEMORY_MMAP;
            if(ioctl(fd2, VIDIOC_REQBUFS, &requestbuffers) < 0)
            {
                printf("open_hdmi %d: VIDIOC_REQBUFS failed\n",
                    __LINE__);
                close(fd2);
                fd2 = -1;
            }
            else
            {
                for(int j = 0; j < HDMI_BUFFERS; j++)
                {
                    struct v4l2_buffer buffer;
                    buffer.type = requestbuffers.type;
                    buffer.index = j;

                    if(ioctl(fd2, VIDIOC_QUERYBUF, &buffer) < 0)
				    {
					    printf("open_hdmi %d: VIDIOC_QUERYBUF failed\n",
                            __LINE__);
                        close(fd2);
                        fd2 = -1;
                        break;
				    }
                    else
                    {
                        mmap_buffer[j] = (unsigned char*)mmap(NULL,
					        buffer.length,
					        PROT_READ | PROT_WRITE,
					        MAP_SHARED,
					        fd2,
					        buffer.m.offset);
                        printf("open_hdmi %d: allocated buffer size=%d\n",
                            __LINE__,
                            buffer.length);
                        if(ioctl(fd2, VIDIOC_QBUF, &buffer) < 0)
                        {
                            printf("open_hdmi %d: VIDIOC_QBUF failed\n",
                                __LINE__);
                            close(fd2);
                            fd2 = -1;
                            break;
                        }
                    }
                }
            }

            if(fd2 >= 0)
            {
                int streamon_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	            if(ioctl(fd2, VIDIOC_STREAMON, &streamon_arg) < 0)
                {
		            printf("open_hdmi %d: VIDIOC_STREAMON failed\n",
                        __LINE__);
                    close(fd2);
                    fd2 = -1;
                }
                clock_gettime(CLOCK_MONOTONIC, &fps_time1);
            }




            if(fd2 >= 0)
            {
                if(verbose)
                {
                    printf("open_hdmi %d opened %s\n", __LINE__, paths.at(current_hdmi));
                }
                break;
            }


        }
    }
//printf("open_hdmi %d fd2=%d\n", __LINE__, fd2);

    while(paths.size() > 0)
    {
        free(paths.back());
        paths.pop_back();
    }

    return fd2;
}






// read frames from HDMI
void* hdmi_thread(void *ptr)
{
    struct timeval time1;
    gettimeofday(&time1, 0);

    int verbose = 1;
    int frame_count = 0;


    while(1)
    {
        if(hdmi_fd < 0)
        {
            hdmi_fd = open_hdmi(verbose);
            if(hdmi_fd < 0)
            {
                if(!(error_flags & VIDEO_DEVICE_ERROR))
                {
                    printf("hdmi_thread %d: no video device\n",
                        __LINE__);
                }
                error_flags |= VIDEO_DEVICE_ERROR;
                send_error();
                sleep(1);
            }
        }



        if(hdmi_fd >= 0)
        {
            struct v4l2_buffer buffer;
		    bzero(&buffer, sizeof(buffer));
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		    buffer.memory = V4L2_MEMORY_MMAP;

// select seems to protect against USB disconnects
            fd_set fds;
            struct timeval tv;
            FD_ZERO(&fds);
            FD_SET(hdmi_fd, &fds);
            tv.tv_sec = 2;
            tv.tv_usec = 0;
            int result = select(hdmi_fd + 1, &fds, NULL, NULL, &tv);
            if(result <= 0)
            {
                printf("hdmi_thread %d: V4L2 died\n", __LINE__);
                error_flags |= VIDEO_BUFFER_ERROR;
                send_error();
                close(hdmi_fd);
                hdmi_fd = -1;
                sleep(1);
            }
            else
            {

                if(ioctl(hdmi_fd, VIDIOC_DQBUF, &buffer) < 0)
                {
                    printf("hdmi_thread %d: VIDIOC_DQBUF failed\n",
                        __LINE__);
                    error_flags |= VIDEO_BUFFER_ERROR;
                    send_error();
                    close(hdmi_fd);
                    hdmi_fd = -1;
                    sleep(1);
                }
                else
                {
                    error_flags &= ~VIDEO_BUFFER_ERROR;
                    error_flags &= ~VIDEO_DEVICE_ERROR;
//                  printf("hdmi_thread %d\n", __LINE__);
                    send_error();
    //printf("hdmi_thread %d\n", __LINE__);
                    unsigned char *ptr = mmap_buffer[buffer.index];
    //                 printf("hdmi_thread %d: index=%d size=%d %02x %02x %02x %02x %02x %02x %02x %02x\n",
    //                     __LINE__,
    //                     buffer.index,
    //                     buffer.bytesused,
    //                     ptr[0],
    //                     ptr[1],
    //                     ptr[2],
    //                     ptr[3],
    //                     ptr[4],
    //                     ptr[5],
    //                     ptr[6],
    //                     ptr[7]);

                    if(
    // HDMI
                        (ptr[0] == 0xff && 
                        ptr[1] == 0xd8 && 
                        ptr[2] == 0xff && 
                        ptr[3] == 0xdb) ||
    // generalplus
                        (ptr[0] == 0xff && 
                        ptr[1] == 0xd8 && 
                        ptr[2] == 0xff && 
                        ptr[3] == 0xc0))
                    {
    // discard if it arrived too soon
                        struct timespec fps_time2;
                        clock_gettime(CLOCK_MONOTONIC, &fps_time2);
                        double delta = 
                            (double)((fps_time2.tv_sec * 1000 + fps_time2.tv_nsec / 1000000) -
                            (fps_time1.tv_sec * 1000 + fps_time1.tv_nsec / 1000000)) / 1000;
                        if(delta >= 1.0 / FPS)
                        {
    // send it
                            pthread_mutex_lock(&frame_lock);
                            memcpy(reader_buffer3, ptr, buffer.bytesused);
                            frame_size = buffer.bytesused;
                            pthread_mutex_unlock(&frame_lock);
                            sem_post(&frame_ready_sema);
                            fps_time1 = fps_time2;
                        }
                        else
                        {
    //                         printf("hdmi_thread %d: discarding\n",
    //                             __LINE__);
    // discard it
                        }
                    }
    //printf("hdmi_thread %d\n", __LINE__);

                    if(ioctl(hdmi_fd, VIDIOC_QBUF, &buffer) < 0)
                    {
                        printf("hdmi_thread %d: VIDIOC_QBUF failed\n",
                            __LINE__);
                    }




                    frame_count++;
                    struct timeval time2;
                    gettimeofday(&time2, 0);
                    int64_t diff = TO_MS(time2) - TO_MS(time1);
                    if(diff >= 1000)
                    {
    //                     printf("hdmi_thread %d FPS: %f\n",
    //                         __LINE__,
    //                         (double)frame_count * 1000 / diff);
                        frame_count = 0;
                        time1 = time2;
                    }

                }
            }
//printf("hdmi_thread %d\n", __LINE__);
        }
    }
}

void init_hdmi()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&frame_lock, &attr);
    sem_init(&frame_ready_sema, 0, 0);

    pthread_t x;
	pthread_create(&x, 
		0, 
		hdmi_thread, 
		0);
}


// some kind of bloated, academic Google version of bilinear interpolation
// hard coded for uint8
template <class T>
void resize(T* out, 
    uint8_t* in, 
    int image_height, 
    int image_width,
    int image_channels, 
    int wanted_height, 
    int wanted_width,
    int wanted_channels)
{
  int number_of_pixels = image_height * image_width * image_channels;
  std::unique_ptr<tflite::Interpreter> interpreter(new tflite::Interpreter);
  int base_index = 0;

// printf("resize %d %d %d %d %d\n", __LINE__, image_width, image_height, wanted_width, wanted_height);


  // two inputs: input and new_sizes
  interpreter->AddTensors(2, &base_index);
  // one output
  interpreter->AddTensors(1, &base_index);
  // set input and output tensors
  interpreter->SetInputs({0, 1});
  interpreter->SetOutputs({2});

  // set parameters of tensors
  TfLiteQuantizationParams quant;
  interpreter->SetTensorParametersReadWrite(
      0, kTfLiteFloat32, "input",
      {1, image_height, image_width, image_channels}, quant);
  interpreter->SetTensorParametersReadWrite(1, kTfLiteInt32, "new_size", {2},
                                            quant);
  interpreter->SetTensorParametersReadWrite(
      2, kTfLiteFloat32, "output",
      {1, wanted_height, wanted_width, wanted_channels}, quant);

  tflite::ops::builtin::BuiltinOpResolver resolver;
  const TfLiteRegistration* resize_op =
      resolver.FindOp(tflite::BuiltinOperator_RESIZE_BILINEAR, 1);
  auto* params = reinterpret_cast<TfLiteResizeBilinearParams*>(
      malloc(sizeof(TfLiteResizeBilinearParams)));
  params->align_corners = false;
  params->half_pixel_centers = false;
  interpreter->AddNodeWithParameters({0, 1}, {2}, nullptr, 0, params, resize_op,
                                     nullptr);

  interpreter->AllocateTensors();

  // fill input image
  // in[] are integers, cannot do memcpy() directly
  auto input = interpreter->typed_tensor<float>(0);
  for (int i = 0; i < number_of_pixels; i++) {
    input[i] = in[i];
  }

  // fill new_sizes
  interpreter->typed_tensor<int>(1)[0] = wanted_height;
  interpreter->typed_tensor<int>(1)[1] = wanted_width;

  interpreter->Invoke();

  auto output = interpreter->typed_tensor<float>(2);
  auto output_number_of_pixels = wanted_height * wanted_height * wanted_channels;

  for (int i = 0; i < output_number_of_pixels; i++) {
// hard coded for uint8
    out[i] = static_cast<uint8_t>(output[i]);
  }
}

// Returns the top N confidence values over threshold in the provided vector,
// sorted by confidence in descending order.
// Hard coded for float32
template <class T>
void get_top_n(T* prediction, 
    int prediction_size, 
    size_t num_results,
    float threshold, 
    std::vector<std::pair<float, int>>* top_results) 
{
  // Will contain top N results in ascending order.
  std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>,
                      std::greater<std::pair<float, int>>>
      top_result_pq;

  const long count = prediction_size;  // NOLINT(runtime/int)
  float value = 0.0;

  for (int i = 0; i < count; ++i) {
// hard coded for kTfLiteFloat32
    value = prediction[i];
    // Only add it if it beats the threshold and has a chance at being in
    // the top N.
    if (value < threshold) {
      continue;
    }

    top_result_pq.push(std::pair<float, int>(value, i));

    // If at capacity, kick the smallest value out.
    if (top_result_pq.size() > num_results) {
      top_result_pq.pop();
    }
  }

  // Copy to output vector and reverse into descending order.
  while (!top_result_pq.empty()) {
    top_results->push_back(top_result_pq.top());
    top_result_pq.pop();
  }
  std::reverse(top_results->begin(), top_results->end());
}




void visualize(cv::Mat& input, 
    float *scores, 
    cv::Mat& lions, 
    double fps, 
    int best_index)
{
    int thickness = 2;
    float text_size = 1;
    int text_h = 30;
    struct timespec settings_time2;
    clock_gettime(CLOCK_MONOTONIC, &settings_time2);
    double delta = 
        (double)((settings_time2.tv_sec * 1000 + settings_time2.tv_nsec / 1000000) -
        (settings_time.tv_sec * 1000 + settings_time.tv_nsec / 1000000)) / 1000;
    double scale = (double)PREVIEW_W / RAW_W;
    double x_offset = 0;
    double y_offset = 0;
    if(landscape)
    {
        x_offset = prev_window_x * scale;
    }
    else
    {
        y_offset = (PREVIEW_W - (PREVIEW_H * 3 / 2)) / 2;
    }

    if(delta < 3)
    {
        putText(input, 
            "SETTINGS UPDATED", 
            cv::Point(input.cols / 2, input.rows / 2), 
            cv::FONT_HERSHEY_SIMPLEX, 
            text_size, 
            cv::Scalar(0, 255, 0), 
            2);
    }

    for (int i = 0; i < lions.rows; i++)
    {
        int x = (int)(lions.at<int>(i, 0) * scale + x_offset);
        int y = (int)(lions.at<int>(i, 1) * scale + y_offset);
// Draw bounding box
        auto color = cv::Scalar(0, 255, 0);
        if(i == best_index)
        {
            color = cv::Scalar(0, 0, 255);
        }
        rectangle(input, 
            cv::Rect2i(x, // x
                y, 
                (int)(lions.at<int>(i, 2) * scale),  // w
                (int)(lions.at<int>(i, 3) * scale)), // h
            color, thickness);

// draw score
        std::string scoreString = cv::format("%.2f", 
            scores[i]);
//         std::string scoreString = cv::format("%.2f %s", 
//             scores[i],
//             best_index == i ? "REAL LION" : "FAKE LION");
        
        
        int score_y = y - 2;
        if(y < text_h)
        {
            score_y += text_h;
        }
        putText(input, 
            scoreString, 
            cv::Point(x, 
                score_y), 
            cv::FONT_HERSHEY_SIMPLEX, 
            text_size, 
            cv::Scalar(0, 255, 0), 
            2);
    }

    std::string fpsString = cv::format("%.2f FPS", (float)fps);
    putText(input, 
        fpsString, 
        cv::Point(0, text_h), 
        cv::FONT_HERSHEY_SIMPLEX, 
        text_size, 
        cv::Scalar(0, 255, 0), 
        2);
}




static const char* signal_titles[] =
{
	"NULL",
	"SIGHUP",
	"SIGINT",
	"SIGQUIT",
	"SIGILL",
	"SIGTRAP",
	"SIGABRT",
	"SIGBUS",
	"SIGFPE",
	"SIGKILL",
	"SIGUSR1", // 10
	"SIGSEGV",
	"SIGUSR2",
	"SIGPIPE",
	"SIGALRM",
	"SIGTERM",
    "SIGSTKFLT",
    "SIGCHLD",
    "SIGCONT",
    "SIGSTOP",
    "SIGTSTP", // 20
};



void quit(int sig)
{
// reset the console
	struct termios info;
	tcgetattr(fileno(stdin), &info);
	info.c_lflag |= ICANON;
	info.c_lflag |= ECHO;
	tcsetattr(fileno(stdin), TCSANOW, &info);

    printf("quit %d sig=%s\n", __LINE__, signal_titles[sig]);

    if(ffmpeg_fd)
    {
        printf("quit %d\n", __LINE__);
        fclose(ffmpeg_fd);
        printf("quit %d\n", __LINE__);
    }
    exit(0);
}

void ignore(int sig)
{
    printf("ignore %d sig=%s\n", __LINE__, signal_titles[sig]);
}






int main(int argc, char *argv[])
{
// reset the console for most signals
// ignore signals from the child process
    signal(SIGHUP, quit);
//    signal(SIGINT, quit);
    signal(SIGQUIT, quit);
    signal(SIGTRAP, quit);
    signal(SIGABRT, quit);
    signal(SIGBUS, quit);
    signal(SIGKILL, quit);
    signal(SIGUSR1, quit);
    signal(SIGSEGV, quit);
    signal(SIGUSR2, quit);
    signal(SIGPIPE, ignore);
    signal(SIGALRM, quit);
    signal(SIGTERM, quit);
    signal(SIGCHLD, ignore);

    load_defaults();


    std::unique_ptr<tflite::FlatBufferModel> model;
    std::unique_ptr<tflite::Interpreter> interpreter;
    tflite::ops::builtin::BuiltinOpResolver resolver;

    printf("main %d loading %s\n", __LINE__, MODEL);
    model = tflite::FlatBufferModel::BuildFromFile(MODEL);
    tflite::InterpreterBuilder builder(*model, resolver);
    builder(&interpreter);
    interpreter->AllocateTensors();
//    tflite::PrintInterpreterState(interpreter.get());
    interpreter->Invoke();
//    tflite::PrintInterpreterState(interpreter.get());

    interpreter->SetNumThreads(4);

    const std::vector<int> inputs = interpreter->inputs();
    int input = inputs[0];
    TfLiteIntArray* dims = interpreter->tensor(input)->dims;
    input_h = dims->data[1];
    input_w = dims->data[2];
    int wanted_channels = dims->data[3];
    int input_type = interpreter->tensor(input)->type; // 3 = kTfLiteUInt8

// if the output layers are sorted, the score & location are always in
// the same slots
    std::vector<int> outputs = interpreter->outputs();

    std::sort(outputs.begin(), outputs.begin() + outputs.size());
/// the output slots are hard coded
    int output_location_layer = outputs[0];
    int output_score_layer = outputs[2];
    TfLiteIntArray* output_dims = interpreter->tensor(output_score_layer)->dims;
    auto output_size = output_dims->data[output_dims->size - 1];
    int output_type = interpreter->tensor(output_score_layer)->type; // 1 = kTfLiteFloat32

    TfLiteIntArray* location_dims = interpreter->tensor(output_location_layer)->dims;

    printf("main %d input layer w=%d h=%d\n", 
        __LINE__, 
        input_w, 
        input_h);

    if(wanted_channels != 3)
    {
        printf("main %d: %d channels not supported\n", __LINE__, wanted_channels);
        exit(1);
    }

    if(input_type != kTfLiteUInt8)
    {
        printf("main %d: input_type %d not supported\n", __LINE__, input_type);
        exit(1);
    }


    init_hdmi();

    init_server();

    init_servos();

    int frame_size2;
    cv::Mat rawData;
    cv::Mat raw_image;
    cv::Mat preview_image(PREVIEW_H, PREVIEW_W, CV_8UC3);
    cv::Mat cropped;
    cv::Mat cropped_image;
    cv::Mat rotated_image(P_CROP_H, P_CROP_W, CV_8UC3);
    cv::Mat rotated_image2(PREVIEW_H, PREVIEW_W, CV_8UC3);

    struct timespec feedback_time1;
    int have_feedback_time1 = 0;
    float fps = 0;
    int fps_frames = 0;
    struct timespec fps_time1;
    clock_gettime(CLOCK_MONOTONIC, &fps_time1);



// mane loop
    while(1)
    {
        int got_it = 0;


// wait for next frame
        sem_wait(&frame_ready_sema);
        pthread_mutex_lock(&frame_lock);
        int frame_size2 = frame_size;
        if(frame_size > 0)
        {
            rawData.reserveBuffer(frame_size);
            memcpy(rawData.ptr(), reader_buffer3, frame_size);
            frame_size = 0;
        }
        pthread_mutex_unlock(&frame_lock);

        if(frame_size2 > 0)
        {
            raw_image = cv::imdecode(rawData, cv::IMREAD_COLOR);
            if(raw_image.cols > 0 && raw_image.rows > 0)
            {
                got_it = 1;
            }
        }


        if(got_it)
        {
            uint8_t *input_tensor = interpreter->typed_tensor<uint8_t>(input);
            prev_window_x = window_x;

// in landscape mode, crop & pan a square window
            if(landscape)
            {
                cv::Rect cropping(window_x, 
                    0, 
                    CROP_W,
                    CROP_H);
    // just changes the starting pointer & row stride
                cropped = raw_image(cropping);
    // allocates a contiguous frame buffer
                cropped_image = cropped.clone();
                resize<uint8_t>(input_tensor, 
                    cropped_image.ptr(0),
                    CROP_H,
                    CROP_W,
                    3,
                    input_h,
                    input_w,
                    3);
            }
            else
            {
// in portrait mode, crop & scale a center window
                cv::Rect cropping((RAW_W - P_CROP_H) / 2, 
                    0, 
                    P_CROP_H,
                    P_CROP_W);
                cropped = raw_image(cropping);
                rotate(cropped, rotated_image, cv::ROTATE_90_COUNTERCLOCKWISE);
// allocate a contiguous frame buffer
//                cropped_image = cropped.clone();
                resize<uint8_t>(input_tensor, 
                    rotated_image.ptr(0),
                    P_CROP_H,
                    P_CROP_W,
                    3,
                    input_h,
                    input_w,
                    3);
            }

// comment this out to not do any detection
            interpreter->Invoke();


            std::vector<std::pair<float, int>> top_results;
            get_top_n<float>(interpreter->typed_tensor<float>(output_score_layer),
              output_size, 
              MAX_HITS, // total results
              THRESHOLD, // threshold
              &top_results);


            
            cv::Mat boxes(top_results.size(), 4, CV_32S);
            float scores[top_results.size()] = { 0 };
            int i = 0;
            for(const auto& result : top_results)
            {
// get the location box
                float score = result.first;
                int index = result.second;
                float *box = interpreter->typed_tensor<float>(output_location_layer) + 
                    index * 4;

                scores[i] = score;
                if(landscape)
                {
                    boxes.at<int>(i, 0) = box[1] * CROP_W;
                    boxes.at<int>(i, 1) = box[0] * CROP_H;
                    boxes.at<int>(i, 2) = (box[3] - box[1]) * CROP_W;
                    boxes.at<int>(i, 3) = (box[2] - box[0]) * CROP_H;
                }
                else
                {
                    boxes.at<int>(i, 0) = box[1] * P_CROP_W;
                    boxes.at<int>(i, 1) = box[0] * P_CROP_H;
                    boxes.at<int>(i, 2) = (box[3] - box[1]) * P_CROP_W;
                    boxes.at<int>(i, 3) = (box[2] - box[0]) * P_CROP_H;
                }

//                 printf("main %d score=%f index=%d %f %f %f %f\n", 
//                     __LINE__, 
//                     score, 
//                     index,
//                     box[0],
//                     box[1],
//                     box[2],
//                     box[3]);
                i++;
            }

// search for biggest hit
            int best_index = -1;
//             int max_size = 0;
//             for(int i = 0; i < boxes.rows; i++)
//             {
//                 int w = boxes.at<int>(i, 2);
//                 int h = boxes.at<int>(i, 3);
//                 int area = w * h;
//                 if(area > max_size)
//                 {
//                     max_size = area;
//                     best_index = i;
//                 }
//             }

// use best score
            if(top_results.size() > 0)
            {
                best_index = 0;
            }



// generate servo command
            if(best_index >= 0)
            {
                int x;
                int y;
                int w;
                int h;

                if(boxes.rows == 1)
                {
                    x = boxes.at<int>(best_index, 0);
                    y = boxes.at<int>(best_index, 1);
                    w = boxes.at<int>(best_index, 2);
                    h = boxes.at<int>(best_index, 3);
                }
                else
                {
// If 2 are visible, combine the 2 boxes
                    int x1 = boxes.at<int>(0, 0); // left
                    int y1 = boxes.at<int>(0, 1); // top
                    int x2 = x1 + boxes.at<int>(0, 2); // right
                    int y2 = y1 + boxes.at<int>(0, 3); // bottom
                    int x3 = boxes.at<int>(1, 0); // left
                    int y3 = boxes.at<int>(1, 1); // top
                    int x4 = x3 + boxes.at<int>(1, 2); // right
                    int y4 = y3 + boxes.at<int>(1, 3); // bottom

                    x = MIN(x1, x3);
                    y = MIN(y1, y3);
                    w = MAX(x2, x4) - x;
                    h = MAX(y2, y4) - y;
                }


                if(current_operation == TRACKING)
                {
// get frame period in seconds
                    struct timespec feedback_time2;
                    clock_gettime(CLOCK_MONOTONIC, &feedback_time2);
                    double delta = 
                        (double)((feedback_time2.tv_sec * 1000 + feedback_time2.tv_nsec / 1000000) -
                        (feedback_time1.tv_sec * 1000 + feedback_time1.tv_nsec / 1000000)) / 1000;
                    feedback_time1 = feedback_time2;
                    if(!have_feedback_time1)
                    {
                        delta = 0;
                    }
                    have_feedback_time1 = 1;

                    int center_x;
                    int center_y;
                    int width;
                    int height;
                    int top_y;

                    if(landscape)
                    {
                        center_x = RAW_W / 2;
                        center_y = RAW_H / 2;
                        width = RAW_W;
                        height = RAW_H;
                        top_y = FROM_PERCENT_Y(lenses[lens].top_y);
                    }
                    else
                    {
                        width = RAW_H;
                        height = width * 3 / 2;
                        center_x = RAW_H / 2;
                        center_y = height / 2;
                        top_y = FROM_PERCENT_Y(lenses[lens].top_y);
                    }

                    int x_error = 0;
                    
                    if(landscape)
                    {
                        x_error = (prev_window_x + x + w / 2) - center_x;
                    }
                    else
                    {
                        x_error = (x + w / 2) - center_x;
                    }

                    if(TO_PERCENT_X(abs(x_error)) > lenses[lens].deadband)
                    {
                        float pan_change = delta * 
                            lenses[lens].x_gain * 
                            TO_PERCENT_X(x_error);
                        CLAMP(pan_change, 
                            -lenses[lens].max_pan_change, 
                            lenses[lens].max_pan_change);
                        pan += pan_change * pan_sign;
                    }

                    int y_error = 0;

//printf("main %d y=%d top_y=%d y+h/2=%d center_y=%d\n", __LINE__, y, top_y, y + h / 2, center_y);
// head is too high
//                    if(y < top_y)
// box is too tall
                    if(h >= lenses[lens].min_h * height / 100)
                    {
//printf("main %d track head\n", __LINE__);
// track the head
                        y_error = y - top_y;
                    }
                    else
                    {
//printf("main %d track center\n", __LINE__);
// track the center
                        y_error = (y + h / 2) - center_y;
                    }

                    if(abs(y_error) > FROM_PERCENT_Y(lenses[lens].deadband))
                    {
                        float tilt_change = delta * 
                            lenses[lens].y_gain * 
                            TO_PERCENT_Y(y_error);
                        CLAMP(tilt_change, 
                            -lenses[lens].max_tilt_change, 
                            lenses[lens].max_tilt_change);
                        tilt -= tilt_change * tilt_sign;
                    }


                    write_servos(0);
                }
                else
                {
                    have_feedback_time1 = 0;
                }


// center the window position on the hits
                if(landscape)
                {
                    window_x = window_x + x + w / 2 - CROP_W / 2;
//printf("main %d x=%d w=%d window_x=%d\n", __LINE__, x, w, window_x);
                    if(window_x < 0)
                    {
                        window_x = 0;
                    }
                    else
                    if(window_x > RAW_W - CROP_W)
                    {
                        window_x = RAW_W - CROP_W;
                    }
                    crop_state = LOCKED;
                    crop_counter = 0;
                }
            }
            else
            if(landscape)
            {
// got nothing
                switch(crop_state)
                {
                    case SCANNING0:
                        window_x = RAW_W / 2 - CROP_W / 2;
                        crop_state = SCANNING1;
                        break;
                    case SCANNING1:
                        window_x = RAW_W - CROP_W;
                        crop_state = SCANNING2;
                        break;
                    case SCANNING2:
                        window_x = 0;
                        crop_state = SCANNING0;
                        break;
                    case LOCKED:
                        if(crop_counter < CROP_TIMEOUT)
                        {
                            crop_counter++;
                        }
                        else
                        {
                            window_x = RAW_W / 2 - CROP_W / 2;
                            crop_state = SCANNING1;
                        }
                        break;
                }
            }



            fps_frames++;
            struct timespec fps_time2;
            clock_gettime(CLOCK_MONOTONIC, &fps_time2);
            double delta = 
                (double)((fps_time2.tv_sec * 1000 + fps_time2.tv_nsec / 1000000) -
                (fps_time1.tv_sec * 1000 + fps_time1.tv_nsec / 1000000)) / 1000;
            if(delta > 1.0)
            {
                fps_time1 = fps_time2;
                fps = fps_frames / delta;
                fps_frames = 0;
                printf("main %d fps=%f\n", 
                    __LINE__, 
                    fps);
            }



// send frame to phone
            if(server_output >= 0)
            {
// need the entire viewfinder with the cropped section overlaid
                resize(raw_image, 
                    preview_image,
                    cv::Size(PREVIEW_W, PREVIEW_H),
                    cv::INTER_LINEAR); // no speed difference
                if(landscape)
                {
                    visualize(preview_image, scores, boxes, fps, 0);
                }
                else
                {
                    rotate(preview_image, rotated_image2, cv::ROTATE_90_COUNTERCLOCKWISE);
                    visualize(rotated_image2, scores, boxes, fps, 0);

// unrotate it
                    rotate(rotated_image2, preview_image, cv::ROTATE_90_CLOCKWISE);
                }


// compress directly to server FIFO
                std::vector<uchar> buf;
                std::vector<int> compressing_factor;
                compressing_factor.push_back(cv::IMWRITE_JPEG_QUALITY);
                compressing_factor.push_back(50);
                cv::imencode(".jpg", 
                    preview_image, 
                    buf, 
                    compressing_factor);
// insert window_x at beginning of buffer
                int jpg_size = buf.size();
                buf.resize(jpg_size + 4);
                memmove(&buf[4], &buf[0], jpg_size);
// scale window_x to preview size
                int preview_x = 0;
                if(landscape)
                {
                    preview_x = prev_window_x * PREVIEW_W / CROP_W;
                }
                else
                {
                    preview_x = 0;
                }
                
                buf[0] = preview_x & 0xff;
                buf[1] = (preview_x >> 8) & 0xff;
                buf[2] = (preview_x >> 16) & 0xff;
                buf[3] = (preview_x >> 24) & 0xff;
                send_vijeo_fifo(&buf[0], buf.size());


#ifdef CAPTURE
// store raw frame
                if(current_operation == TRACKING)
                {
                    if(!ffmpeg_fd)
                    {
                        char string[TEXTLEN];
            		    uuid_t temp_id;
                        uuid_generate(temp_id);
                        sprintf(string, 
    //                        "ffmpeg -y -f rawvideo -y -pix_fmt bgr24 -r 10 -s:v %dx%d -i - -c:v h264 -crf 30 -pix_fmt yuvj420p -an ",
                            "ffmpeg -y -f rawvideo -y -pix_fmt bgr24 -r 10 -s:v %dx%d -i - -c:v mpeg4 -qscale:v 5 -pix_fmt yuvj420p -an ",
                            PREVIEW_W,
                            PREVIEW_H);
                        uuid_unparse(temp_id, string + strlen(string));
                        strcat(string, ".mp4");
                        printf("main %d: running %s\n", __LINE__, string);
                        ffmpeg_fd = popen(string, "w");
                    }

                    if(ffmpeg_fd)
                    {
                        fwrite(preview_image.ptr(0), 
                            1, 
                            PREVIEW_W * PREVIEW_H * 3, 
                            ffmpeg_fd);
                    }
                    else
                    {
                        printf("main %d: Error running ffmpeg\n", __LINE__);
                        sleep(1);
                    }
                }
                else
                if(ffmpeg_fd != 0)
                {
                    pclose(ffmpeg_fd);
                    ffmpeg_fd = 0;
                    int _ = system("sync");
                }
#endif // CAPTURE

            }
        }
    }


    return 0;
}


















