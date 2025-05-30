/*
 * Tracking camera using tensorflow
 *
 * Copyright (C) 2022-2024 Adam Williams <broadcast at earthling dot net>
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



// track whole body using tensorflow-lite

// make deps
// make truckflow



#include <opencv4/opencv2/dnn.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/objdetect.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"
#include "tensorflow/lite/builtin_op_data.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <libusb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>
#include <linux/videodev2.h>
#include <queue>

#include "tracker.h"
#include "trackerlib.h"
#include "trackerserver.h"
#include "truckflow.h"

extern "C"
{
#include <uuid/uuid.h>
}

using namespace cv;
using namespace std;


//#define MODEL "efficientlion0.yolo.300.tflite"
#define MODEL "efficientlion-lite1.300.tflite"
//#define MODEL "efficientlion-lite2.10.tflite"
#define MAX_HITS 4

// disable this to capture at the maximum framerate
#define ENABLE_TRACKER


//#define READ_TEST_IMAGE
//#define READ_TEST_VIDEO
//#define TEST_VIDEO_PATH "test_video.mp4"
#define TEST_VIDEO_PATH "multi_animal4.mp4"
//#define TEST_VIDEO_PATH "67f81122-2c97-49e7-8d32-2ac8fee315a4.mp4"

// score required to get a hit with efficientdet
#define THRESHOLD 0.5

// analog values from arm_cam.c
// an offset is added to the ADC to get this range
#define ADC_CENTER 128
#define ADC_DEADBAND 5
#define ADC_MAX 64
#define MAX_ADC_OFFSET 32
// minimums
#define MIN_LEFT (ADC_CENTER + ADC_DEADBAND)
#define MIN_RIGHT (ADC_CENTER - ADC_DEADBAND)
// maximums
#define MAX_LEFT (ADC_CENTER + ADC_MAX)
#define MAX_RIGHT (ADC_CENTER - ADC_MAX)


// pointers for server writer
// when the value is <= 0, the buffer is ready to accept new data
// when the value is > 0, it's being written
int current_input2 = -1;
int vijeo_offset2 = -1;
// storage for packet header, keypoints & compressed frame
uint8_t vijeo_buffer[HEADER_SIZE + MAX_JPEG * 2];
int status_size = 0;
uint8_t status_buffer[HEADER_SIZE + 256];

#define SETTINGS_SIZE 10
uint8_t settings_buffer[SETTINGS_SIZE];
int have_settings = 0;
// packet to forward to the servo panner
#define SERVO_CONFIG_SIZE 7
uint8_t servo_config[SERVO_CONFIG_SIZE];


// device buffers
#define DEVICE_BUFFERS 1
unsigned char *mmap_buffer[DEVICE_BUFFERS];
// detect USB enumeration
#define FISHEYE_VID 0x32e4
#define FISHEYE_PID 0x9230

// decompressed images
// planar YUV420
uint8_t *raw_y = 0;
uint8_t *raw_u = 0;
uint8_t *raw_v = 0;
// defished image in YUV444 planar
Mat defished_y;
Mat defished_u;
Mat defished_v;
#define IMAGE_BUFFERS 2
// preview image for phone & efficientlion
Mat preview_y;
Mat preview_u;
Mat preview_v;
Mat preview[IMAGE_BUFFERS];
int current_input = 0;

#define PARSE_COMMAND 0
#define PARSE_SETTINGS 1
int parsing_state = PARSE_COMMAND;
int counter = 0;
uint8_t read_packet[16];


#define BUFSIZE2 0x400000
int got_image = 0;
static pthread_mutex_t frame_lock;
static sem_t frame_ready_sema;
int hdmi_fd = -1;
struct timespec fps_time1;
char current_path[TEXTLEN] = { 0 };
// input size for the neural network
int input_w = 0;
int input_h = 0;



#ifdef FISHEYE
// everything is scaled to the input layer size during defishing
// size of tile for efficientdet
#define TILE_W input_w
#define TILE_H input_h
#define DEFISH_THREADS 4
//#define DEFISH_THREADS 1
// topology for efficientlion
#define THREADS 2
#define TILES 2
// size of defished image
#define DEFISH_H 540
#define DEFISH_W (DEFISH_H * 3 / 2)
// size of preview for phone & efficientlion
#define PREVIEW_H TILE_H
#define PREVIEW_W (PREVIEW_H * DEFISH_W / DEFISH_H)
// must set this at runtime after input_w is known
int tile_x[TILES];
// defishing table
#define SMOOTH_DEFISH
    #ifndef SMOOTH_DEFISH
    int *x_lookup;
    int *y_lookup;
    #else
    float *x_lookup;
    float *y_lookup;
    #endif
#endif // FISHEYE


// blending of target histogram must be fast enough to keep up with
// lighting changes & slow enough to defeat glitches.
#define WEIGHT (1.0 / 15)
// must be low enough for chi squared to align the bins but high enough
// to store more spatial information
#define BINS 32
// the target histogram
float want_hist[BINS * 3] = { 0 };

// unique boxes
class Box
{
public:
    int x, y, w, h;
    float score;
// the histogram
    float hist[BINS * 3];
// chi squared difference
    float distance;
// tile the box came from
    int tile;
};
vector<Box> boxes;
int nearest_box = -1;

int servo_fd = -1;
static pthread_mutex_t servo_lock;
uint8_t servo_command = 0;

// error bits
// servos not detected
#define SERVO_ERROR 0x80
#define CAM_ERROR_MASK 0x0f
// /dev/video* doesn't exist
#define VIDEO_DEVICE_ERROR 1
// open, ioctl failed
#define VIDEO_CONFIG_ERROR 2
// camera USB not enumerating
#define CAM_ENUM_ERROR 3
// camera is currently starting
#define CAM_STARTING_ERROR 4
// select or buffer failed
#define CAM_SELECT_ERROR 5
// select or buffer failed
#define CAM_BUFFER_ERROR 6

#define CLEAR_CAM_ERRORS \
error_flags &= ~CAM_ERROR_MASK;

uint8_t error_flags = 0x00;
uint8_t prev_error_flags = 0x00;
FILE *ffmpeg_writer = 0;

// the servo controller ADC for truck cam
int adc = 0;
int deadband = 5;
int speed = 100;
// capture raw video for training
int do_capture = 0;
int frames_captured = 0;

int current_operation = IDLE;
void do_defish();

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






int probe_usb(uint32_t vid, uint32_t pid)
{
	struct libusb_device_handle *devh = 
        libusb_open_device_with_vid_pid(0, vid, pid);
	if(devh)
	{
//         printf("probe_usb %d vid=%04x pid=%04x\n",
//             __LINE__,
//             vid,
//             pid);
		libusb_close(devh);
		return 1;
	}
    return 0;
}


// probe for the video device
int open_hdmi(int verbose)
{
    int got_usb = 0;

// faster probing through libusb
    if(probe_usb(FISHEYE_VID, FISHEYE_PID))
    {
        got_usb = 1;
    }

    if(!got_usb)
    {
//        printf("open_hdmi %d: no USB device\n", __LINE__);
        CLEAR_CAM_ERRORS
        error_flags |= CAM_ENUM_ERROR;
        send_error();
        sleep(1);
        return -1;
    }



    vector<char*> paths;
    char string[TEXTLEN];
// this causes a SIGCHLD
    FILE *fd = popen("ls /dev/video*", "r");
    current_path[0] = 0;
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

    if(paths.size() == 0)
    {
        printf("open_hdmi %d: no paths found\n", __LINE__);
        CLEAR_CAM_ERRORS
        error_flags |= VIDEO_DEVICE_ERROR;
        send_error();
        sleep(1);
        return -1;
    }

    int fd2 = -1;
    for(int i = 0; i < paths.size(); i++)
    {
        printf("open_hdmi %d: opening %s\n", __LINE__, paths.at(i));

// open is slow on the jetson but not the rasp
//         CLEAR_CAM_ERRORS
//         error_flags |= CAM_STARTING_ERROR;
//         send_error();

        fd2 = open(paths.at(i), O_RDWR);
        if(fd2 < 0) continue;

// set webcam parameters
// test the W & H to find the right camera
        struct v4l2_format v4l2_params;
        bzero(&v4l2_params, sizeof(v4l2_params));
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
                paths.at(i),
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
                    printf("open_hdmi %d id=%x min=%d max=%d step=%d default=%d %s\n", 
                        __LINE__, 
                        arg.id, 
                        arg.minimum,
                        arg.maximum,
                        arg.step,
                        arg.default_value,
                        arg.name);
                }
            }

// // backlight compensation
//                 struct v4l2_control ctrl_arg;
//                 ctrl_arg.id = 0x98091c;
//                 if(ioctl(fd2, VIDIOC_G_CTRL, &ctrl_arg))
//                 {
//                     printf("open_hdmi %d VIDIOC_G_CTRL failed\n", __LINE__);
//                 }
//                 else
//                 {
//                     printf("open_hdmi %d got backlight compensation %d\n",
//                         __LINE__,
//                         ctrl_arg.value);
//                 }
// 
//                 ctrl_arg.id = 0x98091c;
//                 ctrl_arg.value = 0;
//                 if(ioctl(fd2, VIDIOC_S_CTRL, &ctrl_arg) < 0)
//                 {
//                     printf("open_hdmi %d VIDIOC_S_CTRL failed\n",
//                         __LINE__);
// 
//     // skip the device
//                     close(fd2);
//                     fd2 = -1;
//                 }
//                 else
//                 {
//                     printf("open_hdmi %d setting backlight compensation to %d\n",
//                         __LINE__,
//                         ctrl_arg.value);
//                 }
        }

//         if(fd2 >= 0)
//         {
// // brightness
//             struct v4l2_control ctrl_arg;
//             ctrl_arg.id = 0x980900;
//             ctrl_arg.value = 255;
//             if(ioctl(fd2, VIDIOC_S_CTRL, &ctrl_arg) < 0)
//             {
//                 printf("open_hdmi %d VIDIOC_S_CTRL failed\n",
//                     __LINE__);
//             }
//         }

//         if(fd2 >= 0)
//         {
// // gamma
//             struct v4l2_control ctrl_arg;
//             ctrl_arg.id = 0x980910;
//             ctrl_arg.value = 20;
//             if(ioctl(fd2, VIDIOC_S_CTRL, &ctrl_arg) < 0)
//             {
//                 printf("open_hdmi %d VIDIOC_S_CTRL failed\n",
//                     __LINE__);
//             }
//         }

//         if(fd2 >= 0)
//         {
// // contrast
//             struct v4l2_control ctrl_arg;
//             ctrl_arg.id = 0x980901;
//             ctrl_arg.value = 16;
//             if(ioctl(fd2, VIDIOC_S_CTRL, &ctrl_arg) < 0)
//             {
//                 printf("open_hdmi %d VIDIOC_S_CTRL failed\n",
//                     __LINE__);
//             }
//         }

        if(fd2 >= 0)
        {
// saturation
            struct v4l2_control ctrl_arg;
            ctrl_arg.id = 0x980902;
            if(ioctl(fd2, VIDIOC_G_CTRL, &ctrl_arg) < 0)
            {
                printf("open_hdmi %d VIDIOC_G_CTRL failed\n",
                    __LINE__);
            }
            else
            {
                printf("open_hdmi %d got saturation %d\n",
                    __LINE__,
                    ctrl_arg.value);
            }


            ctrl_arg.value = 127;
            if(ioctl(fd2, VIDIOC_S_CTRL, &ctrl_arg) < 0)
            {
                printf("open_hdmi %d VIDIOC_S_CTRL failed\n",
                    __LINE__);
            }
            else
            {
                printf("open_hdmi %d setting saturation to %d\n",
                    __LINE__,
                    ctrl_arg.value);
            }
        }




        if(fd2 >= 0)
        {
// codec
            struct v4l2_format v4l2_params;
            v4l2_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd2, VIDIOC_G_FMT, &v4l2_params);

#if defined(FISHEYE) || defined(KEYCAM)
            v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
#endif
#if defined(HDMI)
            v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#endif

            v4l2_params.fmt.pix.width = RAW_W;
            v4l2_params.fmt.pix.height = RAW_H;
            
            if(ioctl(fd2, VIDIOC_S_FMT, &v4l2_params) < 0)
            {
//              printf("open_hdmi %d: VIDIOC_S_FMT failed\n",
//                  __LINE__);
            }


            if(verbose)
            {
                printf("open_hdmi %d opened %s\n", __LINE__, paths.at(i));
                strcpy(current_path, paths.at(i));
            }
        }

// common buffer allocator
        struct v4l2_buffer buffer;
        if(fd2 >= 0)
        {
            struct v4l2_requestbuffers requestbuffers;
            requestbuffers.count = DEVICE_BUFFERS;
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
                for(int i = 0; i < DEVICE_BUFFERS; i++)
                {
                    buffer.type = requestbuffers.type;
                    buffer.index = i;

                    if(ioctl(fd2, VIDIOC_QUERYBUF, &buffer) < 0)
				    {
					    printf("open_hdmi %d: VIDIOC_QUERYBUF failed\n",
                            __LINE__);
                        close(fd2);
                        fd2 = -1;
				    }
                    else
                    {
                        mmap_buffer[i] = (unsigned char*)mmap(NULL,
					        buffer.length,
					        PROT_READ | PROT_WRITE,
					        MAP_SHARED,
					        fd2,
					        buffer.m.offset);
                        printf("open_hdmi %d: allocated buffer %d size=%d\n",
                            __LINE__,
                            i,
                            buffer.length);
                        if(ioctl(fd2, VIDIOC_QBUF, &buffer) < 0)
                        {
                            printf("open_hdmi %d: VIDIOC_QBUF failed\n",
                                __LINE__);
                            close(fd2);
                            fd2 = -1;
                        }
                    }
                }
            }
        }

        if(fd2 >= 0)
        {
            int streamon_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            printf("open_hdmi %d: starting\n", __LINE__);
	        if(ioctl(fd2, VIDIOC_STREAMON, &streamon_arg) < 0)
            {
		        printf("open_hdmi %d: VIDIOC_STREAMON failed\n",
                    __LINE__);
                for(int i = 0; i < DEVICE_BUFFERS; i++)
                    munmap(mmap_buffer[i], buffer.length);
                close(fd2);
                fd2 = -1;
            }
            clock_gettime(CLOCK_MONOTONIC, &fps_time1);
        }

        if(fd2 >= 0) break;

    } // paths
//printf("open_hdmi %d fd2=%d\n", __LINE__, fd2);

// delete the paths
    while(paths.size() > 0)
    {
        free(paths.back());
        paths.pop_back();
    }

    if(fd2 < 0)
    {
        printf("open_hdmi %d: open/config error\n", __LINE__);
        CLEAR_CAM_ERRORS
        error_flags |= VIDEO_CONFIG_ERROR;
        send_error();
        sleep(1);
        return -1;
    }
    else
    {
        printf("open_hdmi %d: video configured\n", __LINE__);
    }

    return fd2;
}


// read frames from HDMI
void* hdmi_thread(void *ptr)
{
    struct timeval time1;
    gettimeofday(&time1, 0);
    INIT_PROFILE

    int verbose = 1;
    int frame_count = 0;


    while(1)
    {
        if(hdmi_fd < 0) 
        {
            hdmi_fd = open_hdmi(verbose);
        }



        if(hdmi_fd >= 0)
        {
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
                CLEAR_CAM_ERRORS
                error_flags |= CAM_SELECT_ERROR;
                send_error();
                close(hdmi_fd);
                hdmi_fd = -1;
                sleep(1);
            }
            else
            {
//                UPDATE_PROFILE
//                printf("hdmi_thread %d delta=%f\n", __LINE__, delta);
                struct v4l2_buffer buffer;
		        bzero(&buffer, sizeof(buffer));
                buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		        buffer.memory = V4L2_MEMORY_MMAP;

                if(ioctl(hdmi_fd, VIDIOC_DQBUF, &buffer) < 0)
                {
                    printf("hdmi_thread %d: VIDIOC_DQBUF failed\n",
                        __LINE__);
                    CLEAR_CAM_ERRORS
                    error_flags |= CAM_BUFFER_ERROR;
                    send_error();
                    close(hdmi_fd);
                    hdmi_fd = -1;
                    sleep(1);
                }
                else
                {
                    CLEAR_CAM_ERRORS
                    send_error();

//                  printf("hdmi_thread %d\n", __LINE__);
                    unsigned char *ptr = mmap_buffer[buffer.index];
//                     printf("hdmi_thread %d: index=%d size=%d %02x %02x %02x %02x %02x %02x %02x %02x\n",
//                         __LINE__,
//                         buffer.index,
//                         buffer.bytesused,
//                         ptr[0],
//                         ptr[1],
//                         ptr[2],
//                         ptr[3],
//                         ptr[4],
//                         ptr[5],
//                         ptr[6],
//                         ptr[7]);

                    if(buffer.bytesused > BUFSIZE2)
                    {
                        printf("hdmi_thread %d frame too big.  Used %d\n", 
                            __LINE__,
                            buffer.bytesused);
                    }
                    else
                    if(
// test for valid JPEG image
// HDMI
                        (ptr[0] == 0xff && 
                        ptr[1] == 0xd8 && 
                        ptr[2] == 0xff && 
                        ptr[3] == 0xdb) ||
// keycam
                        (ptr[0] == 0xff && 
                        ptr[1] == 0xd8 && 
                        ptr[2] == 0xff && 
                        ptr[3] == 0xc0))
                    {
// send it to processor
                        pthread_mutex_lock(&frame_lock);
                        if(!got_image)
                        {
                            int decoded_w;
                            int decoded_h;
                            decompress_jpeg_yuv(ptr, 
                                buffer.bytesused,
                                &decoded_w,
                                &decoded_h,
                                raw_y,
                                raw_u,
                                raw_v);

                            got_image = 1;
                        }
                        pthread_mutex_unlock(&frame_lock);
                        sem_post(&frame_ready_sema);
                    }

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
//                         printf("hdmi_thread %d device FPS: %f\n",
//                             __LINE__,
//                             (double)frame_count * 1000 / diff);
                        frame_count = 0;
                        time1 = time2;
                    }
                } // VIDIOC_DQBUF
            } // select
        } // hdmi_fd >= 0
    } // while(1)
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
// 	pthread_create(&x, 
// 		0, 
// 		hdmi_watchdog, 
// 		0);
}

int vtor_tab[0x100], vtog_tab[0x100];
int utog_tab[0x100], utob_tab[0x100];

// All variables are unsigned
// y -> 24 bits u, v, -> 8 bits r, g, b -> 8 bits
#define YUV_TO_RGB(y, u, v, r, g, b) \
{ \
	(r) = ((y + vtor_tab[v]) >> 16); \
	(g) = ((y + utog_tab[u] + vtog_tab[v]) >> 16); \
	(b) = ((y + utob_tab[u]) >> 16); \
	CLAMP(r, 0, 0xff); \
	CLAMP(g, 0, 0xff); \
	CLAMP(b, 0, 0xff); \
}

void init_yuv()
{
	int *vtor = &(vtor_tab[0x80]);
	int *vtog = &(vtog_tab[0x80]);
	int *utog = &(utog_tab[0x80]);
	int *utob = &(utob_tab[0x80]);
    int i;

// Decompression coefficients straight out of jpeglib
#define V_TO_R    1.40200
#define V_TO_G    -0.71414

#define U_TO_G    -0.34414
#define U_TO_B    1.77200

	for(i = -0x80; i < 0x80; i++)
	{
		vtor[i] = (int)(V_TO_R * 0x10000 * i);
		vtog[i] = (int)(V_TO_G * 0x10000 * i);

		utog[i] = (int)(U_TO_G * 0x10000 * i);
		utob[i] = (int)(U_TO_B * 0x10000 * i);
	}
}

void* servo_reader(void *ptr)
{
    while(1)
    {
        if(servo_fd < 0)
        {
	        servo_fd = init_serial("/dev/ttyACM0");
	        if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM1");
	        if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM2");
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
            int repeat_count = 0;
            int prev_button = 0;
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

void init_servos()
{
    pthread_t x;
	pthread_create(&x, 
		0, 
		servo_reader, 
		0);
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

    if(ffmpeg_writer)
    {
        printf("quit %d\n", __LINE__);
        fclose(ffmpeg_writer);
        printf("quit %d\n", __LINE__);
    }
    exit(0);
}

void ignore_(int sig)
{
//    printf("ignore %d sig=%s\n", __LINE__, signal_titles[sig]);
}


void load_defaults()
{
#define SETTINGS_DIR "/root"
// HOME not available in /etc/rc.local
//    char *home = getenv("HOME");
    const char *home = SETTINGS_DIR;
    char string[TEXTLEN];
    sprintf(string, "%s/.truckcam.rc", home);
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
        if(!strcasecmp(key, "DEADBAND"))
        {
            deadband = atoi(value);
        }
        else
        if(!strcasecmp(key, "SPEED"))
        {
            speed = atoi(value);
        }
    }

    fclose(fd);
}

void save_defaults()
{
    char *home = getenv("HOME");
    char string[TEXTLEN];
    sprintf(string, "%s/.truckcam.rc", home);
    FILE *fd = fopen(string, "w");

    if(!fd)
    {
        printf("save_defaults %d: Couldn't open %s for writing\n", 
            __LINE__, 
            string);
        return;
    }

    fprintf(fd, "########################################\n");
    fprintf(fd, "DEADBAND %d\n", (int)deadband);
    fprintf(fd, "SPEED %d\n", (int)speed);

    fclose(fd);
}

void dump_settings()
{
    printf("dump_settings %d\n", __LINE__);
    printf("DEADBAND %d\n", (int)deadband);
    printf("SPEED %d\n", (int)speed);
}

static std::vector<float> normalization(const std::vector<float>& feature)
{
    std::vector<float> ret;
    float sum = 0.0;
    for(int i = 0; i < (int)feature.size(); i++)
    {
        sum += feature[i] * feature[i];
    }
    sum = sqrt(sum);
    for(int i = 0; i < (int)feature.size(); i++)
    {
        ret.push_back(feature[i] / sum);
    }
    return ret;
}


// use tensorflow for bilinear interpolation
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

  interpreter->SetNumThreads(THREADS);
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


void send_status()
{
    if(status_size <= 0)
    {
        int offset = HEADER_SIZE;
        status_buffer[offset++] = current_operation;
        status_buffer[offset++] = error_flags;
        status_buffer[offset++] = do_capture;
//         statue_buffer[offset++] = frames_captured & 0xff;
//         statue_buffer[offset++] = (frames_captured >> 8) & 0xff;
//         statue_buffer[offset++] = (frames_captured >> 16) & 0xff;
//         statue_buffer[offset++] = (frames_captured >> 24) & 0xff;

// copy the last settings packet
        if(have_settings)
        {
            memcpy(status_buffer + offset, settings_buffer, SETTINGS_SIZE);
            offset += SETTINGS_SIZE;
            have_settings = 0;
        }

        prev_error_flags = error_flags;
        status_size = offset - HEADER_SIZE;
        wake_writer();

//    printf("send_status %d error_flags=%d\n", __LINE__, error_flags);
    }
}

void send_error()
{
    if(error_flags != prev_error_flags)
    {
        printf("send_error %d error_flags=%x\n", __LINE__, error_flags);
		send_status();
    }
}

void send_vijeo(int current_input, int offset)
{
    current_input2 = current_input;
    vijeo_offset2 = offset;
    wake_writer();
}


// not to be called while tracking
void write_servo_config()
{
    if(servo_fd >= 0)
    {
        uint8_t buffer[SERVO_CONFIG_SIZE + 2];
        buffer[0] = 0xff;
        buffer[1] = 0xcf;
        memcpy(buffer + 2, servo_config, SERVO_CONFIG_SIZE);
        int temp = write(servo_fd, buffer, SERVO_CONFIG_SIZE + 2);
    }
}


void write_servos(int use_pwm_limits)
{
	if(servo_fd >= 0)
	{
// 1 axis
        uint8_t buffer[3];
        buffer[0] = 0xff;
        buffer[1] = 0xd2;
        buffer[2] = adc;
	    int _ = write(servo_fd, buffer, 3);
//printf("write_servos %d %d\n", __LINE__, adc);
    }
}


// async writer
void server_write()
{
    if(current_input2 >= 0)
    {
// have vijeo data
        std::vector<uchar> buf;
        std::vector<int> compressing_factor;
// get under maximum UDP size of 65535
        compressing_factor.push_back(IMWRITE_JPEG_QUALITY);
        compressing_factor.push_back(50);

// seems to require BGR
        uint8_t *ptr = preview[current_input2].ptr(0);
        for(int i = 0; i < PREVIEW_H * PREVIEW_W; i++)
        {
            uint8_t temp = ptr[0];
            ptr[0] = ptr[2];
            ptr[2] = temp;
            ptr += 3;
        }

        imencode(".jpg", 
            preview[current_input2], 
            buf, 
            compressing_factor);
//printf("server_write %d %d\n", __LINE__, buf.size());

        int jpg_size = buf.size();
        int offset = vijeo_offset2;


        memcpy(vijeo_buffer + offset, &buf[0], jpg_size);
        offset += jpg_size;
        send_packet(VIJEO, 
            vijeo_buffer, // pointer to start of header
            offset - HEADER_SIZE);
        current_input2 = -1;
        vijeo_offset2 = -1;
    }



    if(status_size)
    {
        send_packet(STATUS, status_buffer, status_size);
        status_size = 0;
    }
}

// async reader
void server_read(uint8_t c)
{
    if(parsing_state == PARSE_COMMAND)
    {
        switch(c)
        {
            case '*':
                send_status();
                break;
            case ' ':
                current_operation = TRACKING;
                send_status();
                break;
            case 'q':
                current_operation = IDLE;
                send_status();
                break;
            case 's':
                parsing_state = PARSE_SETTINGS;
                counter = 0;
                break;
            case 'c':
                do_capture = !do_capture;
                break;
        }
    }
    else
    {
// PARSE_SETTINGS
        settings_buffer[counter++] = c;
        if(counter >= SETTINGS_SIZE)
        {
            parsing_state = PARSE_COMMAND;
            
            deadband = settings_buffer[1];
            speed = settings_buffer[2];
            memcpy(servo_config, settings_buffer + 3, SERVO_CONFIG_SIZE);
            write_servo_config();
            
            ::save_defaults();
            have_settings = 1;
            send_status();
            dump_settings();
        }
    }
}

// defishing engine
class Defisher
{
public:
    sem_t go;
    sem_t done;
    int row1;
    int row2;
    
    void init(int n)
    {
        row1 = n * DEFISH_H / DEFISH_THREADS;
        row2 = (n + 1) * DEFISH_H / DEFISH_THREADS;
        if((row1 % 2) > 0) row1--;
        if((row2 % 2) > 0) row2--;
//printf("Defisher::init %d: row1=%d row2=%d\n", __LINE__, row1, row2);
        sem_init(&go, 0, 0);
        sem_init(&done, 0, 0);
        pthread_attr_t  attr;
        pthread_attr_init(&attr);
        pthread_t tid;
        pthread_create(&tid, 
		    &attr, 
		    entrypoint, 
		    this);
    }


    void process()
    {
        sem_post(&go);
    }
    
    void wait()
    {
        sem_wait(&done);
    }

    static void* entrypoint(void *ptr)
    {
        Defisher *engine = (Defisher*)ptr;
        engine->run();
        return 0;
    }

    void run()
    {
        while(1)
        {
            sem_wait(&go);

// Y plane
            for(int i = row1; i < row2; i++)
            {
                uint8_t *dst_row = defished_y.ptr(0) + i * DEFISH_W;
                uint8_t *src_plane = raw_y;
                for(int j = 0; j < DEFISH_W; j++)
                {
                    float x_in = x_lookup[i * DEFISH_W + j];
                    float y_in = y_lookup[i * DEFISH_W + j];
					float y1_fraction = y_in - floor(y_in);
					float y2_fraction = 1.0 - y1_fraction;
					float x1_fraction = x_in - floor(x_in);
					float x2_fraction = 1.0 - x1_fraction;
                    uint8_t *in_pixel1 = src_plane + (int)y_in * RAW_W + (int)x_in;
                    uint8_t *in_pixel2 = in_pixel1 + RAW_W;
                    uint8_t *in_pixel3 = in_pixel1 + 1;
                    uint8_t *in_pixel4 = in_pixel2 + 1;
					*dst_row++ = (uint8_t)(
                        *in_pixel1 * x2_fraction * y2_fraction +
						*in_pixel2 * x2_fraction * y1_fraction +
						*in_pixel3 * x1_fraction * y2_fraction +
						*in_pixel4 * x1_fraction * y1_fraction);
                }
            }

// UV planes
            for(int plane = 1; plane < 3; plane++)
            {
                for(int i = row1; i < row2; i++)
                {
                    uint8_t *dst_row;
                    uint8_t *src_plane;
                    switch(plane)
                    {
                         case 1: 
                            dst_row = defished_u.ptr(0) + i * DEFISH_W; 
                            src_plane = raw_u;
                            break;
                         case 2: 
                            dst_row = defished_v.ptr(0) + i * DEFISH_W; 
                            src_plane = raw_v;
                            break;
                    }

                    for(int j = 0; j < DEFISH_W; j++)
                    {
                        float x_in = x_lookup[i * DEFISH_W + j] / 2;
                        float y_in = y_lookup[i * DEFISH_W + j] / 2;
					    float y1_fraction = y_in - floor(y_in);
					    float y2_fraction = 1.0 - y1_fraction;
					    float x1_fraction = x_in - floor(x_in);
					    float x2_fraction = 1.0 - x1_fraction;
                        uint8_t *in_pixel1 = src_plane + 
                            (int)y_in * (RAW_W / 2) + 
                            (int)x_in;
                        uint8_t *in_pixel2 = in_pixel1 + (RAW_W / 2);
                        uint8_t *in_pixel3 = in_pixel1 + 1;
                        uint8_t *in_pixel4 = in_pixel2 + 1;


//*dst_row++ = *in_pixel1;
 					    *dst_row++ = (uint8_t)(
                            *in_pixel1 * x2_fraction * y2_fraction +
 						    *in_pixel2 * x2_fraction * y1_fraction +
 						    *in_pixel3 * x1_fraction * y2_fraction +
 						    *in_pixel4 * x1_fraction * y1_fraction);
                    }
                } // i
            } // plane

            
            sem_post(&done);
        }
    }
};

Defisher defishers[DEFISH_THREADS];

// efficientlion engine
// 1 engine for each tile
class Engine
{
public:
    std::unique_ptr<tflite::FlatBufferModel> model;
    std::unique_ptr<tflite::Interpreter> interpreter;
    tflite::ops::builtin::BuiltinOpResolver resolver;
    int input;
    int tile;
    sem_t go;
    sem_t done;
    vector<int> boxes;
    vector<float> scores;
    int output_score_layer;
    int output_location_layer;
    int output_size;

    void init(const char *model_path, int tile)
    {
        this->tile = tile;
        model = tflite::FlatBufferModel::BuildFromFile(model_path);
        tflite::InterpreterBuilder builder(*model, resolver);
        builder(&interpreter);
        interpreter->SetNumThreads(THREADS);
        interpreter->AllocateTensors();
//    tflite::PrintInterpreterState(interpreter.get());
        interpreter->Invoke();
//    tflite::PrintInterpreterState(interpreter.get());


        const std::vector<int> inputs = interpreter->inputs();
        input = inputs[0];
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
        output_location_layer = outputs[0];
        output_score_layer = outputs[2];
        TfLiteIntArray* output_dims = interpreter->tensor(output_score_layer)->dims;
        output_size = output_dims->data[output_dims->size - 1];
        int output_type = interpreter->tensor(output_score_layer)->type; // 1 = kTfLiteFloat32

        TfLiteIntArray* location_dims = interpreter->tensor(output_location_layer)->dims;
    // printf("Engine::init %d output layers=%d location type=%d size=%d\n", 
    // __LINE__, 
    // outputs.size(),
    // interpreter->tensor(output_location_layer)->type,
    // location_dims->data[2]);

    // for(int i = 0; i < outputs.size(); i++)
    // {
    //     printf("Engine::init %d output %d layer=%d\n", 
    //         __LINE__, 
    //         i, 
    //         outputs[i]);
    // }

    // printf("main %d input_layers=%d w=%d h=%d channels=%d input_type=%d\n", 
    // __LINE__, 
    // inputs.size(),
    // input_w, 
    // input_h, 
    // wanted_channels,
    // input_type);
    // printf("main %d output_size=%d output_type=%d\n", 
    // __LINE__, 
    // output_size,
    // output_type);

        printf("Engine::init %d: input layer w=%d h=%d\n", 
            __LINE__, 
            input_w, 
            input_h);

        if(wanted_channels != 3)
        {
            printf("Engine::init %d: %d channels not supported\n", __LINE__, wanted_channels);
            exit(1);
        }

        if(input_type != kTfLiteUInt8)
        {
            printf("Engine::init %d: input_type %d not supported\n", __LINE__, input_type);
            exit(1);
        }
        
        sem_init(&go, 0, 0);
        sem_init(&done, 0, 0);
        pthread_attr_t  attr;
        pthread_attr_init(&attr);
        pthread_t tid;
        pthread_create(&tid, 
		    &attr, 
		    entrypoint, 
		    this);
    }

    void process()
    {
        sem_post(&go);
    }
    
    void wait()
    {
        sem_wait(&done);
    }

    static void* entrypoint(void *ptr)
    {
        Engine *engine = (Engine*)ptr;
        engine->run();
        return 0;
    }
    
    void run()
    {
        while(1)
        {
            sem_wait(&go);

            uint8_t *input_tensor = interpreter->typed_tensor<uint8_t>(input);
            boxes.clear();
            scores.clear();


// crop only
            for(int i = 0; i < TILE_H; i++)
            {
                memcpy(input_tensor + i * TILE_W * 3,
                    preview[current_input].ptr(0) + 
                        i * PREVIEW_W * 3 + 
                        tile_x[tile] * 3,
                    TILE_W * 3);
            }



// save the tile
// if(tile == 1)
// {
// printf("Engine::run %d tile_x=%d\n", __LINE__, tile_x[tile]);
// static int counter = 0;
// Mat temp(input_h, input_w, CV_8UC3);
// memcpy(temp.ptr(0), input_tensor, input_w * input_h * 3);
// std::vector<uchar> buf;
// char string[TEXTLEN];
// sprintf(string, "test%06d.jpg", counter++);
// imwrite(string, temp);
// printf("Engine::run %d wrote %s\n", __LINE__, string);
// }

    // comment this out to not do any detection
            interpreter->Invoke();


            std::vector<std::pair<float, int>> top_results;
            get_top_n<float>(interpreter->typed_tensor<float>(output_score_layer),
                output_size, 
                MAX_HITS, // total results
                THRESHOLD, // threshold
                &top_results);



            for(const auto& result : top_results)
            {
    // get the location box
                float score = result.first;
                int index = result.second;
                float *box = interpreter->typed_tensor<float>(output_location_layer) + 
                    index * 4;

                scores.push_back(score);
                boxes.push_back((int)(box[1] * TILE_W) + tile_x[tile]);
                boxes.push_back((int)(box[0] * TILE_H));
                boxes.push_back((int)((box[3] - box[1]) * TILE_W));
                boxes.push_back((int)((box[2] - box[0]) * TILE_H));

//                 printf("main %d score=%f index=%d %f %f %f %f\n", 
//                     __LINE__, 
//                     score, 
//                     index,
//                     box[0],
//                     box[1],
//                     box[2],
//                     box[3]);
            }


            sem_post(&done);
        }
    }
};



float calculate_error(float current_x, float want_x)
{
    return current_x - want_x;
}

Engine engines[TILES];

void get_hist(float *dst, int x, int y, int w, int h)
{
// easier to do it from scratch than massage the opencv histogram function
    bzero(dst, sizeof(float) * BINS * 3);
    int temp[BINS * 3];
    bzero(temp, sizeof(int) * BINS * 3);

    if(x < 0)
    {
        w += x;
        x = 0;
    }

    if(y < 0)
    {
        h += y;
        y = 0;
    }

    int rows = preview[current_input].rows;
    int cols = preview[current_input].cols;
    if(x + w > cols)
    {
        w -= x + w - cols;
    }

    if(y + h > rows)
    {
        h -= y + h - rows;
    }

    if(w <= 0 || h <= 0) return;

    for(int i = y; i < y + h; i++)
    {
        uint8_t *row = preview[current_input].ptr(0) +
            cols * 3 * i + x * 3;
        for(int j = 0; j < w; j++)
        {
            int bin = row[0] * BINS / 256;
            temp[bin]++;
            bin = row[1] * BINS / 256;
            temp[BINS + bin]++;
            bin = row[2] * BINS / 256;
            temp[BINS * 2 + bin]++;
            row += 3;
        }
    }

// normalize
    int max = 0;
    for(int i = 0; i < BINS * 3; i++)
        max = MAX(temp[i], max);
    if(max < 1) max = 1;
    for(int i = 0; i < BINS * 3; i++)
        dst[i] = (float)temp[i] / max;
}

void blend_hist(float *old_hist, float *new_hist, float weight)
{
    for(int i = 0; i < BINS * 3; i++)
    {
        old_hist[i] = old_hist[i] * (1.0 - weight) + new_hist[i] * weight;
    }
}

float diff_hist(float *want, float *unknown)
{
    float total = 0;
    for(int i = 0; i < BINS * 3; i++)
    {
        total += SQR(unknown[i] - want[i]);
    }
    return sqrt(total);
}

void print_fixed(float n)
{
    char string[TEXTLEN];
    char string2[TEXTLEN];
    sprintf(string, "%.2f", n);
    int spaces = 16 - strlen(string);
    if(spaces < 1) spaces = 1;
//printf("print_fixed %d spaces=%d\n", __LINE__, spaces);
    for(int i = 0; i < spaces; i++)
    {
        string2[i] = ' ';
    }
    strcpy(string2 + spaces, string);
    printf("%s", string2);
}

void dump_hist(float *hist, const char *title)
{
    printf("%s:\n", title);
    for(int i = 0; i < BINS; i++)
    {
        printf("\t");
        print_fixed(hist[i]);
        print_fixed(hist[BINS + i]);
        print_fixed(hist[BINS * 2 + i]);
        printf("\n");
    }
}


void process_truck()
{
    for(int tile = 0; tile < TILES; tile++)
    {
        engines[tile].process();
    }
    for(int tile = 0; tile < TILES; tile++)
    {
        engines[tile].wait();
    }

// search for biggest one
//     int best_index = -1;
//     int best_tile = -1;
//     int max_size = 0;
//     for(int tile = 0; tile < TILES; tile++)
//     {
//         for(int i = 0; i < engines[tile].boxes.size() / 4; i++)
//         {
//             int w = engines[tile].boxes.at(i * 4 + 2);
//             int h = engines[tile].boxes.at(i * 4 + 3);
//             int area = w * h;
//             if(area > max_size)
//             {
//                 max_size = area;
//                 best_index = i;
//                 best_tile = tile;
//             }
//         }
//     }

// delete the overlapping boxes
    boxes.clear();
    nearest_box = -1;
    for(int tile = 0; tile < TILES; tile++)
    {
        for(int src = 0; src < engines[tile].boxes.size() / 4; src++)
        {
            int src_x = engines[tile].boxes.at(src * 4 + 0);
            int src_y = engines[tile].boxes.at(src * 4 + 1);
            int src_w = engines[tile].boxes.at(src * 4 + 2);
            int src_h = engines[tile].boxes.at(src * 4 + 3);
            float score = engines[tile].scores.at(src);
            int got_it = 0;

// find existing overlapping box
            for(int i = 0; i < boxes.size() && !got_it; i++)
            {
                Box *dst = &boxes[i];

// keep overlapping boxes in the same tile
                if(dst->tile != tile)
                {
                    if(src_x < dst->x + dst->w &&
                        src_x + src_w > dst->x &&
                        src_y < dst->y + dst->h &&
                        src_y + src_h > dst->y)
                    {
// they overlap
                        int src_area = src_w * src_h;
                        int dst_area = dst->w * dst->h;
// replace with the bigger one
                        if(src_area > dst_area)
                        {
                            dst->x = src_x;
                            dst->y = src_y;
                            dst->w = src_w;
                            dst->h = src_h;
                            dst->tile = tile;
                            dst->score = score;
                        }

                        got_it = 1;
                    }
                }
            }

            if(!got_it)
            {
                Box dst;
                dst.x = src_x;
                dst.y = src_y;
                dst.w = src_w;
                dst.h = src_h;
                dst.tile = tile;
                dst.score = score;
                boxes.push_back(dst);
            }
        }
    }

// recompute the goal color if 1 hit
    if(boxes.size() == 1)
    {
        Box *dst = &boxes[0];
        get_hist(dst->hist, dst->x, dst->y, dst->w, dst->h);



// defeat the case of the new target color coming from the wrong animal
// with a moving average
        blend_hist(want_hist, dst->hist, WEIGHT);

//dump_hist(dst->hist, "new_hist");
//dump_hist(want_hist, "want_hist");

// distance between self for testing
        dst->distance = diff_hist(want_hist, dst->hist);
//printf("main %d: distance=%f\n", __LINE__, dst->distance);
        nearest_box = 0;
    }
    else
    if(boxes.size() > 1)
    {
// get the color separation in each box
        float nearest_distance;
        for(int i = 0; i < boxes.size(); i++)
        {
            Box *box = &boxes[i];
            get_hist(box->hist, box->x, box->y, box->w, box->h);
            box->distance = diff_hist(want_hist, box->hist);
            if(i == 0 || box->distance < nearest_distance)
            {
                nearest_box = i;
                nearest_distance = box->distance;
            }
        }

// update the target color with the closest match in the group
        Box *box = &boxes[nearest_box];
        blend_hist(want_hist, box->hist, WEIGHT);
//dump_hist(box->hist, "nearest_hist");
//dump_hist(want_hist, "want_hist");
//printf("main %d: nearest_box=%d distance=%f\n", __LINE__, nearest_box, nearest_distance);
    }


// generate servo command
    if(nearest_box >= 0)
    {
        Box *box = &boxes[nearest_box];
        float face_x = (float)(box->x + box->w / 2) / PREVIEW_W;
        float face_y = (float)box->y / PREVIEW_H;

        float center_x = 0.5;
        if(current_operation == TRACKING)
        {
            int x_error = (int)(calculate_error(face_x, center_x) * 100);
            if(x_error >= deadband)
            {
                adc = MIN_LEFT +
                    (MAX_LEFT - MIN_LEFT) *
                    (x_error - deadband) *
                    speed / 
                    100 /
                    100;
                CLAMP(adc, 0, 255);
            }
            else
            if(x_error < -deadband)
            {
                adc = MIN_RIGHT - 
                    (MIN_RIGHT - MAX_RIGHT) *
                    (-deadband - x_error) * 
                    speed / 
                    100 /
                    100;
                CLAMP(adc, 0, 255);
            }
            else
            {
// stop servo
                adc = ADC_CENTER;
            }
            write_servos(0);
        }
    }
}



int send_truck(int offset)
{
    int total = boxes.size();
    if(total > 255) total = 255;

    vijeo_buffer[offset++] = total;
    for(int i = 0; i < total; i++)
    {
        Box *box = &boxes[i];
        vijeo_buffer[offset++] = (box->x & 0xff);
        vijeo_buffer[offset++] = (box->x >> 8) & 0xff;
        vijeo_buffer[offset++] = (box->y & 0xff);
        vijeo_buffer[offset++] = (box->y >> 8) & 0xff;
        vijeo_buffer[offset++] = (box->w & 0xff);
        vijeo_buffer[offset++] = (box->w >> 8) & 0xff;
        vijeo_buffer[offset++] = (box->h & 0xff);
        vijeo_buffer[offset++] = (box->h >> 8) & 0xff;

// info text
//            const char *name = "LION";
        char string[TEXTLEN];
//            sprintf(string, "%.2f %d", box->score, box->tile);
        sprintf(string, "%.2f %.2f", 
            box->score, 
            box->distance);
//        sprintf(string, "%.2f", 
//            box->distance);
        int len = strlen(string);
        memcpy(vijeo_buffer + offset, string, len);
        offset += len;
// null terminate
        vijeo_buffer[offset++] = 0;
    }

    vijeo_buffer[offset++] = nearest_box;


    return offset;
}


void init_defish()
{
    x_lookup = new float[DEFISH_W * DEFISH_H];
    y_lookup = new float[DEFISH_W * DEFISH_H];

    for(int i = 0; i < DEFISH_THREADS; i++)
        defishers[i].init(i);

// init defishing table
// parameters from Cinelerra
    double radius = 1.0;
    double fov = 0.65;
    double aspect = 1.0;
    double center_x = RAW_W * 49 / 100;
    double center_y = RAW_H * 50 / 100;

    double x_factor = aspect;
    double y_factor = 1.0 / aspect;
    double max_z = hypot(RAW_W, RAW_H) / 2;
    double r = max_z / M_PI / (fov / 2);

    for(int y_out = 0; y_out < DEFISH_H; y_out++)
    {
        for(int x_out = 0; x_out < DEFISH_W; x_out++)
        {
            double x_in = x_out * RAW_H / DEFISH_H + 150;
            double y_in = y_out * RAW_H / DEFISH_H;
            double x_diff = x_in - center_x;
            double y_diff = y_in - center_y;
            double z = hypot(x_diff, y_diff);
            double angle = atan2(y_diff, x_diff);
            double radius1 = (z / r) * 2 * radius;
            double z_in = r * atan(radius1) / (M_PI / 2);
            x_in = z_in * cos(angle) * x_factor + center_x;
            y_in = z_in * sin(angle) * y_factor + center_y;
#ifndef SMOOTH_DEFISH
            CLAMP(x_in, 0, RAW_W - 1);
            CLAMP(y_in, 0, RAW_H - 1);
            x_lookup[y_out * DEFISH_W + x_out] = (int)x_in;
            y_lookup[y_out * DEFISH_W + x_out] = (int)y_in;
#else
            CLAMP(x_in, 0, RAW_W - 2);
            CLAMP(y_in, 0, RAW_H - 2);
            x_lookup[y_out * DEFISH_W + x_out] = x_in;
            y_lookup[y_out * DEFISH_W + x_out] = y_in;
#endif
//printf("%d %d -> %d %d\n", (int)x_in, (int)y_in, x, y);
        }
    }
}

void do_defish()
{
// defish
    defished_y.create(DEFISH_H, DEFISH_W, CV_8UC1);
    defished_u.create(DEFISH_H, DEFISH_W, CV_8UC1);
    defished_v.create(DEFISH_H, DEFISH_W, CV_8UC1);

    for(int i = 0; i < DEFISH_THREADS; i++)
        defishers[i].process();
    for(int i = 0; i < DEFISH_THREADS; i++)
        defishers[i].wait();
}

void handle_capture()
{
    if(do_capture)
    {
        if(!ffmpeg_writer)
        {
            char string[TEXTLEN];
            uuid_t temp_id;
            uuid_generate(temp_id);
            sprintf(string, 
//                        "ffmpeg -y -f rawvideo -y -pix_fmt bgr24 -r 10 -s:v %dx%d -i - -c:v h264 -crf 30 -pix_fmt yuvj420p -an ",
//                "ffmpeg -y -f rawvideo -y -pix_fmt rgb24 -r 10 -s:v %dx%d -i - -c:v mpeg4 -qscale:v 5 -pix_fmt yuvj420p -an ",
                "ffmpeg -y -f rawvideo -y -pix_fmt yuv420p -r 10 -s:v %dx%d -i - -c:v mpeg4 -qscale:v 5 -pix_fmt yuvj420p -an ",
// write the raw images from the camera
                RAW_W,
                RAW_H);
// write the defished image
//                        DEFISH_W,
//                        DEFISH_H);
            uuid_unparse(temp_id, string + strlen(string));
            strcat(string, ".mp4");
            printf("main %d: running %s\n", __LINE__, string);
            ffmpeg_writer = popen(string, "w");
            frames_captured = 0;
        }

        if(ffmpeg_writer)
        {
// write the raw images from the camera
            fwrite(raw_y, 
                1, 
                RAW_W * RAW_H, 
                ffmpeg_writer);
            fwrite(raw_u, 
                1, 
                RAW_W * RAW_H / 4, 
                ffmpeg_writer);
            fwrite(raw_v, 
                1, 
                RAW_W * RAW_H / 4, 
                ffmpeg_writer);
            frames_captured++;
        }
        else
        {
            printf("main %d: Error running ffmpeg\n", __LINE__);
            do_capture = 0;
            sleep(1);
        }
    }
    else
    if(ffmpeg_writer != 0)
    {
        pclose(ffmpeg_writer);
        ffmpeg_writer = 0;
        int _ = system("sync");
        printf("main %d: stopped recording\n", __LINE__);
    }
}


// get the average color in the box
// void get_color(float *dst, int x, int y, int w, int h)
// {
//     double accum_r = 0;
//     double accum_g = 0;
//     double accum_b = 0;
//     int total = 0;
//     for(int i = y; i < y + h; i++)
//     {
//         if(i >= 0 && i < PREVIEW_H)
//         {
//             uint8_t *row = preview[current_input].ptr(0) +
//                 PREVIEW_W * 3 * i + x * 3;
//             for(int j = x; j < x + w; j++)
//             {
//                 if(j >= 0 && j < PREVIEW_W)
//                 {
//                     accum_r += row[0];
//                     accum_g += row[1];
//                     accum_b += row[2];
//                     total++;
//                 }
//                 row += 3;
//             }
//         }
//     }
//     
//     dst[0] = accum_r / total;
//     dst[1] = accum_g / total;
//     dst[2] = accum_b / total;
// }

int main(int argc, char** argv)
{
// reset the console for most signals
// ignoresend_packet signals from the child process
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
    signal(SIGPIPE, ignore_);
    signal(SIGALRM, quit);
    signal(SIGTERM, quit);
    signal(SIGCHLD, ignore_);

    INIT_PROFILE

    load_defaults();
    dump_settings();
    libusb_init(0);

    printf("main %d loading %s\n", __LINE__, MODEL);
    for(int i = 0; i < TILES; i++)
        engines[i].init(MODEL, i);

// must set this after the input size is known
    tile_x[0] = 0;
    tile_x[1] = PREVIEW_W - TILE_W;

// need to round up to a multiple of the MCU height
    int h_round = MCU_H * (RAW_H / MCU_H);
    if((RAW_H % MCU_H) > 0) h_round += MCU_H;
    raw_y = new uint8_t[RAW_W * h_round];
    raw_u = new uint8_t[(RAW_W / 2) * (h_round / 2)];
    raw_v = new uint8_t[(RAW_W / 2) * (h_round / 2)];

#ifndef READ_TEST_VIDEO
    init_hdmi();
#endif

    init_server();
    init_servos();
    init_yuv();
    init_defish();



//     while(1)
//     {
//         send_servo(0xaa);
//         usleep(100000);
//     }

// compressed data from V4L2
    Mat rawData;
    float fps = 0;
    int fps_frames = 0;
    struct timespec fps_time1;
    clock_gettime(CLOCK_MONOTONIC, &fps_time1);


// mane loop
    while(1)
    {
        int got_it = 0;


// DEBUG load test images
#ifdef READ_TEST_VIDEO
        static FILE *ffmpeg_reader = 0;
        if(!ffmpeg_reader)
        {
            char string[TEXTLEN];
            sprintf(string,
                "ffmpeg -i %s -f rawvideo -pix_fmt yuv420p -",
                TEST_VIDEO_PATH);
            printf("main %d: running %s\n", __LINE__, string);
            ffmpeg_reader = popen(string, "r");
            if(!ffmpeg_reader)
            {
                printf("main %d: %s\n", __LINE__, strerror(errno));
                sleep(1); 
            }
        }
        
        int result = fread(raw_y, 1, RAW_W * RAW_H, ffmpeg_reader);
        result |= fread(raw_u, 1, RAW_W * RAW_H / 4, ffmpeg_reader);
        result |= fread(raw_v, 1, RAW_W * RAW_H / 4, ffmpeg_reader);
        if(result <= 0) 
        {
            printf("main %d: decoder quit\n", __LINE__);
            return 0;
        }

// raw_yuv -> defished_yuv
        do_defish();
        got_it = 1;
#endif // READ_TEST_VIDEO

#ifdef READ_TEST_IMAGE
        if(raw_image.cols <= 0 && raw_image.rows <= 0)
        {
            raw_image = imread("test2.jpg");
        }
        got_it = 1;
#endif

        if(!got_it)
        {
// injest the next frame from the camera
            sem_wait(&frame_ready_sema);
            pthread_mutex_lock(&frame_lock);
            if(got_image)
            {
                got_image = 0;
                got_it = 1;
                handle_capture();

// raw_yuv -> defished_yuv
                do_defish();
            }
            pthread_mutex_unlock(&frame_lock);

        } // !got_it

        if(got_it)
        {

            Size newSize(PREVIEW_W, PREVIEW_H);
// scale for preview/efficientlion
            preview_y.create(PREVIEW_H, PREVIEW_W, CV_8UC1);
            preview_u.create(PREVIEW_H, PREVIEW_W, CV_8UC1);
            preview_v.create(PREVIEW_H, PREVIEW_W, CV_8UC1);
            preview[current_input].create(PREVIEW_H, PREVIEW_W, CV_8UC3);
// opencv resizer
            resize(defished_y, preview_y, newSize);
            resize(defished_u, preview_u, newSize);
            resize(defished_v, preview_v, newSize);

            for(int i = 0; i < PREVIEW_H; i++)
            {
                uint8_t *dst = preview[current_input].ptr(0) + 
                    i * PREVIEW_W * 3;
                uint8_t *in_y = preview_y.ptr(0) + i * PREVIEW_W;
                uint8_t *in_u = preview_u.ptr(0) + i * PREVIEW_W;
                uint8_t *in_v = preview_v.ptr(0) + i * PREVIEW_W;
                for(int j = 0; j < PREVIEW_W; j++)
                {
                    int y = *in_y++;
                    y |= (y << 16) | (y << 8);
                    int u = *in_u++;
                    int v = *in_v++;
                    int r, g, b;
                    YUV_TO_RGB(y, u, v, r, g, b);
                    *dst++ = r;
                    *dst++ = g;
                    *dst++ = b;
                }
            }

#ifdef ENABLE_TRACKER
            process_truck();
#endif // ENABLE_TRACKER

            fps_frames++;
            struct timespec fps_time2;
            clock_gettime(CLOCK_MONOTONIC, &fps_time2);
            delta = 
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
            if(current_input2 < 0)
            {

// pack the metadata
                int offset = HEADER_SIZE;
                vijeo_buffer[offset++] = (int)(fps * 256) & 0xff;
                vijeo_buffer[offset++] = (int)fps;
// pack the mode
                vijeo_buffer[offset++] = 1;



// store the hit boxes
                offset = send_truck(offset);

                send_vijeo(current_input, offset);
                send_status();
                current_input = !current_input;
            }
//            UPDATE_PROFILE
//            printf("main %d delta=%f\n", __LINE__, delta);
        }
    }

    return 0;
}








