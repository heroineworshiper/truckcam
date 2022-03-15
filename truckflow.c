/*
 * Tracking camera using tensorflow
 *
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



// based on opencv-4.x/samples/dnn/face_detect.cpp
// this runs on the odroid to capture vijeo & send servo commands
// This requires ffmpeg out of an original hope H264 could be low latency
// but only JPEG is low enough.

// make truckflow



#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

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
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>
#include <linux/videodev2.h>
#include <queue>

#include "truckflow.h"

extern "C"
{
#include <uuid/uuid.h>
}

using namespace cv;
using namespace std;


#ifdef USE_PI
#include <linux/spi/spidev.h>
#endif

#define MODEL "efficientlion0.yolo.300.tflite"
//#define MODEL "efficientlion0.yolo.100.tflite"
//#define MODEL "efficientlion1.yolo.tflite"
//#define MODEL "efficientlion0.tflite.1000.300epoch"
#define MAX_HITS 4

// crop the video to the aspect ratio of the input layer
#define CROP

// capture raw video for offline training when tracking
//#define CAPTURE

// score required to get a hit
#define THRESHOLD 0.5

// ADC values from arm_cam.c
// deadband
#define MIN_LEFT 130
#define MIN_RIGHT 126
// maximums
#define MAX_LEFT 240
#define MAX_RIGHT 20

#define HDMI_BUFFERS 2

#define BUFSIZE2 0x400000
// compressed frame from the hardware
unsigned char reader_buffer3[BUFSIZE2];
// size of the frame in reader_buffer3
int frame_size;
static pthread_mutex_t frame_lock;
static sem_t frame_ready_sema;
int hdmi_fd = -1;
char current_path[TEXTLEN] = { 0 };

// current position of cropped section, derived from the crop state
int window_x = 0;
// previous position of cropped section
int prev_window_x = 0;

#ifdef CROP

    #define CROP_W 720
    #define CROP_H 720
    // size of preview for phone
    #define PREVIEW_W 360
    #define PREVIEW_H 360

    #define SCANNING0 0
    #define SCANNING1 1
    #define SCANNING2 2
    #define LOCKED 4
    int crop_state = SCANNING0;
    int crop_counter = 0;
    #define CROP_TIMEOUT 0

#else

    // size of preview for phone
    #define PREVIEW_W 640
    #define PREVIEW_H 360

#endif

// input size for the neural network
int input_w;
int input_h;

int servo_fd = -1;
static pthread_mutex_t servo_lock;
static sem_t servo_ready_sema;
uint8_t servo_command = 0;

uint8_t error_flags = 0xff;
FILE *ffmpeg_fd = 0;

int current_operation = IDLE;
int deadband = 5;
int speed = 100;
// show the alert for this amount of time
struct timespec settings_time = { 0 };

// center of lion in previous frame
int prev_x = -1;
int prev_y = -1;
int prev_w;
int prev_h;

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


// probe for the video device
int open_hdmi(int verbose)
{
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

    int fd2 = -1;
    for(int i = 0; i < paths.size(); i++)
    {
        fd2 = open(paths.at(i), O_RDWR);
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
            printf("hdmi_thread %d: %s format=%c%c%c%c w=%d h=%d\n",
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
                    printf("hdmi_thread %d id=%x min=%d max=%d step=%d default=%d %s\n", 
                        __LINE__, 
                        arg.id, 
                        arg.minimum,
                        arg.maximum,
                        arg.step,
                        arg.default_value,
                        arg.name);
                }
            }

// backlight compensation
            struct v4l2_control ctrl_arg;
            ctrl_arg.id = 0x98091c;
            if(ioctl(fd2, VIDIOC_G_CTRL, &ctrl_arg))
            {
                printf("open_hdmi %d VIDIOC_G_CTRL failed\n", __LINE__);
            }
            else
            {
                printf("hdmi_thread %d got backlight compensation %d\n",
                    __LINE__,
                    ctrl_arg.value);
            }

            ctrl_arg.id = 0x98091c;
            ctrl_arg.value = 0;
            if(ioctl(fd2, VIDIOC_S_CTRL, &ctrl_arg) < 0)
            {
                printf("hdmi_thread %d VIDIOC_S_CTRL failed\n",
                    __LINE__);

// skip the device
                close(fd2);
                fd2 = -1;
            }
            else
            {
                printf("hdmi_thread %d setting backlight compensation to %d\n",
                    __LINE__,
                    ctrl_arg.value);
            }
        }

//         if(fd2 >= 0)
//         {
// // brightness
//             struct v4l2_control ctrl_arg;
//             ctrl_arg.id = 0x980900;
//             ctrl_arg.value = 255;
//             if(ioctl(fd2, VIDIOC_S_CTRL, &ctrl_arg) < 0)
//             {
//                 printf("hdmi_thread %d VIDIOC_S_CTRL failed\n",
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
//                 printf("hdmi_thread %d VIDIOC_S_CTRL failed\n",
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
//                 printf("hdmi_thread %d VIDIOC_S_CTRL failed\n",
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
                printf("hdmi_thread %d VIDIOC_G_CTRL failed\n",
                    __LINE__);
            }
            else
            {
                printf("hdmi_thread %d got saturation %d\n",
                    __LINE__,
                    ctrl_arg.value);
            }


            ctrl_arg.value = 127;
            if(ioctl(fd2, VIDIOC_S_CTRL, &ctrl_arg) < 0)
            {
                printf("hdmi_thread %d VIDIOC_S_CTRL failed\n",
                    __LINE__);
            }
            else
            {
                printf("hdmi_thread %d setting saturation to %d\n",
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

// codec only works for HDMI
#ifdef RAW_HDMI
            v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#else
            v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
#endif
            if(ioctl(fd2, VIDIOC_S_FMT, &v4l2_params) < 0)
            {
//              printf("hdmi_thread %d: VIDIOC_S_FMT failed\n",
//                  __LINE__);
            }


            if(verbose)
            {
                printf("open_hdmi %d opened %s\n", __LINE__, paths.at(i));
                strcpy(current_path, paths.at(i));
            }




            break;
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
    struct timespec fps_time1;
    gettimeofday(&time1, 0);

    int verbose = 1;
    unsigned char *mmap_buffer[HDMI_BUFFERS];
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
            else
            {
    //          printf("hdmi_thread %d: opened %s\n",
    //              __LINE__,
    //              string);


    //          struct v4l2_jpegcompression jpeg_opts;
    //          if(ioctl(fd2, VIDIOC_G_JPEGCOMP, &jpeg_opts) < 0)
    //          {
    //              printf("hdmi_thread %d: VIDIOC_G_JPEGCOMP failed\n",
    //                  __LINE__);
    //          }
    //          printf("hdmi_thread %d: quality=%d\n",
    //              __LINE__,
    //              jpeg_opts.quality);
    //          
    //          if(ioctl(fd2, VIDIOC_S_JPEGCOMP, &jpeg_opts) < 0)
    //          {
    //              printf("hdmi_thread %d: VIDIOC_S_JPEGCOMP failed\n",
    //                  __LINE__);
    //          }

                struct v4l2_requestbuffers requestbuffers;
                requestbuffers.count = HDMI_BUFFERS;
                requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                requestbuffers.memory = V4L2_MEMORY_MMAP;
                if(ioctl(hdmi_fd, VIDIOC_REQBUFS, &requestbuffers) < 0)
                {
                    printf("hdmi_thread %d: VIDIOC_REQBUFS failed\n",
                        __LINE__);
                }
                else
                {
                    for(int i = 0; i < HDMI_BUFFERS; i++)
                    {
                        struct v4l2_buffer buffer;
                        buffer.type = requestbuffers.type;
                        buffer.index = i;

                        if(ioctl(hdmi_fd, VIDIOC_QUERYBUF, &buffer) < 0)
				        {
					        printf("hdmi_thread %d: VIDIOC_QUERYBUF failed\n",
                                __LINE__);
				        }
                        else
                        {
                            mmap_buffer[i] = (unsigned char*)mmap(NULL,
					            buffer.length,
					            PROT_READ | PROT_WRITE,
					            MAP_SHARED,
					            hdmi_fd,
					            buffer.m.offset);
    //                             printf("hdmi_thread %d: allocated buffer size=%d\n",
    //                                 __LINE__,
    //                                 buffer.length);
                            if(ioctl(hdmi_fd, VIDIOC_QBUF, &buffer) < 0)
                            {
                                printf("hdmi_thread %d: VIDIOC_QBUF failed\n",
                                    __LINE__);
                            }
                        }
                    }
                }

                int streamon_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	            if(ioctl(hdmi_fd, VIDIOC_STREAMON, &streamon_arg) < 0)
                {
		            printf("hdmi_thread %d: VIDIOC_STREAMON failed\n",
                        __LINE__);
                }
                clock_gettime(CLOCK_MONOTONIC, &fps_time1);
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
    // send it to openpose
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




// reset saturation
                    struct v4l2_control ctrl_arg;
                    ctrl_arg.id = 0x980902;
                    ctrl_arg.value = 127;
                    ioctl(hdmi_fd, VIDIOC_S_CTRL, &ctrl_arg);
// reset backlight compensation
                    ctrl_arg.id = 0x98091c;
                    ctrl_arg.value = 0;
                    ioctl(hdmi_fd, VIDIOC_S_CTRL, &ctrl_arg);







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
// 	pthread_create(&x, 
// 		0, 
// 		hdmi_watchdog, 
// 		0);
}

void send_servo(uint8_t command)
{

    servo_command = command;
    sem_post(&servo_ready_sema);
}

void* servo_thread(void *ptr)
{
    while(1)
    {
        sem_wait(&servo_ready_sema);

#ifdef USE_PI
        struct spi_ioc_transfer xfer;
        uint8_t txbuf[8];
        uint8_t rxbuf[8];
        xfer.tx_buf = (__u64)&txbuf;
        xfer.rx_buf = (__u64)&rxbuf;
        xfer.len = 1;
        xfer.speed_hz = 9600;
        xfer.delay_usecs = 0;
        xfer.bits_per_word = 8;
        xfer.cs_change = 0;
        xfer.tx_nbits = 8;
        xfer.rx_nbits = 8;
        xfer.word_delay_usecs = 0;
        xfer.pad = 0;
        txbuf[0] = servo_command;

        if(ioctl(servo_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
        {
		    printf("main %d: couldn't send SPI\n", __LINE__);
        }
#endif
    }
}

void init_servo()
{
#ifdef USE_PI
	if ((servo_fd = open("/dev/spidev0.0", O_RDWR, 0)) == -1) 
    {
		printf("main %d: couldn't open SPI\n", __LINE__);
		return;
	}
#endif


	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&servo_lock, &attr);
    sem_init(&servo_ready_sema, 0, 0);

    pthread_t x;
	pthread_create(&x, 
		0, 
		servo_thread, 
		0);
}


void visualize(Mat& input, 
    float *scores, 
    Mat& lions, 
    double fps, 
    int best_index)
{
    int thickness = 2;
    float text_size = 1.5;
    struct timespec settings_time2;
    clock_gettime(CLOCK_MONOTONIC, &settings_time2);
    double delta = 
        (double)((settings_time2.tv_sec * 1000 + settings_time2.tv_nsec / 1000000) -
        (settings_time.tv_sec * 1000 + settings_time.tv_nsec / 1000000)) / 1000;

    if(delta < 3)
    {
        putText(input, 
            "SETTINGS UPDATED", 
            Point(input.cols / 2, input.rows / 2), 
            FONT_HERSHEY_SIMPLEX, 
            text_size, 
            Scalar(0, 255, 0), 
            2);
    }

    for (int i = 0; i < lions.rows; i++)
    {
// Draw bounding box
        auto color = Scalar(0, 255, 0);
        if(i == best_index)
        {
            color = Scalar(0, 0, 255);
        }
        rectangle(input, 
            Rect2i(lions.at<int>(i, 0), // x
                lions.at<int>(i, 1),  // y
                lions.at<int>(i, 2),  // w
                lions.at<int>(i, 3)), // h
            color, thickness);

// draw score
#ifndef USE_SIZE  
        std::string scoreString = cv::format("%.2f %s", 
            scores[i],
            best_index == i ? "REAL LION" : "FAKE LION");
        putText(input, 
            scoreString, 
            Point(lions.at<int>(i, 0), int(lions.at<int>(i, 1)) - 2), 
            FONT_HERSHEY_SIMPLEX, 
            text_size, 
            Scalar(0, 255, 0), 
            2);
#endif // !USE_SIZE
    }

    std::string fpsString = cv::format("%.2f FPS", (float)fps);
    putText(input, 
        fpsString, 
        Point(0, 40), 
        FONT_HERSHEY_SIMPLEX, 
        text_size, 
        Scalar(0, 255, 0), 
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

void ignore_(int sig)
{
    printf("ignore %d sig=%s\n", __LINE__, signal_titles[sig]);
}


void load_defaults()
{
    char *home = getenv("HOME");
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

int main(int argc, char** argv)
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
    signal(SIGPIPE, ignore_);
    signal(SIGALRM, quit);
    signal(SIGTERM, quit);
    signal(SIGCHLD, ignore_);

    INIT_PROFILE

    load_defaults();
    dump_settings();

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
// printf("main %d output layers=%d location type=%d size=%d\n", 
// __LINE__, 
// outputs.size(),
// interpreter->tensor(output_location_layer)->type,
// location_dims->data[2]);

// for(int i = 0; i < outputs.size(); i++)
// {
//     printf("main %d output %d layer=%d\n", 
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


// DEBUG disable to load test image
    init_hdmi();
    init_server();
    init_servo();
    
//     while(1)
//     {
//         send_servo(0xaa);
//         usleep(100000);
//     }

    int frame_size2;
    Mat rawData;
    Mat raw_image;
    Mat preview_image;
    Mat cropped_image;
    float fps = 0;
    int fps_frames = 0;
    struct timespec fps_time1;
    clock_gettime(CLOCK_MONOTONIC, &fps_time1);


// mane loop
    while(1)
    {
        int got_it = 0;


// DEBUG load test image
//         if(raw_image.cols <= 0 && raw_image.rows <= 0)
//         {
//             raw_image = imread("test.jpg");
//         }
//         got_it = 1;

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
            raw_image = imdecode(rawData, cv::IMREAD_COLOR);
            if(raw_image.cols > 0 && raw_image.rows > 0)
            {
                got_it = 1;
            }
        }

// 37ms
        UPDATE_PROFILE
//        printf("main %d delta=%f\n", __LINE__, delta);

        if(got_it)
        {
            uint8_t *input_tensor = interpreter->typed_tensor<uint8_t>(input);

#ifdef CROP
            prev_window_x = window_x;
            cv::Rect cropping(window_x, 
                0, 
                CROP_W,
                CROP_H);
// just changes the starting pointer & row stride
            Mat cropped = raw_image(cropping);
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
#else

            resize<uint8_t>(input_tensor, 
                raw_image.ptr(0),
                RAW_H,
                RAW_W,
                3,
                input_h,
                input_w,
                3);
#endif

            
// comment this out to not do any detection
            interpreter->Invoke();


            std::vector<std::pair<float, int>> top_results;
            get_top_n<float>(interpreter->typed_tensor<float>(output_score_layer),
              output_size, 
              MAX_HITS, // total results
              THRESHOLD, // threshold
              &top_results);


            
            Mat boxes(top_results.size(), 4, CV_32S);
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
#ifdef CROP
                boxes.at<int>(i, 0) = box[1] * CROP_W;
                boxes.at<int>(i, 1) = box[0] * CROP_H;
                boxes.at<int>(i, 2) = (box[3] - box[1]) * CROP_W;
                boxes.at<int>(i, 3) = (box[2] - box[0]) * CROP_H;
#else
                boxes.at<int>(i, 0) = box[1] * RAW_W;
                boxes.at<int>(i, 1) = box[0] * RAW_H;
                boxes.at<int>(i, 2) = (box[3] - box[1]) * RAW_W;
                boxes.at<int>(i, 3) = (box[2] - box[0]) * RAW_H;
#endif

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

// search for biggest one
            int best_index = -1;
            int max_size = 0;
            for(int i = 0; i < boxes.rows; i++)
            {
                int w = boxes.at<int>(i, 2);
                int h = boxes.at<int>(i, 3);
                int area = w * h;
                if(area > max_size)
                {
                    max_size = area;
                    best_index = i;
                }
            }

// generate servo command
            if(best_index >= 0)
            {
                int x = boxes.at<int>(best_index, 0);
                int y = boxes.at<int>(best_index, 1);
                int w = boxes.at<int>(best_index, 2);
                int h = boxes.at<int>(best_index, 3);


#ifdef CROP
// position of hit in full frame from 0 - 1
                float face_x = (float)(x + w / 2 + window_x) / RAW_W;
                float face_y = (float)y / CROP_H;


// center the window position on the hit
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
#else
                float face_x = (float)(x + w / 2) / RAW_W;
                float face_y = (float)y / RAW_H;
#endif


                prev_x = x + w / 2;
                prev_y = y + h / 2;


                float center_x = 0.5;
                if(current_operation == TRACKING)
                {
                    float deadband2 = (float)deadband / 100;
                    if(face_x >= center_x + deadband2)
                    {
                        float error = face_x - center_x - deadband2;
                        float step = MIN_RIGHT - MIN_RIGHT * 
                            error * 
                            speed / 
                            100;
                        CLAMP(step, 0, 255);
                        send_servo((int)step);
                    }
                    else
                    if(face_x <= center_x - deadband2)
                    {
                        float error = center_x - deadband2 - face_x;
                        float step = MIN_LEFT + (255 - MIN_LEFT) *
                            error *
                            speed / 
                            100;
                        CLAMP(step, 0, 255);
                        send_servo((int)step);
                    }
                    else
                    {
// stop servo
                        send_servo((MIN_LEFT + MIN_RIGHT) / 2);
                    }
                }
            }
            else
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

// 100ms
            UPDATE_PROFILE
//            printf("main %d delta=%f\n", __LINE__, delta);



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
                        RAW_W,
                        RAW_H);
                    uuid_unparse(temp_id, string + strlen(string));
                    strcat(string, ".mp4");
                    printf("main %d: running %s\n", __LINE__, string);
                    ffmpeg_fd = popen(string, "w");
                }

                if(ffmpeg_fd)
                {
                    fwrite(raw_image.ptr(0), 1, RAW_W * RAW_H * 3, ffmpeg_fd);
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


// send frame to phone
            if(server_output >= 0)
            {
// draw the results
#ifdef CROP
                visualize(cropped, scores, boxes, fps, 0);
#else
                visualize(raw_image, scores, boxes, fps, 0);
#endif

#ifdef USE_FFMPEG
// send to ffmpeg
//                 int _ = write(server_output, 
//                     (unsigned char*)preview_image.ptr(0),
//                     PREVIEW_W * PREVIEW_H * 3);
#else // USE_FFMPEG
// compress directly to server FIFO
                std::vector<uchar> buf;
                std::vector<int> compressing_factor;
                compressing_factor.push_back(IMWRITE_JPEG_QUALITY);
                compressing_factor.push_back(50);
#ifdef CROP
                resize(cropped, 
                    preview_image,
                    Size(PREVIEW_W, PREVIEW_H),
                    INTER_LINEAR); // no speed difference
#else
                resize(raw_image, 
                    preview_image,
                    Size(PREVIEW_W, PREVIEW_H),
                    INTER_LINEAR); // no speed difference
#endif
                imencode(".jpg", 
                    preview_image, 
                    buf, 
                    compressing_factor);
// insert window_x at beginning of buffer
                int jpg_size = buf.size();
                buf.resize(jpg_size + 4);
                memmove(&buf[4], &buf[0], jpg_size);
// scale window_x to preview size
                int preview_x = prev_window_x * PREVIEW_W / CROP_W;
                buf[0] = preview_x & 0xff;
                buf[1] = (preview_x >> 8) & 0xff;
                buf[2] = (preview_x >> 16) & 0xff;
                buf[3] = (preview_x >> 24) & 0xff;
                send_vijeo_fifo(&buf[0], buf.size());
#endif // !USE_FFMPEG
            }

// 10ms
            UPDATE_PROFILE
//            printf("main %d delta=%f\n", __LINE__, delta);
        }
    }

    return 0;
}








