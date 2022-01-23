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



// based on opencv-4.x/samples/dnn/face_detect.cpp
// this runs on the odroid to capture vijeo & send servo commands
// This requires ffmpeg out of an original hope H264 could be low latency
// but only JPEG is low enough.




#include <opencv4/opencv2/dnn.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/objdetect.hpp>

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

#include "truckcam.h"

using namespace cv;
using namespace std;


#ifdef USE_PI
#include <linux/spi/spidev.h>
#endif

// ADC values from arm_cam.c
// deadband
#define MIN_LEFT 130
#define MIN_RIGHT 126
// maximums
#define MAX_LEFT 240
#define MAX_RIGHT 20


#define DETECT_MODEL "yunet.onnx"
#define COMPARE_MODEL "face_recognizer_fast.onnx"
#define REF_FACE "lion.jpg"
#define COS_THRESHOLD 0.363
#define SCORE_THRESHOLD 0.9
#define NMS_THRESHOLD 0.3
#define TOP_K 5000

#define HDMI_BUFFERS 2
#define HDMI_TEXT "PRODUCT=1b3f/2202/100"

#define BUFSIZE2 0x400000
// compressed frame from the hardware
unsigned char reader_buffer3[BUFSIZE2];
// size of the frame in reader_buffer3
int frame_size;
static pthread_mutex_t frame_lock;
static sem_t frame_ready_sema;
int hdmi_fd = -1;
char current_path[TEXTLEN] = { 0 };

int servo_fd = -1;
static pthread_mutex_t servo_lock;
static sem_t servo_ready_sema;
uint8_t servo_command = 0;

uint8_t error_flags = 0xff;
FILE *ffmpeg_fd = 0;

int current_operation = IDLE;
int face_position = FACE_CENTER;
int deadband = 0;
int speed = 100;
int xy_radius = 10;
int size_radius = 25;
int color_radius = 25;
struct timespec settings_time = { 0 };

// center of face in previous frame
int prev_x = -1;
int prev_y = -1;
int prev_w;
int prev_h;
int prev_r;
int prev_g;
int prev_b;

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

// select seems to protect against disconnects
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


void visualize(Mat& input, float *scores, Mat& faces, double fps, int best_index)
{
    int thickness = 2;
    struct timespec settings_time2;
    clock_gettime(CLOCK_MONOTONIC, &settings_time2);
    double delta = 
        (double)((settings_time2.tv_sec * 1000 + settings_time2.tv_nsec / 1000000) -
        (settings_time.tv_sec * 1000 + settings_time.tv_nsec / 1000000)) / 1000;

    if(delta < 3)
    {
        putText(input, 
            "SETTINGS UPDATED", 
            Point(SCALED_W / 2, SCALED_H / 2), 
            FONT_HERSHEY_SIMPLEX, 
            0.7, 
            Scalar(0, 255, 0), 
            2);
    }

    std::string fpsString = cv::format("%.2f FPS", (float)fps);
    for (int i = 0; i < faces.rows; i++)
    {
// Draw bounding box
        auto color = Scalar(0, 255, 0);
        if(i == best_index)
        {
            color = Scalar(0, 0, 255);
        }
        rectangle(input, 
            Rect2i(int(faces.at<float>(i, 0)), // x
                int(faces.at<float>(i, 1)),  // y
                int(faces.at<float>(i, 2)),  // w
                int(faces.at<float>(i, 3))), // h
            color, thickness);

// draw score
#ifndef USE_SIZE  
        std::string scoreString = cv::format("%.2f %s", 
            scores[i],
            best_index == i ? "REAL LION" : "FAKE LION");
        putText(input, 
            scoreString, 
            Point(faces.at<float>(i, 0), int(faces.at<float>(i, 1)) - 2), 
            FONT_HERSHEY_SIMPLEX, 
            0.7, 
            Scalar(0, 255, 0), 
            2);
#endif // !USE_SIZE


// Draw landmarks
//         circle(input, Point2i(int(faces.at<float>(i, 4)), int(faces.at<float>(i, 5))), 2, Scalar(255, 0, 0), thickness);
//         circle(input, Point2i(int(faces.at<float>(i, 6)), int(faces.at<float>(i, 7))), 2, Scalar(0, 0, 255), thickness);
//         circle(input, Point2i(int(faces.at<float>(i, 8)), int(faces.at<float>(i, 9))), 2, Scalar(0, 255, 0), thickness);
//         circle(input, Point2i(int(faces.at<float>(i, 10)), int(faces.at<float>(i, 11))), 2, Scalar(255, 0, 255), thickness);
//         circle(input, Point2i(int(faces.at<float>(i, 12)), int(faces.at<float>(i, 13))), 2, Scalar(0, 255, 255), thickness);
    }

    putText(input, 
        fpsString, 
        Point(0, 20), 
        FONT_HERSHEY_SIMPLEX, 
        0.7, 
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
        if(!strcasecmp(key, "FACE_POSITION"))
        {
            face_position = atoi(value);
        }
        else
        if(!strcasecmp(key, "DEADBAND"))
        {
            deadband = atoi(value);
        }
        else
        if(!strcasecmp(key, "SPEED"))
        {
            speed = atoi(value);
        }
        else
        if(!strcasecmp(key, "XY_RADIUS"))
        {
            xy_radius = atoi(value);
        }
        else
        if(!strcasecmp(key, "SIZE_RADIUS"))
        {
            size_radius = atoi(value);
        }
        else
        if(!strcasecmp(key, "COLOR_RADIUS"))
        {
            color_radius = atoi(value);
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

    fprintf(fd, "FACE_POSITION %d\n", (int)face_position);
    fprintf(fd, "DEADBAND %d\n", (int)deadband);
    fprintf(fd, "SPEED %d\n", (int)speed);
    fprintf(fd, "XY_RADIUS %d\n", (int)xy_radius);
    fprintf(fd, "SIZE_RADIUS %d\n", (int)size_radius);
    fprintf(fd, "COLOR_RADIUS %d\n", (int)color_radius);

    fclose(fd);
}

void dump_settings()
{
    printf("dump_settings %d\n", __LINE__);
    printf("FACE_POSITION %d\n", (int)face_position);
    printf("DEADBAND %d\n", (int)deadband);
    printf("SPEED %d\n", (int)speed);
    printf("XY_RADIUS %d\n", (int)xy_radius);
    printf("SIZE_RADIUS %d\n", (int)size_radius);
    printf("COLOR_RADIUS %d\n", (int)color_radius);
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

// set threading for the odroid
    setNumThreads(4);

    Ptr<FaceDetectorYN> detector = FaceDetectorYN::create(DETECT_MODEL, 
        "", 
        Size(320, 320), 
        SCORE_THRESHOLD, 
        NMS_THRESHOLD, 
        TOP_K);
    Ptr<FaceRecognizerSF> recognizer = FaceRecognizerSF::create(COMPARE_MODEL, 
        "");

// Load reference face.  Size makes no difference in speed
#ifndef USE_SIZE
    Mat ref_image = imread(REF_FACE);
    if (ref_image.empty())
    {
        printf("main %d: couldn't load ref image %s\n", __LINE__, REF_FACE);
        return 1;
    }

    detector->setInputSize(ref_image.size());
    Mat ref_faces;
    detector->detect(ref_image, ref_faces);
    if (ref_faces.rows < 1)
    {
        printf("main %d: couldn't find face in ref image %s\n", __LINE__, REF_FACE);
        return 1;
    }

    Mat aligned_ref;
    recognizer->alignCrop(ref_image, ref_faces.row(0), aligned_ref);

    Mat ref_features;
    recognizer->feature(aligned_ref, ref_features);
    ref_features = ref_features.clone();
#endif // !USE_SIZE


    detector->setInputSize(Size(SCALED_W, SCALED_H));


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
    Mat scaled_image;
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
            cv::Rect cropping(CROP_X, 
                CROP_Y, 
                CROP_W,
                CROP_H);
            resize(raw_image(cropping), 
                scaled_image,
                Size(SCALED_W, SCALED_H),
//                INTER_NEAREST);
                INTER_LINEAR); // no speed difference
            Mat faces;
            detector->detect(scaled_image, faces);
//printf("main %d\n", __LINE__);
// 125ms
            UPDATE_PROFILE
//            printf("main %d delta=%f\n", __LINE__, delta);



            float scores[faces.rows] = { 0 };
            int best_index = -1;
            double best_score = -1;
            if(faces.rows > 0)
            {
#ifdef USE_SIZE
// find nearest face to previous frame
                if(prev_x >= 0 && 
                    prev_y >= 0)
                {
                    int distance = -1;
                    int closest_index = -1;
                    for(int i = 0; i < faces.rows; i++)
                    {
                        int w = faces.at<float>(i, 2);
                        int h = faces.at<float>(i, 3);
// new center
                        int x = faces.at<float>(i, 0) + w / 2;
                        int y = faces.at<float>(i, 1) + h / 2;
                        int distance2 = (int)hypot(x - prev_x, y - prev_y);
                        if(distance2 < xy_radius * SCALED_W / 100 &&
                            (closest_index < 0 ||
                            distance2 < distance))
                        {
                            distance = distance2;
                            closest_index = i;
                        }
                    }

//                     if(closest_index >= 0)
//                     {
// // test size
//                         int w = faces.at<float>(closest_index, 2);
//                         int h = faces.at<float>(closest_index, 3);
//                         if(w >= prev_w - prev_w * size_radius / 100 &&
//                             w <= prev_w + prev_w * size_radius / 100 &&
//                             h >= prev_h - prev_h * size_radius / 100 &&
//                             h <= prev_h + prev_h * size_radius / 100)
//                         {
// // test color
//                             double accum_r = 0;
//                             double accum_g = 0;
//                             double accum_b = 0;
// // new top left
//                             int x = faces.at<float>(closest_index, 0);
//                             int y = faces.at<float>(closest_index, 1);
//                             for(int i = 0; i < h; i++)
//                             {
//                                 unsigned char *row = (unsigned char*)scaled_image.ptr(0) + 
//                                     i * SCALED_W * 3 +
//                                     x * 3;
//                                 for(int j = 0; j < w; j++)
//                                 {
//                                     accum_r += *row++;
//                                     accum_g += *row++;
//                                     accum_b += *row++;
//                                 }
//                             }
//                             accum_r /= w * h;
//                             accum_g /= w * h;
//                             accum_b /= w * h;
// 
//                             double color_distance = hypot(accum_r - prev_r,
//                                 accum_g - prev_g);
//                             color_distance = hypot(color_distance, 
//                                 accum_b - prev_b);
//             //                printf("main %d color_distance=%f\n", __LINE__, color_distance);
//                             if(color_distance <= 255 * color_radius / 100)
//                             {
//                                 best_index = closest_index;
//                             }
//                             else
//                             {
// // reset the previous frame
//                                 prev_x = -1;
//                                 prev_y = -1;
//                                 closest_index = -1;
//                                 printf("main %d not within COLOR_RADIUS\n", __LINE__);
//                             }
//                         }
//                         else
//                         {
//                             printf("main %d not within SIZE_RADIUS\n", __LINE__);
//                         }
//                     }
//                     else
//                     {
//                         printf("main %d not within XY_RADIUS\n", __LINE__);
//                     }
                }

// XY test failed.  Search based on largest size
                if(best_index < 0)
                {
                    int max_size = 0;
                    for(int i = 0; i < faces.rows; i++)
                    {
                        int w = faces.at<float>(i, 2);
                        int h = faces.at<float>(i, 3);
                        int area = w * h;
                        if(area > max_size)
                        {
                            max_size = area;
                            best_index = i;
                        }
                    }
                }

#else // USE_SIZE
// compare all the faces to the ref
                for(int i = 0; i < faces.rows; i++)
                {
                    Mat aligned_face;
                    recognizer->alignCrop(scaled_image, 
                        faces.row(i), 
                        aligned_face);
                    Mat features;
                    recognizer->feature(aligned_face, features);
                    double cos_score = recognizer->match(features, 
                        ref_features, 
                        FaceRecognizerSF::DisType::FR_COSINE);
                    scores[i] = cos_score;
//printf("main %d score %d=%f\n", __LINE__, i, cos_score);
                    if(cos_score > best_score)
                    {
                        best_score = cos_score;
                        best_index = i;
                    }
                }

// drive servo
                if(best_score >= COS_THRESHOLD)
                {
                }
                else
                {
                    best_index = -1;
                }
#endif // !USE_SIZE

                if(best_index >= 0)
                {
// update chroma data
                    int x = faces.at<float>(best_index, 0);
                    int y = faces.at<float>(best_index, 1);
                    int w = faces.at<float>(best_index, 2);
                    int h = faces.at<float>(best_index, 3);
//                     double accum_r = 0;
//                     double accum_g = 0;
//                     double accum_b = 0;
//                     for(int i = 0; i < h; i++)
//                     {
//                         unsigned char *row = (unsigned char*)scaled_image.ptr(0) + 
//                             i * SCALED_W * 3 +
//                             x * 3;
//                         for(int j = 0; j < w; j++)
//                         {
//                             accum_r += *row++;
//                             accum_g += *row++;
//                             accum_b += *row++;
//                         }
//                     }
//                     accum_r /= w * h;
//                     accum_g /= w * h;
//                     accum_b /= w * h;
//                     prev_r = accum_r;
//                     prev_g = accum_g;
//                     prev_b = accum_b;
//                     prev_w = w;
//                     prev_h = h;

                    prev_x = x + w / 2;
                    prev_y = y + h / 2;

                    float face_x = (float)x / SCALED_W;
                    float face_y = (float)y / SCALED_H;


                    float center_x = 0.5;
                    switch(face_position)
                    {
                        case FACE_LEFT:
                            center_x = 0.33;
                            break;
                        case FACE_CENTER:
                            center_x = 0.5;
                            break;
                        case FACE_RIGHT:
                            center_x = 0.66;
                            break;
                    }

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
//                printf("main %d fps=%f\n", __LINE__, fps);
            }

// 100ms
            UPDATE_PROFILE
//            printf("main %d delta=%f\n", __LINE__, delta);

// send frame to phone
            if(server_output >= 0)
            {
// draw the results
                visualize(scaled_image, scores, faces, fps, best_index);

#ifdef USE_FFMPEG
// send to ffmpeg
                int _ = write(server_output, 
                    (unsigned char*)scaled_image.ptr(0),
                    SCALED_W * SCALED_H * 3);
#else // USE_FFMPEG
// compress directly to server FIFO
                std::vector<uchar> buf;
                std::vector<int> compressing_factor;
                compressing_factor.push_back(IMWRITE_JPEG_QUALITY);
                compressing_factor.push_back(50);
                imencode(".jpg", 
                    scaled_image, 
                    buf, 
                    compressing_factor);
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








