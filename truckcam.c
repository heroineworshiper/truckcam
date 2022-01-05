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


#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>

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

using namespace cv;
using namespace std;


#define COS_THRESHOLD 0.363
#define SCORE_THRESHOLD 0.9
#define NMS_THRESHOLD 0.3
#define TOP_K 5000
#define DETECT_MODEL "yunet.onnx"
#define COMPARE_MODEL "face_recognizer_fast.onnx"
#define REF_FACE "lion.jpg"

// size of downscaled vijeo
#define RAW_W 1920
#define RAW_H 1080
#define HDMI0 0
#define HDMI1 4


#define BUFSIZE2 0x400000
// compressed frame from the hardware
unsigned char reader_buffer3[BUFSIZE2];
// size of the frame in reader_buffer3
int frame_size;
static pthread_mutex_t frame_lock;
static sem_t frame_ready_sema;
uint8_t error_flags = 0xff;
FILE *ffmpeg_fd = 0;
int servo_fd = -1;



// read frames from HDMI
void* hdmi_thread(void *ptr)
{
    struct timeval time1;
    struct timespec fps_time1;
    gettimeofday(&time1, 0);

    int current_path = HDMI0;
    int verbose = 1;
    int fd;
    unsigned char *mmap_buffer[HDMI_BUFFERS];

    while(1)
    {
        if(fd < 0)
        {
            if(verbose)
            {
                printf("hdmi_thread %d opening video4linux\n", __LINE__);
            }

// probe for the video device
            char string[TEXTLEN];
            sprintf(string, "/dev/video%d", current_path);
            fd = open(string, O_RDWR);

            if(fd < 0)
            {
                if(!(error_flags & VIDEO_DEVICE_ERROR))
                {
                    printf("hdmi_thread %d: failed to open %s\n",
                        __LINE__,
                        string);
                }
                error_flags |= VIDEO_DEVICE_ERROR;
                send_error();
                sleep(1);
            }
            else
            {

//                     printf("hdmi_thread %d: opened %s\n",
//                         __LINE__,
//                         string);

                struct v4l2_format v4l2_params;
                v4l2_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                ioctl(fd, VIDIOC_G_FMT, &v4l2_params);

                if(verbose)
                {
                    printf("hdmi_thread %d: default format=%c%c%c%c w=%d h=%d\n",
                        __LINE__,
                        v4l2_params.fmt.pix.pixelformat & 0xff,
                        (v4l2_params.fmt.pix.pixelformat >> 8) & 0xff,
                        (v4l2_params.fmt.pix.pixelformat >> 16) & 0xff,
                        (v4l2_params.fmt.pix.pixelformat >> 24) & 0xff,
                        v4l2_params.fmt.pix.width,
                        v4l2_params.fmt.pix.height);
                }

// reject it if it's the wrong resolution, since it's the laptop webcam
                if(v4l2_params.fmt.pix.width != RAW_W ||
                    v4l2_params.fmt.pix.height != RAW_H)
                {
                    if(!(error_flags & VIDEO_DEVICE_ERROR))
                    {
                        printf("hdmi_thread %d wrong camera\n",
                            __LINE__);
                    }
                    error_flags |= VIDEO_DEVICE_ERROR;
                    send_error();
                    close(fd);
                    fd = -1;
                    sleep(1);
                }
                else
                {
                    if((error_flags & VIDEO_DEVICE_ERROR))
                    {
                        printf("hdmi_thread %d: opened %s\n",
                            __LINE__,
                            string);
                        error_flags &= ~VIDEO_DEVICE_ERROR;
                        send_error();
                    }

                    v4l2_params.fmt.pix.width = RAW_W;
                    v4l2_params.fmt.pix.height = RAW_H;

#ifdef RAW_HDMI
                    v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#else
                    v4l2_params.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
#endif
                    if(ioctl(fd, VIDIOC_S_FMT, &v4l2_params) < 0)
                    {
                        printf("hdmi_thread %d: VIDIOC_S_FMT failed\n",
                            __LINE__);
                    }
                }
            }

            if(fd >= 0)
            {
//                     struct v4l2_jpegcompression jpeg_opts;
//                     if(ioctl(fd, VIDIOC_G_JPEGCOMP, &jpeg_opts) < 0)
//                     {
//                         printf("hdmi_thread %d: VIDIOC_G_JPEGCOMP failed\n",
//                             __LINE__);
//                     }
//                     printf("hdmi_thread %d: quality=%d\n",
//                         __LINE__,
//                         jpeg_opts.quality);
//                     
//                     if(ioctl(fd, VIDIOC_S_JPEGCOMP, &jpeg_opts) < 0)
//                     {
//                         printf("hdmi_thread %d: VIDIOC_S_JPEGCOMP failed\n",
//                             __LINE__);
//                     }

                struct v4l2_requestbuffers requestbuffers;
                requestbuffers.count = HDMI_BUFFERS;
                requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                requestbuffers.memory = V4L2_MEMORY_MMAP;
                if(ioctl(fd, VIDIOC_REQBUFS, &requestbuffers) < 0)
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

                        if(ioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0)
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
					            fd,
					            buffer.m.offset);
                            printf("hdmi_thread %d: allocated buffer size=%d\n",
                                __LINE__,
                                buffer.length);
                            if(ioctl(fd, VIDIOC_QBUF, &buffer) < 0)
                            {
                                printf("hdmi_thread %d: VIDIOC_QBUF failed\n",
                                    __LINE__);
                            }
                        }
                    }
                }

                int streamon_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	            if(ioctl(fd, VIDIOC_STREAMON, &streamon_arg) < 0)
                {
		            printf("hdmi_thread %d: VIDIOC_STREAMON failed\n",
                        __LINE__);
                }
                clock_gettime(CLOCK_MONOTONIC, &fps_time1);
            }
        }


        if(fd >= 0)
        {
            struct v4l2_buffer buffer;
		    bzero(&buffer, sizeof(buffer));
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		    buffer.memory = V4L2_MEMORY_MMAP;
            if(ioctl(fd, VIDIOC_DQBUF, &buffer) < 0)
            {
                printf("hdmi_thread %d: VIDIOC_DQBUF failed\n",
                    __LINE__);
                error_flags |= VIDEO_BUFFER_ERROR;
                send_error();
                close(fd);
                fd = -1;
                sleep(1);
            }
            else
            {
                error_flags &= ~VIDEO_BUFFER_ERROR;
                error_flags &= ~VIDEO_DEVICE_ERROR;
                send_error();
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

                if(ptr[0] == 0xff && 
                    ptr[1] == 0xd8 && 
                    ptr[2] == 0xff && 
                    ptr[3] == 0xdb)
                {
// discard if it arrived too soon
                    fps_frame_count++;
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
                }

                if(ioctl(fd, VIDIOC_QBUF, &buffer) < 0)
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
//                         printf("hdmi_thread %d FPS: %f\n",
//                             __LINE__,
//                             (double)frame_count * 1000 / diff);
                    frame_count = 0;
                    time1 = time2;
                }

            }
        }

        if(fd < 0)
        {
            current_path++;
            if(current_path > HDMI1)
            {
                current_path = HDMI0;
            }
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

void visualize(Mat& input, float[] &scores, Mat& faces, double fps)
{
    int thickness = 2;
    std::string fpsString = cv::format("%.2f FPS", (float)fps);
    for (int i = 0; i < faces.rows; i++)
    {
// Draw bounding box
        rectangle(input, 
            Rect2i(int(faces.at<float>(i, 0)), 
                int(faces.at<float>(i, 1)), 
                int(faces.at<float>(i, 2)), 
                int(faces.at<float>(i, 3))), 
            Scalar(0, 255, 0), thickness);
// draw score
        std::string scoreString = cv::format("%.2f", 
            scores[i]);
        putText(input, 
            scoreString, 
            Point(faces.at<float>(i, 0), int(faces.at<float>(i, 1)), 
            FONT_HERSHEY_SIMPLEX, 
            0.5, 
            Scalar(0, 255, 0), 
            2);

// Draw landmarks
//         circle(input, Point2i(int(faces.at<float>(i, 4)), int(faces.at<float>(i, 5))), 2, Scalar(255, 0, 0), thickness);
//         circle(input, Point2i(int(faces.at<float>(i, 6)), int(faces.at<float>(i, 7))), 2, Scalar(0, 0, 255), thickness);
//         circle(input, Point2i(int(faces.at<float>(i, 8)), int(faces.at<float>(i, 9))), 2, Scalar(0, 255, 0), thickness);
//         circle(input, Point2i(int(faces.at<float>(i, 10)), int(faces.at<float>(i, 11))), 2, Scalar(255, 0, 255), thickness);
//         circle(input, Point2i(int(faces.at<float>(i, 12)), int(faces.at<float>(i, 13))), 2, Scalar(0, 255, 255), thickness);
    }

    putText(input, 
        fpsString, 
        Point(0, 15), 
        FONT_HERSHEY_SIMPLEX, 
        0.5, 
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

void ignore(int sig)
{
    printf("ignore %d sig=%s\n", __LINE__, signal_titles[sig]);
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
    signal(SIGPIPE, ignore);
    signal(SIGALRM, quit);
    signal(SIGTERM, quit);
    signal(SIGCHLD, ignore);



    Ptr<FaceDetectorYN> detector = FaceDetectorYN::create(DETECT_MODEL, 
        "", 
        Size(320, 320), 
        SCORE_THRESHOLD, 
        NMS_THRESHOLD, 
        TOP_K);
    Ptr<FaceRecognizerSF> recognizer = FaceRecognizerSF::create(COMPARE_MODEL, 
        "");

// Load reference face
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

    detector->setInputSize(Size(SCALED_W, SCALED_H));


    init_hdmi();

    int frame_size2;
    Mat raw_image;
    Mat scaled_image;
    float fps = 0;
// mane loop
    while(1)
    {
// wait for next frame
        sem_wait(&frame_ready_sema);
        pthread_mutex_lock(&frame_lock);
        int got_it = 0;
        if(frame_size > 0)
        {
            Mat rawData(1, frame_size, CV_8UC1, (void*)reader_buffer3);
            raw_image = imdecode(rawData, cv::IMREAD_COLOR);
            frame_size = 0;
            if(raw_image.cols > 0 && raw_image.rows > 0)
            {
                got_it = 1;
            }
        }
        pthread_mutex_unlock(&frame_lock);

        if(got_it)
        {
            cv::Rect cropping((RAW_W - RAW_H * 4 / 3) / 2, 
                0, 
                RAW_H * 4 / 3,
                RAW_H);
            resize(raw_image(cropping), 
                scaled_image,
                Size(SCALED_W, SCALED_H),
                INTER_NEAREST);
            Mat faces;
            detector->detect(scaled_image, faces);
            
            if(faces.rows > 0)
            {
// compare all the faces to the ref
                int best_index = -1;
                double best_score = -1;
                double scores[faces.rows];
                for(int i = 0; i < faces.rows; i++)
                {
                    Mat aligned_face;
                    recognizer->alignCrop(scaled_image, 
                        faces.row(i), 
                        aligned_face);
                    Mat features;
                    recognizer->feature(aligned_face, features);
                    double cos_score = faceRecognizer->match(features, 
                        ref_features, 
                        FaceRecognizerSF::DisType::FR_COSINE);
                    scores[i] = cos_score;
                    if(cos_score > best_score)
                    {
                        best_score = cos_score;
                        best_index = i;
                    }
                }

// draw the results
                visualize(scaled_image, scores, faces, fps);
// send frame to ffmpeg server
                if(server_output > 0)
                {
                    int _ = write(server_output, 
                        (unsigned char*)scaled_image.ptr(0),
                        SCALED_W * SCALED_H * 3);
                }

// drive servo
                if(best_score >= COS_THRESHOLD)
                {
                }
            }
        }
    }

    return 0;
}








