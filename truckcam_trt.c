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

// To build it:
// 
// make -f Makefile.trt
// 
// To run it:
// 
// ./truckcam_trt


#include <dirent.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "trackerlib.h"
#include "truckcam_trt.h"
#include <unistd.h>
#include <linux/videodev2.h>

#include "NvInfer.h"
#include "faceNet.h"
#include <string>
#include "mtcnn.h"

#define HDMI_BUFFERS 2

int current_operation = IDLE;
uint8_t error_flags = 0;
int deadband = 0;
int speed = 100;
int servo_fd = -1;
int cam_fd = -1;
float fps = 0;
int fps_count = 0;
struct timespec fps_time1;
unsigned char *mmap_buffer[HDMI_BUFFERS];
// output buffer for JPEG decompression
// 1 is owned by the server for JPEG compression
// 1 is owned by the scanner
uint8_t *input_image[INPUT_IMAGES];
uint8_t **input_rows[INPUT_IMAGES];
int current_input = 0;
// storage for packet header, keypoints & compressed frame
uint8_t vijeo_buffer[HEADER_SIZE + 2 + 2 + MAX_FACES * 256 + MAX_JPEG];

Logger gLogger;

void loadInputImage(std::string inputFilePath, cv::Mat& image, int videoFrameWidth, int videoFrameHeight) 
{
    image = cv::imread(inputFilePath.c_str());
    cv::resize(image, image, cv::Size(videoFrameWidth, videoFrameHeight));
}


void getFilePaths(std::string imagesPath, std::vector<struct Paths>& paths) 
{
//    std::cout << "Parsing Directory: " << imagesPath << std::endl;
    DIR *dir;
    struct dirent *entry;
    if ((dir = opendir (imagesPath.c_str())) != NULL) {
        while ((entry = readdir (dir)) != NULL) {
            std::string readmeCheck(entry->d_name);
            if (entry->d_type != DT_DIR && readmeCheck != "README.md") {
                struct Paths tempPaths;
                tempPaths.fileName = std::string(entry->d_name);
                tempPaths.absPath = imagesPath + "/" + tempPaths.fileName;
                paths.push_back(tempPaths);
            }
        }
        closedir (dir);
    }
}


// probe for the video device
int open_hdmi(int verbose)
{
    vector<char*> paths;
    char string[TEXTLEN];

// discover the paths
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

            v4l2_params.fmt.pix.width = RAW_W;
            v4l2_params.fmt.pix.height = RAW_H;

            if(ioctl(fd2, VIDIOC_S_FMT, &v4l2_params) < 0)
            {
//              printf("hdmi_thread %d: VIDIOC_S_FMT failed\n",
//                  __LINE__);
            }


            if(verbose)
            {
                printf("open_hdmi %d opened %s\n", __LINE__, paths.at(i));
            }


            struct v4l2_requestbuffers requestbuffers;
            requestbuffers.count = HDMI_BUFFERS;
            requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            requestbuffers.memory = V4L2_MEMORY_MMAP;
            if(ioctl(fd2, VIDIOC_REQBUFS, &requestbuffers) < 0)
            {
                printf("open_hdmi %d: VIDIOC_REQBUFS failed\n",
                    __LINE__);
            }
            else
            {
                for(int i = 0; i < HDMI_BUFFERS; i++)
                {
                    struct v4l2_buffer buffer;
                    buffer.type = requestbuffers.type;
                    buffer.index = i;

                    if(ioctl(fd2, VIDIOC_QUERYBUF, &buffer) < 0)
				    {
					    printf("open_hdmi %d: VIDIOC_QUERYBUF failed\n",
                            __LINE__);
				    }
                    else
                    {
                        mmap_buffer[i] = (unsigned char*)mmap(NULL,
					        buffer.length,
					        PROT_READ | PROT_WRITE,
					        MAP_SHARED,
					        fd2,
					        buffer.m.offset);
//                             printf("hdmi_thread %d: allocated buffer size=%d\n",
//                                 __LINE__,
//                                 buffer.length);
                        if(ioctl(fd2, VIDIOC_QBUF, &buffer) < 0)
                        {
                            printf("open_hdmi %d: VIDIOC_QBUF failed\n",
                                __LINE__);
                        }
                    }
                }
            }

            int streamon_arg = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	        if(ioctl(fd2, VIDIOC_STREAMON, &streamon_arg) < 0)
            {
		        printf("open_hdmi %d: VIDIOC_STREAMON failed\n",
                    __LINE__);
            }
            clock_gettime(CLOCK_MONOTONIC, &fps_time1);


            break;
        }
    }
//printf("open_hdmi %d fd2=%d\n", __LINE__, fd2);

// delete the paths
    while(paths.size() > 0)
    {
        free(paths.back());
        paths.pop_back();
    }

    return fd2;
}


void do_tracker()
{
    cv::Mat *cv_frame[INPUT_IMAGES];
    FaceNetClassifier faceNet = FaceNetClassifier(gLogger, 
        DataType::kHALF, 
        string("/root/face_recognition_tensorRT/facenetModels/facenet.uff"), 
        string("/root/face_recognition_tensorRT/facenetModels/facenet.engine"), 
        1, // batchSize
        true, // serializeEngine
        (float)1.0, // knownPersonThreshold
        MAX_FACES, // maxFacesPerScene
        SCAN_W, 
        SCAN_H);
// this creates a bunch of .engine files
    mtcnn mtCNN(SCAN_H, 
        SCAN_W, 
        "/root/face_recognition_tensorRT/mtCNNModels/");

    std::vector<struct Bbox> outputBoxes;
    outputBoxes.reserve(MAX_FACES);
    std::vector<struct Paths> paths;
    getFilePaths(KNOWN_PATH, paths);
    if(paths.size() == 0)
    {
        printf("do_tracker %d: no known faces in %s\n", __LINE__, KNOWN_PATH);
    }

    for(int j = 0; j < INPUT_IMAGES; j++)
    {
        cv_frame[j] = new cv::Mat(SCAN_H, SCAN_W, CV_8UC3);
        input_image[j] = cv_frame[j]->data;
        input_rows[j] = new uint8_t*[RAW_H];
        for(int i = 0; i < RAW_H; i++)
        {
            input_rows[j][i] = cv_frame[j]->ptr<uint8_t>(i);
//            input_image[j] + i * RAW_W * 3;
        }
    }

// known faces have to be the the same size as the MTCNN
    cv::Mat known_image;
    std::vector<std::string> known;
    for(int i=0; i < paths.size(); i++) {
        loadInputImage(paths[i].absPath, known_image, SCAN_W, SCAN_H);
        outputBoxes = mtCNN.findFace(known_image);
        std::size_t index = paths[i].fileName.find_last_of(".");
        std::string rawName = paths[i].fileName.substr(0,index);
        faceNet.forwardAddFace(known_image, outputBoxes, rawName);
        known.push_back(rawName);
        faceNet.resetVariables();
        printf("main %d known=%s\n", __LINE__, paths[i].absPath.c_str());
    }
//    printf("main %d total known=%d\n", __LINE__, (int)outputBoxes.size());
    outputBoxes.clear();

// the mane loop
    int verbose = 1;
    while(1)
    {
        if(cam_fd < 0)
        {
            cam_fd = open_hdmi(verbose);
        }

        if(cam_fd < 0)
        {
            if(!(error_flags & VIDEO_DEVICE_ERROR))
            {
                printf("do_tracker %d: no video device\n",
                    __LINE__);
            }
            error_flags |= VIDEO_DEVICE_ERROR;
            send_error();
            sleep(1);
        }
        else
        {
            struct v4l2_buffer buffer;
		    bzero(&buffer, sizeof(buffer));
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		    buffer.memory = V4L2_MEMORY_MMAP;

// select seems to protect against disconnects
            fd_set fds;
            struct timeval tv;
            FD_ZERO(&fds);
            FD_SET(cam_fd, &fds);
            tv.tv_sec = 2;
            tv.tv_usec = 0;
            int result = select(cam_fd + 1, &fds, NULL, NULL, &tv);
            if(result <= 0)
            {
                printf("do_tracker %d: V4L2 died\n", __LINE__);
                error_flags |= VIDEO_BUFFER_ERROR;
                send_error();
                close(cam_fd);
                cam_fd = -1;
                sleep(1);
            }
            else
            {

                if(ioctl(cam_fd, VIDIOC_DQBUF, &buffer) < 0)
                {
                    printf("do_tracker %d: VIDIOC_DQBUF failed\n",
                        __LINE__);
                    error_flags |= VIDEO_BUFFER_ERROR;
                    send_error();
                    close(cam_fd);
                    cam_fd = -1;
                    sleep(1);
                }
                else
                {
                    error_flags &= ~VIDEO_BUFFER_ERROR;
                    error_flags &= ~VIDEO_DEVICE_ERROR;
//                  printf("do_tracker %d\n", __LINE__);
                    send_error();
    //printf("do_tracker %d\n", __LINE__);
                    unsigned char *ptr = mmap_buffer[buffer.index];
    //                 printf("do_tracker %d: index=%d size=%d %02x %02x %02x %02x %02x %02x %02x %02x\n",
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
// process it
                        int decoded_w;
                        int decoded_h;
                        decompress_jpeg(ptr, 
                            buffer.bytesused,
                            &decoded_w,
                            &decoded_h,
                            input_rows[current_input]);

// must swap byte order
                        for(int i = 0; i < SCAN_H; i++)
                        {
                            uint8_t *row = input_rows[current_input][i];
                            for(int j = 0; j < SCAN_W; j++)
                            {
                                uint8_t temp = row[0];
                                row[0] = row[2];
                                row[2] = temp;
                                row += 3;
                            }
                        }

// transfer to opencv
                        outputBoxes = mtCNN.findFace(*cv_frame[current_input]);

// printf("do_tracker %d %d\n", 
// __LINE__, 
// (int)outputBoxes.size());
                        faceNet.forward(*cv_frame[current_input], outputBoxes);
                        std::vector<struct Bbox> boxes2 = faceNet.featureMatching2(*cv_frame[current_input]);
                        faceNet.resetVariables();

                        fps_count++;
                        struct timespec fps_time2;
                        clock_gettime(CLOCK_MONOTONIC, &fps_time2);
                        int64 delta = 
                            (int64_t)((fps_time2.tv_sec * 1000 + fps_time2.tv_nsec / 1000000) -
                            (fps_time1.tv_sec * 1000 + fps_time1.tv_nsec / 1000000));
                        if(delta >= 1000)
                        {
                            fps = (float)fps_count * 1000 / delta;
                            fps_time1 = fps_time2;
                            fps_count = 0;
                            printf("do_tracker %d FPS: %f\n", __LINE__, fps);
                        }

// server is ready for data
                        if(current_input2 < 0)
                        {
                            int offset = HEADER_SIZE;
// frames per second
                            int fps_i = (int)(fps * 256);
                            vijeo_buffer[offset++] = fps_i & 0xff;
                            vijeo_buffer[offset++] = fps_i >> 8;
// number of animals detected & bounding boxes
                            int total = boxes2.size();
                            if(total > MAX_FACES) total = MAX_FACES;
                            vijeo_buffer[offset++] = total;
                            vijeo_buffer[offset++] = 0;
                            for(int i = 0; i < total; i++)
                            {
// the bounding box
                                vijeo_buffer[offset++] = boxes2[i].y1 & 0xff;
                                vijeo_buffer[offset++] = boxes2[i].y1 >> 8;
                                vijeo_buffer[offset++] = boxes2[i].x1 & 0xff;
                                vijeo_buffer[offset++] = boxes2[i].x1 >> 8;
                                vijeo_buffer[offset++] = boxes2[i].y2 & 0xff;
                                vijeo_buffer[offset++] = boxes2[i].y2 >> 8;
                                vijeo_buffer[offset++] = boxes2[i].x2 & 0xff;
                                vijeo_buffer[offset++] = boxes2[i].x2 >> 8;
// the name
                                if(boxes2[i].score >= 0)
                                {
// printf("do_tracker %d i=%d id=%d\n", 
// __LINE__,
// i,
// (int)boxes2[i].score);
                                    strcpy((char*)vijeo_buffer + offset,
                                        known.at((int)boxes2[i].score).c_str());
                                    offset += known.at((int)boxes2[i].score).length() + 1;
                                }
                                else
                                {
                                    vijeo_buffer[offset++] = 0;
                                }
                            }
                            send_vijeo(current_input, offset - HEADER_SIZE);
// avoid a race condition by not toggling this if the server is busy
                            current_input = !current_input;
                        }
                    }

                    if(ioctl(cam_fd, VIDIOC_QBUF, &buffer) < 0)
                    {
                        printf("do_tracker %d: VIDIOC_QBUF failed\n",
                            __LINE__);
                    }

// these have to be reset for every frame
// reset saturation
                    struct v4l2_control ctrl_arg;
                    ctrl_arg.id = 0x980902;
                    ctrl_arg.value = 127;
                    ioctl(cam_fd, VIDIOC_S_CTRL, &ctrl_arg);
// reset backlight compensation
                    ctrl_arg.id = 0x98091c;
                    ctrl_arg.value = 0;
                    ioctl(cam_fd, VIDIOC_S_CTRL, &ctrl_arg);
                }
            }
        }
    }
}


// detect when the servo board is unplugged & manetain the connection
void* servo_reader(void *ptr)
{
    int servos = 0;

    while(1)
    {
// open the device
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

void init_servos()
{
    pthread_t x;
	pthread_create(&x, 
		0, 
		servo_reader, 
		0);


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

int main(int argc, char** argv)
{
    load_defaults();
    dump_settings();


    init_server();
    init_servos();
    
    do_tracker();
}







