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





#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <sys/socket.h>
#include "truckcam_trt.h"
#include <unistd.h>
#include <jpeglib.h>
#include <setjmp.h>





#define RECV_PORT 1234
#define SEND_PORT 1235
#define SOCKET_BUFSIZE 1024

#define START_CODE0 0xff
#define START_CODE1 0xe7
// packet type
#define VIJEO 0x00
#define STATUS 0x01

int recv_socket = -1;
int send_socket = -1;
uint32_t send_addr = 0;
sem_t data_ready;
uint8_t prev_error_flags = 0xff;
pthread_mutex_t www_mutex;
int current_input2 = -1;
int keypoint_size2 = 0;
uint8_t status_buffer[HEADER_SIZE + 32];
int status_size = 0;


// storage for packet header, keypoints & compressed frame
extern uint8_t vijeo_buffer[];
int vijeo_size = 0;


struct my_jpeg_error_mgr {
  struct jpeg_error_mgr pub;	/* "public" fields */
  jmp_buf setjmp_buffer;	/* for return to caller */
};

static struct my_jpeg_error_mgr my_jpeg_error;

typedef struct 
{
	struct jpeg_destination_mgr pub; /* public fields */

	JOCTET *buffer;		/* Pointer to buffer */
} my_destination_mgr;


METHODDEF(void) init_destination(j_compress_ptr cinfo)
{
  	my_destination_mgr *dest = (my_destination_mgr*)cinfo->dest;

/* Set the pointer to the preallocated buffer */
    vijeo_size = 0;
  	dest->buffer = vijeo_buffer + HEADER_SIZE + keypoint_size2;
  	dest->pub.next_output_byte = dest->buffer;
  	dest->pub.free_in_buffer = MAX_JPEG - HEADER_SIZE - keypoint_size2;
}


/*
 * Terminate destination --- called by jpeg_finish_compress
 * after all data has been written.  Usually needs to flush buffer.
 *
 * NB: *not* called by jpeg_abort or jpeg_destroy; surrounding
 * application must deal with any cleanup that should happen even
 * for error exit.
 */
METHODDEF(void) term_destination(j_compress_ptr cinfo)
{
/* Just get the length */
	my_destination_mgr *dest = (my_destination_mgr*)cinfo->dest;
	vijeo_size = MAX_JPEG - HEADER_SIZE - dest->pub.free_in_buffer;
}

/*
 * Empty the output buffer --- called whenever buffer fills up.
 *
 * In typical applications, this should write the entire output buffer
 * (ignoring the current state of next_output_byte & free_in_buffer),
 * reset the pointer & count to the start of the buffer, and return TRUE
 * indicating that the buffer has been dumped.
 *
 * In applications that need to be able to suspend compression due to output
 * overrun, a FALSE return indicates that the buffer cannot be emptied now.
 * In this situation, the compressor will return to its caller (possibly with
 * an indication that it has not accepted all the supplied scanlines).  The
 * application should resume compression after it has made more room in the
 * output buffer.  Note that there are substantial restrictions on the use of
 * suspension --- see the documentation.
 *
 * When suspending, the compressor will back up to a convenient restart point
 * (typically the start of the current MCU). next_output_byte & free_in_buffer
 * indicate where the restart point will be if the current call returns FALSE.
 * Data beyond this point will be regenerated after resumption, so do not
 * write it out when emptying the buffer externally.
 */

METHODDEF(boolean) empty_vijeo_buffer(j_compress_ptr cinfo)
{
/* Allocate a bigger buffer. */
	my_destination_mgr *dest = (my_destination_mgr*)cinfo->dest;

// not implemented
printf("empty_vijeo_buffer %d NOT IMPLEMENTED\n", __LINE__);
	dest->buffer = vijeo_buffer + HEADER_SIZE;
	dest->pub.next_output_byte = dest->buffer;
	dest->pub.free_in_buffer = MAX_JPEG - HEADER_SIZE;
	return TRUE;
}



void compress_jpeg()
{
	struct jpeg_compress_struct cinfo;
	cinfo.err = jpeg_std_error(&(my_jpeg_error.pub));
    jpeg_create_compress(&cinfo);
    cinfo.image_width = SERVER_W;
    cinfo.image_height = SERVER_H;
    cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 50, 0);
    cinfo.dct_method = JDCT_IFAST;
    my_destination_mgr *dest;
    if(cinfo.dest == NULL) 
	{
/* first time for this JPEG object? */
      	cinfo.dest = (struct jpeg_destination_mgr *)
    		(*cinfo.mem->alloc_small)((j_common_ptr)&cinfo, 
				JPOOL_PERMANENT,
				sizeof(my_destination_mgr));
	}

	dest = (my_destination_mgr*)cinfo.dest;
	dest->pub.init_destination = init_destination;
	dest->pub.empty_output_buffer = empty_vijeo_buffer;
	dest->pub.term_destination = term_destination;


    jpeg_start_compress(&cinfo, TRUE);
    while(cinfo.next_scanline < cinfo.image_height)
	{
    	jpeg_write_scanlines(&cinfo, 
			&input_rows[current_input2][cinfo.next_scanline], 
			1);
    }
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
}

void send_status()
{
    int offset = HEADER_SIZE;
    status_buffer[offset++] = current_operation;
    status_buffer[offset++] = error_flags;
    prev_error_flags = error_flags;

    status_size = offset - HEADER_SIZE;
// wake up the writer
    sem_post(&data_ready);
}

void send_error()
{
    pthread_mutex_lock(&www_mutex);
    if(error_flags != prev_error_flags)
    {
		send_status();
    }
    pthread_mutex_unlock(&www_mutex);
}




void send_vijeo(int current_input, int keypoint_size)
{
    pthread_mutex_lock(&www_mutex);
    current_input2 = current_input;
    keypoint_size2 = keypoint_size;
    sem_post(&data_ready);
    pthread_mutex_unlock(&www_mutex);
}



void* web_server_reader(void *ptr)
{
	unsigned char buffer[SOCKET_BUFSIZE];
	while(1)
	{
        struct sockaddr_in peer_addr;
        socklen_t peer_addr_len = sizeof(struct sockaddr_in);
//printf("web_server_reader %d\n", __LINE__);
        int bytes_read = recvfrom(recv_socket,
            buffer, 
            SOCKET_BUFSIZE, 
            0,
            (struct sockaddr *) &peer_addr, 
            &peer_addr_len);
//printf("web_server_reader %d\n", __LINE__);
        if(send_addr != peer_addr.sin_addr.s_addr)
        {
            printf("web_server_reader %d: new connection\n", __LINE__);
            printf("web_server_reader %d peer_addr=%08x\n", 
                __LINE__, 
                peer_addr.sin_addr.s_addr);
            if(send_socket >= 0)
                close(send_socket);
            send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            send_addr = peer_addr.sin_addr.s_addr;
            peer_addr.sin_port = htons((unsigned short)SEND_PORT);
            connect(send_socket, 
		        (struct sockaddr*)&peer_addr, 
		        peer_addr_len);
        }

        int i;
        for(i = 0; i < bytes_read; i++)
        {
            int c = buffer[i];
//printf("web_server_reader %d '%c'\n", __LINE__, buffer[i]);
            if(c == '*')
            {
                send_status();
            }
        }
    }
}


int send_packet(int type, 
    uint8_t *data, // pointer to start of header
    int bytes) // size not including header
{
    int size = HEADER_SIZE + bytes;

// write the header
    data[0] = START_CODE0;
    data[1] = START_CODE1;
    data[2] = type;
    data[3] = 0;
    data[4] = bytes & 0xff;
    data[5] = (bytes >> 8) & 0xff;
    data[6] = (bytes >> 16) & 0xff;
    data[7] = (bytes >> 24) & 0xff;

//printf("send_packet %d %d %d\n", __LINE__, type, size);
    int result = -1;
    pthread_mutex_lock(&www_mutex);
    result = write(send_socket, data, size);
    pthread_mutex_unlock(&www_mutex);
    return result;
}


void* web_server_writer(void *ptr)
{
	unsigned char buffer[SOCKET_BUFSIZE];
	int i;

	while(1)
	{
// wait for data to write
        sem_wait(&data_ready);

// no client.  Clear all the buffers
        if(send_socket < 0)
        {
            status_size = 0;
            keypoint_size2 = 0;
            current_input2 = -1;
            continue;
        }

        if(status_size)
        {
//printf("web_server_writer %d STATUS\n", __LINE__);
            send_packet(STATUS, status_buffer, status_size);
            status_size = 0;
        }

        if(keypoint_size2 && current_input2 >= 0)
        {

// scale input image 2:1
            for(int i = 0; i < SERVER_H; i++)
            {
                uint8_t *src_row = input_rows[current_input2][i * 2];
                uint8_t *dst_row = input_rows[current_input2][i];
                for(int j = 0; j < SERVER_W; j++)
                {
// swap bytes
                    uint8_t temp = src_row[0];
                    *dst_row++ = src_row[2];
                    *dst_row++ = src_row[1];
                    *dst_row++ = temp;
                    src_row += 6;
                }
            }
// compress it
            compress_jpeg();
            send_packet(VIJEO, vijeo_buffer, vijeo_size);
            keypoint_size2 = 0;
            current_input2 = -1;
        }
	}
}


void init_server()
{
	recv_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	int reuseon = 1;
    setsockopt(recv_socket, SOL_SOCKET, SO_REUSEADDR, &reuseon, sizeof(reuseon));
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(RECV_PORT);

    int result = bind(recv_socket, (struct sockaddr *) &addr, sizeof(addr));
	if(result)
	{
		printf("init_server %d: bind port %d failed\n", __LINE__, RECV_PORT);
	}



	sem_init(&data_ready, 0, 0);

	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_t tid;

	pthread_create(&tid, 
		&attr, 
		web_server_writer, 
		0);
	pthread_create(&tid, 
		&attr, 
		web_server_reader, 
		0);
}
