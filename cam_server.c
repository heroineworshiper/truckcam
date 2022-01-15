/*
 * Server for the face tracking camera
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

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <pthread.h>
#include "truckcam.h"
#include <vector>

#define PORT0 1234
#define PORT1 1238
#define TOTAL_CONNECTIONS 4
#define SOCKET_BUFSIZE 1024
#define SERVER_NAME "Tracker"
#define FFMPEG_STDIN "/tmp/ffmpeg_stdin"
#define FFMPEG_STDOUT "/tmp/ffmpeg_stdout"

#define START_CODE0 0xff
#define START_CODE1 0xe7

int server_output = -1;
uint8_t prev_error_flags = 0xff;


typedef struct 
{
	int is_writing;
	int is_reading;
	int fd;
	sem_t write_lock;
	sem_t read_lock;
} webserver_connection_t;
webserver_connection_t* connections[TOTAL_CONNECTIONS];

webserver_connection_t *current_connection = 0;

pthread_mutex_t www_mutex;


int send_packet(int fd, 
    int type, 
    uint8_t *data, 
    int bytes)
{
    uint8_t header[8];
    header[0] = START_CODE0;
    header[1] = START_CODE1;
    header[2] = type;
    header[3] = 0;
    header[4] = bytes & 0xff;
    header[5] = (bytes >> 8) & 0xff;
    header[6] = (bytes >> 16) & 0xff;
    header[7] = (bytes >> 24) & 0xff;

    int result = -1;
    pthread_mutex_lock(&www_mutex);
    if(fd >= 0)
    {
//printf("send_packet %d\n", __LINE__);
        result = write(fd, header, 8);
        result = write(fd, data, bytes);
//printf("send_packet %d\n", __LINE__);
    }
    pthread_mutex_unlock(&www_mutex);
    return result;
}

void send_status(webserver_connection_t *connection)
{
    uint8_t buffer[3];
    buffer[0] = current_operation;
    buffer[1] = face_position;
    buffer[2] = error_flags;

    prev_error_flags = error_flags;

//    printf("send_status %d error_flags=%d\n", __LINE__, error_flags);
    send_packet(connection->fd, STATUS, buffer, 3);
}


void send_error()
{
// only called in the hdmi thread, so not locking 
// to work around a deadlock with send_packet
//    pthread_mutex_lock(&www_mutex);
//     printf("send_error %d error_flags=%d prev_error_flags=%d\n", 
//         __LINE__,
//         error_flags,
//         prev_error_flags);
    if(error_flags != prev_error_flags)
    {
        if(current_connection)
	    {
		    if(current_connection->is_writing &&
                current_connection->is_reading)
		    {
                //printf("send_error %d\n", __LINE__);
			    send_status(current_connection);
		    }
	    }
    }
//    pthread_mutex_unlock(&www_mutex);
}




pthread_mutex_t fifo_mutex;
sem_t fifo_ready;
int fifo_read_ptr = 0;
int fifo_write_ptr = 0;
int fifo_used = 0;
#define FIFO_SIZE 2
std::vector<uint8_t> fifo_buf[FIFO_SIZE];

void send_vijeo_fifo(uint8_t *data, int bytes)
{
    pthread_mutex_lock(&fifo_mutex);
// discard if the FIFO is full
    if(fifo_used < FIFO_SIZE)
    {
        auto fifo_ptr = &fifo_buf[fifo_write_ptr];
        fifo_ptr->resize(bytes);
        memcpy(&(*fifo_ptr)[0], data, bytes);
        fifo_used++;
        fifo_write_ptr++;
        if(fifo_write_ptr >= FIFO_SIZE)
        {
            fifo_write_ptr = 0;
        }
        sem_post(&fifo_ready);
    }
    pthread_mutex_unlock(&fifo_mutex);
}


void send_vijeo(webserver_connection_t *connection)
{
#ifdef USE_FFMPEG
//#define FFMPEG_COMMAND "ffmpeg -y -f rawvideo -y -pix_fmt bgr24 -r 30 -s:v %dx%d -i - -vf scale=%d:%d -f h264 -c:v h264 -bufsize 0 -b:v 1000k -maxrate 2M -flush_packets 1 -an - > %s"
//#define FFMPEG_COMMAND "ffmpeg -y -f rawvideo -y -pix_fmt bgr24 -r 30 -s:v %dx%d -i - -vf scale=%d:%d -f hevc -bufsize 0 -b:v 1000k -maxrate 2M -flush_packets 1 -an - > %s"
//#define FFMPEG_COMMAND "cat %s | ffmpeg -y -f rawvideo -y -pix_fmt bgr24 -r 30 -s:v %dx%d -i - -vf scale=%d:%d -f mjpeg -pix_fmt yuvj420p -bufsize 0 -b:v 5M -flush_packets 1 -an - > %s"
#define FFMPEG_COMMAND "cat %s | ffmpeg -y -f rawvideo -y -pix_fmt bgr24 -r 30 -s:v %dx%d -i - -f mjpeg -pix_fmt yuvj420p -bufsize 0 -b:v 5M -flush_packets 1 -an - > %s"

    char string[TEXTLEN];
    sprintf(string, 
        FFMPEG_COMMAND, 
        FFMPEG_STDIN,
        SCALED_W,
        SCALED_H,
        FFMPEG_STDOUT);
    printf("send_vijeo %d: running %s\n",
        __LINE__,
        string);


    pthread_mutex_lock(&www_mutex);

    int ffmpeg_pid = fork();
    if(!ffmpeg_pid)
    {
        execl("/bin/sh", "sh", "-c", string, 0);
        exit(0);
    }

// must be frame aligned so must be reopened for every stream
    FILE *server_output_fd = fopen(FFMPEG_STDIN, "w");


    if(!server_output_fd)
    {
        printf("send_vijeo %d: failed to open %s\n",
            __LINE__,
            FFMPEG_STDIN);
        pthread_mutex_unlock(&www_mutex);
        return;
    }

    FILE *ffmpeg_read = fopen(FFMPEG_STDOUT, "r");
    if(!ffmpeg_read)
    {
        printf("send_vijeo %d: failed to read ffmpeg output\n",
            __LINE__);
        fclose(server_output_fd);
        server_output = -1;
        pthread_mutex_unlock(&www_mutex);
        return;
    }
    server_output = fileno(server_output_fd);

    pthread_mutex_unlock(&www_mutex);


    unsigned char buffer[SOCKET_BUFSIZE];
    int total = 0;
    int done = 0;
    while(!done)
    {
// wait for either an exit or data to read
        fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(fileno(ffmpeg_read), &rfds);
		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;
		int result = select(fileno(ffmpeg_read) + 1, 
			&rfds, 
			0, 
			0, 
			&timeout);

// reader exited
        if(!connection->is_reading)
        {
            printf("send_vijeo %d\n", __LINE__);
            break;
        }

// ffmpeg wrote data
        if((FD_ISSET(fileno(ffmpeg_read), &rfds)))
        {
            int bytes_read = fread(buffer, 1, SOCKET_BUFSIZE, ffmpeg_read);
            int bytes_written = 0;

            if(bytes_read <= 0)
            {
                printf("send_vijeo %d: ffmpeg crashed\n",
                    __LINE__);
                done = 1;
            }
            else
            {
                bytes_written = send_packet(connection->fd, VIJEO, buffer, bytes_read);
                if(bytes_written < bytes_read)
                {
                    printf("send_vijeo %d: connection closed\n",
                        __LINE__);
                    done = 1;
                }
                total += bytes_written;
// printf("send_vijeo %d: bytes_read=%d bytes_written=%d\n",
// __LINE__,
// bytes_read,
// bytes_written);
            }
        }

    }


    pthread_mutex_lock(&www_mutex);
    kill(ffmpeg_pid, SIGKILL);
    int status;
    waitpid(ffmpeg_pid, &status, 0);
    fclose(server_output_fd);
    fclose(ffmpeg_read);
    server_output = -1;
    pthread_mutex_unlock(&www_mutex);


#else // USE_FFMPEG

    int done = 0;
    std::vector<uint8_t> buf;
    while(!done)
    {
        sem_wait(&fifo_ready);
        pthread_mutex_lock(&fifo_mutex);
        if(fifo_used > 0)
        {
            auto fifo_ptr = &fifo_buf[fifo_read_ptr];
            buf = *fifo_ptr;
            fifo_used--;
            fifo_read_ptr++;
            if(fifo_read_ptr >= FIFO_SIZE)
            {
                fifo_read_ptr = 0;
            }
        }
        pthread_mutex_unlock(&fifo_mutex);

    
        if(buf.size() > 0)
        {
            int bytes_written = 0;
            bytes_written = send_packet(connection->fd, VIJEO, &buf[0], buf.size());
            if(bytes_written < buf.size())
            {
                printf("send_vijeo %d: connection closed\n",
                    __LINE__);
                done = 1;
            }
            buf.resize(0);
        }
        else
        if(!connection->is_reading)
        {
            done = 1;
        }
    }

#endif // !USE_FFMPEG
}



void* web_server_reader(void *ptr)
{
	webserver_connection_t *connection = (webserver_connection_t*)ptr;
	unsigned char buffer[SOCKET_BUFSIZE];
	while(1)
	{
// wait for next connection
		sem_wait(&connection->read_lock);
		printf("web_server_reader %d: client opened\n", __LINE__);

#define PARSE_COMMAND 0
#define PARSE_SETTINGS 1
        int parsing_state = PARSE_COMMAND;
        int counter = 0;
        uint8_t packet[16];
        while(1)
        {
            int bytes_read = read(connection->fd, buffer, SOCKET_BUFSIZE);
//            printf("web_server_reader %d bytes_read=%d\n", __LINE__, bytes_read);
            if(bytes_read <= 0)
            {
                break;
            }

            int i;
            for(i = 0; i < bytes_read; i++)
            {
                int c = buffer[i];
//                printf("web_server_reader %d '%c'\n", __LINE__, buffer[i]);

// toggle servo
                if(parsing_state == PARSE_COMMAND)
                {
                    switch(c)
                    {
                        case ' ':
                            current_operation = TRACKING;
                            send_status(connection);
                            break;
                        case 'q':
                            current_operation = IDLE;
                            send_status(connection);
                            break;
                        case 'l':
                            face_position = FACE_LEFT;
                            ::save_defaults();
                            send_status(connection);
                            break;
                        case 'c':
                            face_position = FACE_CENTER;
                            ::save_defaults();
                            send_status(connection);
                            break;
                        case 'r':
                            face_position = FACE_RIGHT;
                            ::save_defaults();
                            send_status(connection);
                            break;
                        case 's':
                            parsing_state = PARSE_SETTINGS;
                            counter = 0;
                            break;
                    }
                }
                else
                {
                    packet[counter++] = c;
                    if(counter >= 5)
                    {
                        parsing_state = PARSE_COMMAND;
                        if(packet[0] != 0xff)
                        {
                            deadband = packet[0];
                        }

                        if(packet[1] != 0xff)
                        {
                            speed = packet[1];
                        }

                        if(packet[2] != 0xff)
                        {
                            xy_radius = packet[2];
                        }

                        if(packet[3] != 0xff)
                        {
                            size_radius = packet[3];
                        }

                        if(packet[4] != 0xff)
                        {
                            color_radius = packet[4];
                        }

                        ::save_defaults();
                        send_status(connection);
                        dump_settings();
                        clock_gettime(CLOCK_MONOTONIC, &settings_time);
                    }
                }
            }
        }

		printf("web_server_reader %d: client closed\n", __LINE__);
        pthread_mutex_lock(&www_mutex);
		connection->is_reading = 0;
        sem_post(&fifo_ready);
        pthread_mutex_unlock(&www_mutex);
    }
}

void* web_server_writer(void *ptr)
{
	webserver_connection_t *connection = (webserver_connection_t*)ptr;
	unsigned char buffer[SOCKET_BUFSIZE];
	int i;

	while(1)
	{
// wait for next connection
		sem_wait(&connection->write_lock);
		printf("web_server_writer %d: client opened\n", __LINE__);

        send_vijeo(connection);


		printf("web_server_writer %d: client closed\n", __LINE__);
        pthread_mutex_lock(&www_mutex);
		close(connection->fd);
        connection->fd = -1;
		connection->is_writing = 0;
        pthread_mutex_unlock(&www_mutex);
	}
}


webserver_connection_t* new_connection()
{
	webserver_connection_t *result = (webserver_connection_t*)calloc(1, sizeof(webserver_connection_t));

	sem_init(&result->write_lock, 0, 0);
	sem_init(&result->read_lock, 0, 0);
    result->fd = -1;

	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_t tid;
	pthread_create(&tid, 
		&attr, 
		web_server_writer, 
		result);
	pthread_create(&tid, 
		&attr, 
		web_server_reader, 
		result);
	return result;
}

void start_connection(webserver_connection_t *connection, int fd)
{
    sem_init(&fifo_ready, 0, 0);
    pthread_mutex_lock(&fifo_mutex);
    fifo_read_ptr = 0;
    fifo_write_ptr = 0;
    fifo_used = 0;
    pthread_mutex_unlock(&fifo_mutex);

    pthread_mutex_lock(&www_mutex);
	connection->is_writing = 1;
	connection->is_reading = 1;
	connection->fd = fd;
#ifndef USE_FFMPEG
    server_output = fd;
#endif
    current_connection = connection;
    pthread_mutex_unlock(&www_mutex);

    send_status(connection);

// start the threads
	sem_post(&connection->write_lock);
	sem_post(&connection->read_lock);
}


void* web_server(void *ptr)
{
	int i;
	for(i = 0; i < TOTAL_CONNECTIONS; i++)
	{
		connections[i] = new_connection();
	}
	
	int fd = socket(AF_INET, SOCK_STREAM, 0);
	
	int reuseon = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuseon, sizeof(reuseon));
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    
    for(int port = PORT0; port < PORT1; port++)
    {
        addr.sin_port = htons(port);

        int result = bind(fd, (struct sockaddr *) &addr, sizeof(addr));
	    if(result)
	    {
		    printf("web_server %d: bind %d failed\n", __LINE__, port);
		    continue;
	    }
        
        printf("web_server %d: bound port %d\n", __LINE__, port);
        break;
    }
	
	while(1)
	{
		listen(fd, 256);
		struct sockaddr_in clientname;
		socklen_t size = sizeof(clientname);
		int connection_fd = accept(fd,
                			(struct sockaddr*)&clientname, 
							&size);

//printf("web_server %d: accept\n", __LINE__);
        int flag = 1; 
        setsockopt(connection_fd, 
            IPPROTO_TCP, 
            TCP_NODELAY, 
            (char *)&flag, 
            sizeof(int));

		int got_it = 0;
		for(i = 0; i < TOTAL_CONNECTIONS; i++)
		{
			if(!connections[i]->is_writing &&
                !connections[i]->is_reading)
			{
				start_connection(connections[i], connection_fd);
				got_it = 1;
				break;
			}
		}
		
		if(!got_it)
		{
			printf("web_server %d: out of connections\n", __LINE__);
		}
	}
}



void init_server()
{
#ifdef USE_FFMPEG
    printf("init_server %d making %s %s\n", __LINE__, FFMPEG_STDIN, FFMPEG_STDOUT);
    int result = mkfifo(FFMPEG_STDIN, 0777);
    if(result != 0)
    {
        printf("init_server %d %s: %s\n", __LINE__, FFMPEG_STDIN, strerror(errno));
    }
    result = mkfifo(FFMPEG_STDOUT, 0777);
    if(result != 0)
    {
        printf("init_server %d %s: %s\n", __LINE__, FFMPEG_STDOUT, strerror(errno));
    }
#endif // USE_FFMPEG

	pthread_mutexattr_t attr2;
	pthread_mutexattr_init(&attr2);
    pthread_mutexattr_settype(&attr2, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&www_mutex, &attr2);
    pthread_mutex_init(&fifo_mutex, &attr2);

    pthread_t tid;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
   	pthread_create(&tid, 
		&attr, 
		web_server, 
		0);
 
}












