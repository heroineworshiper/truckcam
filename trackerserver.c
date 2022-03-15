/*
 * Server for the tracking camera
 * Copyright (C) 2019-2021 Adam Williams <broadcast at earthling dot net>
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
#include <vector>
#include "tensortrack.h"


#ifdef USE_SERVER

#define PORT0 1234
#define PORT1 1238
#define TOTAL_CONNECTIONS 4
#define SOCKET_BUFSIZE 1024
#define SERVER_NAME "Tracker"
#define FFMPEG_STDIN "/tmp/ffmpeg_stdin"
#define FFMPEG_STDOUT "/tmp/ffmpeg_stdout"

// packet type
#define VIJEO 0x00
#define STATUS 0x01

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




int send_packet(int fd, int type, uint8_t *data, int bytes)
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
        result = write(fd, header, 8);
        result = write(fd, data, bytes);
    }
    pthread_mutex_unlock(&www_mutex);
    return result;
}

void send_status(webserver_connection_t *connection)
{
    uint8_t buffer[32];
    buffer[0] = current_operation;
    buffer[1] = 0;

    int tmp = (int)pan;
    buffer[2] = tmp & 0xff;
    buffer[3] = (tmp >> 8) & 0xff;
    buffer[4] = (tmp >> 16) & 0xff;
    buffer[5] = (tmp >> 24) & 0xff;

    tmp = (int)tilt;
    buffer[6] = tmp & 0xff;
    buffer[7] = (tmp >> 8) & 0xff;
    buffer[8] = (tmp >> 16) & 0xff;
    buffer[9] = (tmp >> 24) & 0xff;

    tmp = (int)start_pan;
    buffer[10] = tmp & 0xff;
    buffer[11] = (tmp >> 8) & 0xff;
    buffer[12] = (tmp >> 16) & 0xff;
    buffer[13] = (tmp >> 24) & 0xff;

    tmp = (int)start_tilt;
    buffer[14] = tmp & 0xff;
    buffer[15] = (tmp >> 8) & 0xff;
    buffer[16] = (tmp >> 16) & 0xff;
    buffer[17] = (tmp >> 24) & 0xff;

    buffer[18] = pan_sign;
    buffer[19] = tilt_sign;
    buffer[20] = lens;
    buffer[21] = landscape;
    buffer[22] = error_flags;

    prev_error_flags = error_flags;

//    printf("send_status %d error_flags=%d\n", __LINE__, error_flags);
    send_packet(connection->fd, STATUS, buffer, 23);
}

void send_error()
{
    pthread_mutex_lock(&www_mutex);
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
    pthread_mutex_unlock(&www_mutex);
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
                
// do everything GUI::keypress_event does
                
                if(current_operation == STARTUP)
                {
                    if(c == ' ')
                    {
                        current_operation = CONFIGURING;

                        send_status(connection);

// write it a few times to defeat UART initialization glitches
                        write_servos(1);
                        usleep(100000);
                        write_servos(1);
                        usleep(100000);
                        write_servos(1);
                        usleep(100000);
                        write_servos(1);
                    }
                }
                else
                if(current_operation == CONFIGURING)
                {
                    int need_write_servos = 0;
                    
                    switch(c)
                    {
                        case 'q':
                            stop_servos();
                            ::save_defaults();
                            current_operation = STARTUP;
                            break;

                        case '\n':
                            ::save_defaults();
                            current_operation = TRACKING;
                            break;
                        
                        case 'w':
                            tilt += lenses[lens].tilt_step * tilt_sign;
                            start_pan = pan;
                            start_tilt = tilt;
                            need_write_servos = 1;
                            break;

                        case 's':
                            tilt -= lenses[lens].tilt_step * tilt_sign;
                            start_pan = pan;
                            start_tilt = tilt;
                            need_write_servos = 1;
                            break;

                        case 'a':
                            pan -= lenses[lens].pan_step * pan_sign;
                            start_pan = pan;
                            start_tilt = tilt;
                            need_write_servos = 1;
                            break;

                        case 'd':
                            pan += lenses[lens].pan_step * pan_sign;
                            start_pan = pan;
                            start_tilt = tilt;
                            need_write_servos = 1;
                            break;

                        case 'c':
                            pan = start_pan;
                            tilt = start_tilt;
                            need_write_servos = 1;
                            break;

                        case 't':
                            tilt_sign *= -1;
                            ::save_defaults();
                            break;

                        case 'p':
                            pan_sign *= -1;
                            ::save_defaults();
                            break;

//                         case 'r':
//                             landscape = !landscape;
//                             ::save_defaults();
//                             break;

                        case 'r':
                            if(!landscape)
                            {
                                landscape = 1;
                                ::save_defaults();
                            }
                            break;
                            
                        case 'R':
                            if(landscape)
                            {
                                landscape = 0;
                                ::save_defaults();
                            }
                            break;

                        case 'l':
//                             lens++;
//                             if(lens >= TOTAL_LENSES)
//                             {
//                                 lens = 0;
//                             }
//                             ::save_defaults();
                            break;
                    }
                    
                    if(need_write_servos)
                    {
                        write_servos(1);
                    }
                    send_status(connection);
                }
                else
                if(current_operation == TRACKING)
                {
                    switch(c)
                    {
                        case 'Q':
                            stop_servos();
                            current_operation = STARTUP;
                            send_status(connection);
                            break;

                        case 'b':
                            current_operation = CONFIGURING;
                            send_status(connection);
                            break;
                    }
                }
                
            }
        }

		printf("web_server_reader %d: client closed\n", __LINE__);
        pthread_mutex_lock(&www_mutex);
		connection->is_reading = 0;
// notify the writer
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

        send_status(connection);
        send_vijeo(connection);


		printf("web_server_writer %d: client closed\n", __LINE__);
        pthread_mutex_lock(&www_mutex);
		close(connection->fd);
        connection->fd = -1;
		connection->is_writing = 0;
        server_output = -1;
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
    pthread_mutex_lock(&www_mutex);
	connection->is_writing = 1;
	connection->is_reading = 1;
	connection->fd = fd;
    server_output = fd;
    current_connection = connection;
    pthread_mutex_unlock(&www_mutex);


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
        int flag = 1; 
        setsockopt(connection_fd, 
            IPPROTO_TCP, 
            TCP_NODELAY, 
            (char *)&flag, 
            sizeof(int));

//printf("web_server %d: accept\n", __LINE__);

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


	pthread_mutexattr_t attr2;
	pthread_mutexattr_init(&attr2);
    pthread_mutexattr_settype(&attr2, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&www_mutex, &attr2);
    
    pthread_t tid;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
   	pthread_create(&tid, 
		&attr, 
		web_server, 
		0);
 
}





#endif // USE_SERVER
















