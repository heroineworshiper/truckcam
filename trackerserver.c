/*
 * Server for the tracking cameras
 * Copyright (C) 2019-2024 Adam Williams <broadcast at earthling dot net>
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

#include "tracker.h"



#ifdef USE_SERVER


#include <stdio.h>
#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>
#include <unistd.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>


#define RECV_PORT 2345
#define SEND_PORT 2346
#define SOCKET_BUFSIZE 1024


#define START_CODE0 0xff
#define START_CODE1 0xe7
#define SOCKET_BUFSIZE 1024

int recv_socket = -1;
int send_socket = -1;
uint32_t send_addr = 0;
sem_t data_ready;
// can't lock this outside the writer thread because the wifi stalls
pthread_mutex_t www_mutex;

#define TO_ADDRESS(a, b, c, d) ((d << 24) | (c << 16) | (b << 8) | (a))

void server_read(uint8_t c);
void server_write();


int send_packet(int type, 
    uint8_t *data, // pointer to start of header
    int bytes) // size not including header
{
    int size = HEADER_SIZE + bytes;

    data[0] = START_CODE0;
    data[1] = START_CODE1;
    data[2] = type;
    data[3] = 0;
    data[4] = bytes & 0xff;
    data[5] = (bytes >> 8) & 0xff;
    data[6] = (bytes >> 16) & 0xff;
    data[7] = (bytes >> 24) & 0xff;


// printf("send_packet %d send_socket=%d size=%d\n", __LINE__, send_socket, size);
// if(size >= 16)
// {
// for(int i = 0; i < 16; i++) printf("%02x ", data[i]);
// printf("\n");
// }

    int result = -1;
    pthread_mutex_lock(&www_mutex);
    if(send_socket >= 0)
    {
        result = write(send_socket, data, size);
    }
    pthread_mutex_unlock(&www_mutex);
    return result;
}

void wake_writer()
{
    sem_post(&data_ready);
}

static void* reader(void *ptr)
{
	unsigned char buffer[SOCKET_BUFSIZE];
	while(1)
	{
        struct sockaddr_in peer_addr;
        socklen_t peer_addr_len = sizeof(struct sockaddr_in);
        int bytes_read = recvfrom(recv_socket,
            buffer, 
            SOCKET_BUFSIZE, 
            0,
            (struct sockaddr *) &peer_addr, 
            &peer_addr_len);
        if(send_addr != peer_addr.sin_addr.s_addr)
        {
            printf("reader %d: new connection peer_addr=%08x\n", 
                __LINE__, 
                peer_addr.sin_addr.s_addr);
            if(send_socket >= 0)
                close(send_socket);
            send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            send_addr = peer_addr.sin_addr.s_addr;
// HACK: replace the router with the phone
            if(send_addr == TO_ADDRESS(10, 0, 10, 1)) 
                peer_addr.sin_addr.s_addr = TO_ADDRESS(10, 0, 2, 143);
            peer_addr.sin_port = htons((unsigned short)SEND_PORT);
            connect(send_socket, 
		        (struct sockaddr*)&peer_addr, 
		        peer_addr_len);
        }

        int i;
        for(i = 0; i < bytes_read; i++)
        {
            uint8_t c = buffer[i];
// user function
            server_read(c);
        }
    }
}


static void* writer(void *ptr)
{
	unsigned char buffer[SOCKET_BUFSIZE];
	int i;

	while(1)
	{
// wait for data to write
        sem_wait(&data_ready);

// async user function
        server_write();
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
    else
        printf("init_server %d: listening on port %d\n", __LINE__, RECV_PORT);
	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_t tid;
	sem_init(&data_ready, 0, 0);
	pthread_mutexattr_t attr2;
	pthread_mutexattr_init(&attr2);
    pthread_mutexattr_settype(&attr2, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&www_mutex, &attr2);

	pthread_create(&tid, 
		&attr, 
		writer, 
		0);
	pthread_create(&tid, 
		&attr, 
		reader, 
		0);
}




#endif // USE_SERVER


