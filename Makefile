CFLAGS := \
	-g \
        -O2 \
        -std=c++11 \
        `pkg-config opencv --cflags` \
        -I/usr/local/include/opencv4/


LFLAGS := \
        `pkg-config opencv --libs` \
        -lpthread

CC := g++

OBJS := \
	truckcam.o \
	cam_server.o

truckcam: $(OBJS)
	$(CC) -O2 -o truckcam $(OBJS) $(LFLAGS)

$(OBJS):
	$(CC) $(CFLAGS) -c $< -o $*.o

clean:
	rm -f truckcam *.o

truckcam.o: truckcam.c
cam_server.o: cam_server.c


