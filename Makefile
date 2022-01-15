#PKGCONFIG_INCS := `pkg-config opencv --cflags`
#PKGCONFIG_LIBS := `pkg-config opencv --libs`

PKGCONFIG_INCS := -I/usr/local/include/opencv4
PKGCONFIG_LIBS := -L/usr/local/lib \
	-lopencv_stitching \
	-lopencv_superres \
	-lopencv_videostab \
	-lopencv_aruco \
	-lopencv_bgsegm \
	-lopencv_bioinspired \
	-lopencv_core \
	-lopencv_objdetect \
	-lopencv_imgproc \
	-lopencv_imgcodecs


CFLAGS := \
	-g \
	-O2 \
	-std=c++11 \
	$(PKGCONFIG_INCS)


LFLAGS := \
	$(PKGCONFIG_LIBS) \
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


