#PKGCONFIG_INCS := `pkg-config opencv --cflags`
#PKGCONFIG_LIBS := `pkg-config opencv --libs`

BAZEL_PATH := `expr ../.cache/bazel/_bazel_root/*/external/`

LIBS := -L/usr/local/lib \
	-lopencv_stitching \
	-lopencv_superres \
	-lopencv_videostab \
	-lopencv_aruco \
	-lopencv_bgsegm \
	-lopencv_bioinspired \
	-lopencv_core \
	-lopencv_dnn \
	-lopencv_objdetect \
	-lopencv_imgproc \
	-lopencv_imgcodecs \
	-lpthread

CFLAGS := \
	-O2 \
	-std=c++11 \
	-I/usr/local/include/opencv2

TRUCKFLOW_LIBS := $(LIBS) -L. -ltensorflowlite -luuid

TRUCKFLOW_CFLAGS := \
	$(CFLAGS) \
	-I../tensorflow-2.8.0/ \
        -I$(BAZEL_PATH)flatbuffers/include


CC := g++

OBJS := \
	cam_server.o


TRUCKCAM_OBJS := \
	truckcam.o

TRUCKFLOW_OBJS := \
	truckflow.o

TENSORTRACK_OBJS := \
	tensortrack.o \
	trackerserver.o

truckcam: $(OBJS) $(TRUCKCAM_OBJS)
	$(CC) -O2 -o truckcam $(OBJS) $(TRUCKCAM_OBJS) $(LIBS)

truckflow: $(OBJS) $(TRUCKFLOW_OBJS)
	$(CC) -O2 -o truckflow $(OBJS) $(TRUCKFLOW_OBJS) $(TRUCKFLOW_LIBS)

tensortrack: $(TENSORTRACK_OBJS)
	$(CC) -O2 -o tensortrack $(TENSORTRACK_OBJS) $(TRUCKFLOW_LIBS)

$(OBJS) $(TRUCKCAM_OBJS):
	$(CC) $(CFLAGS) -c $< -o $*.o

$(TRUCKFLOW_OBJS):
	$(CC) $(TRUCKFLOW_CFLAGS) -c $< -o $*.o

$(TENSORTRACK_OBJS):
	$(CC) $(TRUCKFLOW_CFLAGS) -c $< -o $*.o

clean:
	rm -f truckcam truckflow *.o

tensortrack.o: tensortrack.c
trackerserver.o: trackerserver.c
truckflow.o: truckflow.c
truckcam.o: truckcam.c
cam_server.o: cam_server.c


