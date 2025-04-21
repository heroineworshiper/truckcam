#PKGCONFIG_INCS := `pkg-config opencv --cflags`
#PKGCONFIG_LIBS := `pkg-config opencv --libs`

BAZEL_ROOT := /root/.cache/bazel/_bazel_root/510cb3499e6983b92a09020af9102ff3/

BAZEL_LIB := $(BAZEL_ROOT)execroot/_main/bazel-out/aarch64-opt/bin/tensorflow/lite/
BAZEL_INC := $(BAZEL_ROOT)external/flatbuffers/include


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
	-std=c++17 \
	-I/usr/local/include/opencv4 \
        -I/usr/include/libusb-1.0

TRUCKFLOW_LIBS := $(LIBS) -L. -ltensorflowlite -luuid -lusb-1.0 -ljpeg

TRUCKFLOW_CFLAGS := \
        -g \
	$(CFLAGS) \
	-I../tensorflow-master/ \
        -Iinclude


CC := g++

OBJS := \
	trackerserver.o
        

TRUCKCAM_OBJS := \
	truckcam.o

TRUCKFLOW_OBJS := \
	truckflow.o \
        trackerlib.o

truckcam: $(OBJS) $(TRUCKCAM_OBJS)
	$(CC) -O2 -o truckcam $(OBJS) $(TRUCKCAM_OBJS) $(LIBS)

# install dependencies for truckflow
deps:
	cp -a $(BAZEL_LIB)libtensorflowlite.so .
	cp -a $(BAZEL_INC) .

truckflow: $(OBJS) $(TRUCKFLOW_OBJS)
	$(CC) -O2 -o truckflow $(OBJS) $(TRUCKFLOW_OBJS) $(TRUCKFLOW_LIBS)

$(OBJS) $(TRUCKCAM_OBJS):
	$(CC) $(CFLAGS) -c $< -o $*.o

$(TRUCKFLOW_OBJS):
	$(CC) $(TRUCKFLOW_CFLAGS) -c $< -o $*.o

clean:
	rm -f truckcam truckflow *.o

truckflow.o: truckflow.c
truckcam.o: truckcam.c
trackerserver.o: trackerserver.c
trackerlib.o: trackerlib.c


