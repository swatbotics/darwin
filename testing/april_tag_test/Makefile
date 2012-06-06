TARGET = april_tag_test

INCLUDE_DIRS = -I../../darwin/Linux/include -I../../darwin/Framework/include
INCLUDE_DIRS += -I../..
#INCLUDE_DIRS += $(shell pkg-config --cflags opencv)
#INCLUDE_DIRS += -I/usr/include/opencv2

CXX = g++
CXXFLAGS += -O -g -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt
LFLAGS += -L../../april/build -lapriltagcpp
LFLAGS += $(shell pkg-config --libs opencv)

OBJECTS = main.o

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

darwin.a:
	make -C ../../darwin/Linux/build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../../darwin/Linux/lib/darwin.a -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/$(TARGET)_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
