
TARGET = ikloader

INCLUDE_DIRS = -I/usr/include/openrave-0.6/ 

CXX = g++
CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt -lopenrave0.6 -lopenrave-core0.6

SRC = ikfastloader.cpp


all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)
