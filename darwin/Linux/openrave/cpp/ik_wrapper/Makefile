TARGET = iktest

-DIK_HEADER = "\"header.h\""

INCLUDE_DIRS = -I./

CXX = g++
CXXFLAGS += -O2 -DLINUX -DIKFAST_NO_MAIN -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt 

SRC = main.cpp

#OBJ = libik.so


all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.lo

$(TARGET): $(SRC) 
	$(CXX) $(CXXFLAGS) $(SRC) $(OBJ) -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)
