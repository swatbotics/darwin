###############################################################
#
# Purpose: Makefile for "zero_finding"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = zero_finding

INCLUDE_DIRS = -I../../../include -I../../../../Framework/include

CXX = g++
CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt

OBJECTS =   main.o Body.o

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

darwin.a:
	make -C ../../../build

Body.o: 
	$(CXX) -c $(CXXFLAGS) $(LFLAGS) $(LINK_DIRS) ../../../lib/darwin.a Body.cpp

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../../../lib/darwin.a -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/zero_finding__`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
