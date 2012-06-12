
TARGET = control_ee

INCLUDE_DIRS = -I../../../Linux/include -I../../../Framework/include -I/.

CXX = g++
CXXFLAGS += -O2 -DLINUX -DIKFAST_NO_MAIN -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt

OBJECTS = Body.o

SRC = main.cpp

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

darwin.a:
	make -C ../../../Linux/build

Body.o: 
	$(CXX) -c $(CXXFLAGS) $(LFLAGS) $(LINK_DIRS) ../../../Linux/lib/darwin.a Body.cpp

$(TARGET): darwin.a $(OBJECTS) $(SRC)
	$(CXX) $(CXXFLAGS) $(OBJECTS) $(SRC) ../../../Linux/lib/darwin.a -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/zero_finding__`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
