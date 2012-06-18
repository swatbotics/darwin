#
# Makefile for April Tag testing code
#
ifeq "$(shell which clang++)" ""
  CXX = g++
else
  CXX = clang++
endif

INCLUDE_DIRS = -I../../darwin/Linux/include -I../../darwin/Framework/include
INCLUDE_DIRS += -I../..

CXX = g++
CXXFLAGS += -O -g -DLINUX -Wall -Wextra $(INCLUDE_DIRS)

LFLAGS += -lpthread -ljpeg -lrt
LFLAGS += -L../../april/build -lapriltagcpp
LFLAGS += $(shell pkg-config --libs opencv)

DARWIN_LIB = ../../darwin/Linux/lib/darwin.a
APRILTAGS_LIB = ../../april/build/libapriltagcpp.a

INCLUDES =
OBJECTS = explorer.o util.o

TARGETS = run_explorer tag_test_tool

all: $(TARGETS)

$(DARWIN_LIB):
	make -C ../../darwin/Linux/build

$(APRILTAGS_LIB):
	cd ../../april/build
	cmake ..
	cd -
	make -C ../../april/build

$(OBJECTS): %.o: %.cpp %.hpp $(INCLUDES)
	$(CXX) $(CXXFLAGS) -c $<

$(TARGETS): %: %.cpp $(DARWIN_LIB) $(OBJECTS) $(INCLUDES)
	$(CXX) $(CXXFLAGS) $@.cpp $(OBJECTS) $(DARWIN_LIB) \
	    -o $@ $(LFLAGS)

clean-junk:
	rm -f *.a *.o *~ *.so *.lo $.dSYM

clean: clean-junk
	rm -f $(TARGETS)

publish: $(TARGETS) clean-junk

backup: clean
	mkdir -p backups
	tar czvf ./backups/$(TARGET)_`date +"%Y_%m_%d_%H.%M.%S"`.tgz \
	    --exclude backups *

.PHONY: clean-junk clean publish backup
