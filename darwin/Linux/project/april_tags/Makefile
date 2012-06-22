#
# Makefile for April Tag testing code
#
ifeq "$(shell which clang++)" ""
  CXX = g++
else
  CXX = clang++
endif

PATH_TO_ROOT = ../../../..

INCLUDE_DIRS = -I$(PATH_TO_ROOT)/darwin/Linux/include
INCLUDE_DIRS += -I$(PATH_TO_ROOT)/darwin/Framework/include
INCLUDE_DIRS += -I$(PATH_TO_ROOT)

CXX = g++
CXXFLAGS += -O -g -DLINUX -Wall -Wextra $(INCLUDE_DIRS)

LFLAGS += -lpthread -ljpeg -lrt
LFLAGS += -L$(PATH_TO_ROOT)/april/build -lapriltagcpp
LFLAGS += $(shell pkg-config --libs opencv)

DARWIN_LIB = $(PATH_TO_ROOT)/darwin/Linux/lib/darwin.a
APRILTAGS_LIB = $(PATH_TO_ROOT)/april/build/libapriltagcpp.a

INCLUDES =
OBJECTS = explorer.o util.o

TARGETS = run_explorer tag_test_tool

all: $(TARGETS)

$(DARWIN_LIB):
	make -C $(PATH_TO_ROOT)/darwin/Linux/build

$(APRILTAGS_LIB):
	cd $(PATH_TO_ROOT)/april/build
	cmake ..
	cd -
	make -C $(PATH_TO_ROOT)/april/build

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
