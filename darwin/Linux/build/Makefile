###############################################################
#
# Purpose: Makefile for "DARwIn Linux Framework"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

CC = g++
AR = ar
ARFLAGS = cr

TARGET = ../lib/darwin.a

LINUX_INCLUDE_DIR = ../include
FRAMEWORK_INCLUDE_DIR = ../../Framework/include
INCLUDE_DIRS = -I$(LINUX_INCLUDE_DIR) -I$(FRAMEWORK_INCLUDE_DIR)

CXXFLAGS += -O2 -DLINUX -Wall -shared $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall -shared $(INCLUDE_DIRS)
LFLAGS += -lpthread -ldl

#TODO: figure out how to make the object files recompile when headers change
INCLUDES = $(shell find $(LINUX_INCLUDE_DIR) -name '*.h')
INCLUDES += $(shell find $(FRAMEWORK_INCLUDE_DIR) -name '*.h')

OBJS =  ../../Framework/src/CM730.o     	\
        ../../Framework/src/math/Matrix.o   \
        ../../Framework/src/math/Plane.o    \
        ../../Framework/src/math/Point.o    \
        ../../Framework/src/math/Vector.o   \
        ../../Framework/src/motion/JointData.o  	\
        ../../Framework/src/motion/Kinematics.o 	\
        ../../Framework/src/motion/MotionManager.o  \
        ../../Framework/src/motion/MotionStatus.o   \
        ../../Framework/src/motion/modules/Action.o \
        ../../Framework/src/motion/modules/Head.o   \
        ../../Framework/src/motion/modules/Walking.o\
        ../../Framework/src/vision/BallFollower.o   \
        ../../Framework/src/vision/BallTracker.o    \
        ../../Framework/src/vision/ColorFinder.o    \
        ../../Framework/src/vision/Image.o  		\
        ../../Framework/src/vision/ImgProcess.o 	\
        ../../Framework/src/minIni/minIni.o	\
        streamer/httpd.o           \
        streamer/jpeg_utils.o      \
        streamer/mjpg_streamer.o   \
        LinuxActionScript.o   \
        LinuxCamera.o   \
        LinuxCM730.o    \
        LinuxMotionTimer.o    \
        LinuxNetwork.o


.PHONY: clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(AR) $(ARFLAGS) $(TARGET) $(OBJS)

clean:
	rm -f $(OBJS) $(TARGET)
