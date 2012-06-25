#!/bin/bash

IMAGE_FILES=$(ls *-debug_0_0.png | sort -n)
TEMP_FILE=/tmp/montage.png
cleanup () {
    rm -f $TEMP_FILE
}
trap cleanup EXIT
montage -geometry 320x240 $IMAGE_FILES $TEMP_FILE \
    && display $TEMP_FILE
