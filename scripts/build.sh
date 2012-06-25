#!/bin/bash

# Get script name, dir, and full path of DARWIN_ROOT.
SCRIPT_NAME=$(basename "${BASH_SOURCE[0]}")
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
DARWIN_ROOT=$(dirname "$SCRIPT_DIR")

BUILD_DIR_NAME_DEFAULT="build"
BUILD_DIR_NAME_DEBUG="build-debug"
BUILD_DIR_NAME="$BUILD_DIR_NAME_DEFAULT"

CMAKE_FLAGS=""

clean_build_dirs () {
    rm -rf "$DARWIN_ROOT/$BUILD_DIR_NAME_DEFAULT"
    rm -rf "$DARWIN_ROOT/$BUILD_DIR_NAME_DEBUG"
}

run_build () {
    # Create a 'build' directory to use.
    BUILD_DIR="$DARWIN_ROOT/$BUILD_DIR_NAME"
    if [ ! -d "$BUILD_DIR" ]; then
        mkdir -p "$BUILD_DIR"
    fi

    # Change to the build directory, run cmake, then run make.
    (
    cd "$BUILD_DIR"
    if [ -n "$CMAKE_FLAGS" ]; then
	cmake .. "$CMAKE_FLAGS" && make "$@"
    else
	cmake .. && make "$@"
    fi
    )
}

case "$SCRIPT_NAME" in
    build | build.sh)
	run_build
	;;
    build-clean)
	clean_build_dirs
	;;
    build-scratch)
	clean_build_dirs
	run_build
	;;
    build-debug)
	BUILD_DIR_NAME="$BUILD_DIR_NAME_DEBUG"
	CMAKE_FLAGS="-DCMAKE_BUILD_TYPE=Debug"
	run_build
	;;
    *)
	echo "Build script called with unrecognized name!"
	exit 1
	;;
esac
