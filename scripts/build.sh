#!/bin/bash

# Get the full path of DARWIN_ROOT.
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
DARWIN_ROOT=$(dirname "$SCRIPT_DIR")

# Create a 'build' directory to use.
BUILD_DIR="$DARWIN_ROOT/build"
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
fi

# Change to the build directory, run cmake, then run make.
(
cd "$BUILD_DIR"
cmake .. && make "$@"
)
