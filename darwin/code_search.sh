#!/bin/bash
# Convenience script to search the DARwIn-OP source code repository.

# Get the directory that the script is in.
SCRIPT_DIR="$(dirname $0)"

# Locations of source files to search, relative to SCRIPT_DIR.
FRAMEWORK_INCS="$SCRIPT_DIR"/Framework/include
FRAMEWORK_SRCS="$SCRIPT_DIR"/Framework/src
LINUX_INCS="$SCRIPT_DIR"/Linux/include
LINUX_SRCS="$SCRIPT_DIR"/Linux/build
TUTORIALS="$SCRIPT_DIR"/Linux/project/tutorial

# Require an argument (otherwise grep misbehaves).
if (($# < 1)); then
    echo "Usage: $0 <SEARCH_TERM> [extra arguments to grep]"
    exit 1
fi

# Pop first argument off of args to use as search terms.
terms="$1"
shift 1

# Run grep over the provided source file locations, excluding SVN stuff,
# compiled object files, compiled libraries, and binaries (-I).  Extra
# arguments to the script get passed directly to grep.
grep --color=auto -R "$terms" \
    --exclude-dir='.svn' --exclude='*.o' --exclude='*.a' -I "$@" \
    $FRAMEWORK_INCS $FRAMEWORK_SRCS $LINUX_INCS $LINUX_SRCS $TUTORIALS
