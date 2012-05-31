#!/bin/bash

SCRIPT_DIR=$(dirname $0)
CODE_DIR="$SCRIPT_DIR/../darwin/Linux"
MAKEFLAGS=""
TARGETS="all"

if [[ "$1" == "clean" ]]; then
  TARGETS="clean"
fi

(cd "$CODE_DIR"
  (cd build
    echo "Building framework..."
    echo "---------------------"
    if ! make $MAKEFLAGS $TARGETS; then
      echo "\nFramework failed to build."
      exit 1
    fi
  )
  (cd project
    find . -depth -name Makefile \
      -printf "\nBuilding target %h...\n" \
      -printf "---------------------------------------\n" \
      '(' -execdir make $MAKEFLAGS $TARGETS ';' -or \
        '(' -printf "Target %h failed to build.\n" -quit ')' \
      ')' \
      || exit 1
  )
)
