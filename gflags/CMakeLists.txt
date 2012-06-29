# swatbotics/gflags/CMakeLists.txt

include_directories(include)
include_directories(src)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-sign-compare")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")

add_library(gflags
  src/config.h
  src/gflags.cc
  src/gflags_completions.cc
  src/gflags_reporting.cc
  src/mutex.h
  src/util.h
)

set(GFLAGS_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include"
  CACHE INTERNAL "" FORCE)
