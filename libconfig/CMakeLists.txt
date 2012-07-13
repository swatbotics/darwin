# swatbotics/gflags/CMakeLists.txt

include_directories(include)
#include_directories(src)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-sign-compare")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")

if (APPLE)
  add_definitions("-DHAVE_XLOCALE_H")
endif(APPLE)

add_library(config++
  src/grammar.c
  src/libconfig.c
  src/libconfigcpp.c++
  src/scanctx.c
  src/scanner.c
  src/strbuf.c
)

set(LIBCONFIG_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include"
  CACHE INTERNAL "" FORCE)
