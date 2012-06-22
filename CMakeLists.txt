# swatbotics/CMakeLists.txt

cmake_minimum_required(VERSION 2.8)

project(swatbotics)

include(FindPkgConfig)

pkg_check_modules(CAIRO REQUIRED cairo)
pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)

if (APPLE)
find_library(OPENGL_LIBRARY OpenGL)
find_library(GLUT_LIBRARY Glut)
set(OPENGL_LIBRARIES ${OPENGL_LIBRARY} )
else (APPLE)
find_library(OPENGL_LIBRARY GL)
find_library(GLU_LIBRARY GLU)
find_library(GLUT_LIBRARY glut)
set(OPENGL_LIBRARIES ${OPENGL_LIBRARY} ${GLU_LIBRARY})
endif (APPLE)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

set(CMAKE_C_FLAGS "-Wall -g")
set(CMAKE_CXX_FLAGS "-Wall -g")

set(CMAKE_C_FLAGS_DEBUG "-O")
set(CMAKE_CXX_FLAGS_DEBUG "-O")

set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

add_subdirectory(darwin)
add_subdirectory(april)
