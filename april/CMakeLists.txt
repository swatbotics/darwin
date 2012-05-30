cmake_minimum_required(VERSION 2.6)


include(setup_opencv.cmake)

include_directories(/opt/local/include)
find_library(CAIRO_LIBRARY cairo)

if (apple)
find_library(OPENGL_LIBRARY OpenGL)
find_library(GLUT_LIBRARY Glut)
set(OPENGL_LIBRARIES ${OPENGL_LIBRARY} )
else (apple)
find_library(OPENGL_LIBRARY GL)
find_library(GLU_LIBRARY GLU)
find_library(GLUT_LIBRARY glut)
set(OPENGL_LIBRARIES ${OPENGL_LIBRARY} ${GLU_LIBRARY})
endif (apple)

set(CMAKE_C_FLAGS "-Wall -O3 -msse2")
set(CMAKE_CXX_FLAGS ${CMAKE_C_FLAGS})

add_library(apriltagcpp 
  CameraUtil.cpp
  Geometry.cpp 
  GrayModel.cpp
  MathUtil.cpp 
  TagDetector.cpp 
  TagFamily.cpp
  TagFamilies.cpp
  UnionFindSimple.cpp
)

add_executable(camtest camtest.cpp)
target_link_libraries(camtest apriltagcpp)
add_opencv_libs(camtest)

add_executable(tagtest tagtest.cpp)
target_link_libraries(tagtest apriltagcpp)
add_opencv_libs(tagtest)

add_executable(maketags maketags.cpp)
target_link_libraries(maketags apriltagcpp ${CAIRO_LIBRARY})
add_opencv_libs(maketags)

if (GLUT_LIBRARY)
add_executable(gltest gltest.cpp)
target_link_libraries(gltest apriltagcpp ${GLUT_LIBRARY} ${OPENGL_LIBRARIES})
add_opencv_libs(gltest)
endif (GLUT_LIBRARY)