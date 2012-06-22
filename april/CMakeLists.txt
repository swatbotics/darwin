# swatbotics/april/CMakeLists.txt

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Set this globally so it can be picked up by stuff in /darwin.
# Using the project source dir for now, so that the april tag #includes
# get prefixed with "april/" for clarity.
# TODO: move apriltag header files into /april/include/april?
set(APRILTAG_INCLUDE_DIRS ${PROJECT_SOURCE_DIR} CACHE INTERNAL "" FORCE)

include_directories(${OPENCV_INCLUDE_DIRS})

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
target_link_libraries(camtest apriltagcpp ${OPENCV_LIBRARIES})

add_executable(tagtest tagtest.cpp)
target_link_libraries(tagtest apriltagcpp ${OPENCV_LIBRARIES})

add_executable(maketags maketags.cpp)
target_link_libraries(maketags apriltagcpp ${CAIRO_LIBRARIES} ${OPENCV_LIBRARIES})

if (GLUT_LIBRARY)
add_executable(gltest gltest.cpp)
target_link_libraries(gltest apriltagcpp ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} ${OPENCV_LIBRARIES})
add_opencv_libs(gltest)
endif (GLUT_LIBRARY)
