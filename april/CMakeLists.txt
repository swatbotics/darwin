# swatbotics/april/CMakeLists.txt

set(CMAKE_C_FLAGS_RELEASE "-O3 -frounding-math")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -frounding-math")

# Set this globally so it can be picked up by stuff in /darwin.
# Using the project source dir for now, so that the april tag #includes
# get prefixed with "april/" for clarity.
# TODO: move apriltag header files into /april/include/april?
set(APRILTAG_INCLUDE_DIRS ${PROJECT_SOURCE_DIR} CACHE INTERNAL "" FORCE)

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

include_directories(${GFLAGS_INCLUDE_DIRS})

if (CGAL_LIBRARY)
  set(AT_LIBS apriltagcpp ${OPENCV_LIBRARIES} ${CGAL_LIBRARY})
  add_definitions("-DHAVE_CGAL")
else (CGAL_LIBRARY)
  set(AT_LIBS apriltagcpp ${OPENCV_LIBRARIES})
endif (CGAL_LIBRARY)

add_library(apriltagcpp 
  CameraUtil.cpp
  DebugImage.cpp
  Geometry.cpp 
  GrayModel.cpp
  MathUtil.cpp 
  Refine.cpp
  TagDetector.cpp 
  TagFamily.cpp
  TagFamilies.cpp
  UnionFindSimple.cpp
)

add_executable(camtest camtest.cpp)
target_link_libraries(camtest ${AT_LIBS} gflags)

add_executable(tagtest tagtest.cpp)
target_link_libraries(tagtest ${AT_LIBS})

add_executable(quadtest quadtest.cpp)
target_link_libraries(quadtest ${AT_LIBS})

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate ${AT_LIBS} gflags)

add_executable(maketags maketags.cpp)
target_link_libraries(maketags ${CAIRO_LIBRARIES} ${AT_LIBS})

if (GLUT_LIBRARY)
add_executable(gltest gltest.cpp)
target_link_libraries(gltest ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} ${AT_LIBS})
endif (GLUT_LIBRARY)
