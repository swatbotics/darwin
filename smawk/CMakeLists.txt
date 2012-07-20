# swatbotics/smawk/CMakeLists.txt

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})


if(APPLE)
set(CGAL_DIR "/opt/local/lib/cmake")
endif(APPLE)

find_package(CGAL QUIET)
if (CGAL_FOUND)
  include(${CGAL_DIR}/UseCGAL.cmake)
  set(AT_LIBS "${AT_LIBS}" ${CGAL_LIBRARY})
  add_definitions("-DCGAL_DISABLE_ROUNDING_MATH_CHECK")
endif ()

string (REPLACE "-frounding-math" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

add_executable(test_index test_index.cpp)
add_executable(test_smawk test_smawk.cpp)

add_executable(test_furthest test_furthest.cpp)
target_link_libraries(test_furthest ${OPENCV_LIBRARIES} ${CGAL_LIBRARY})
