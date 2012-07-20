# swatbotics/smawk/CMakeLists.txt

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})




add_definitions("-DCGAL_DISABLE_ROUNDING_MATH_CHECK")


add_executable(test_index test_index.cpp)
add_executable(test_smawk test_smawk.cpp)

add_executable(test_furthest test_furthest.cpp)
target_link_libraries(test_furthest ${OPENCV_LIBRARIES} cgal)
