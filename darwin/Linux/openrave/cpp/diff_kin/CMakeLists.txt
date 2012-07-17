
add_executable(diffKin main.cpp)
target_link_libraries(diffKin ${DARWIN_LIBS} ${OPENCV_LIBRARIES} control_robot)
