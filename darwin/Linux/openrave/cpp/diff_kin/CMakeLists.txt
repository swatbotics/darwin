
add_executable(diffKin main.cpp)
target_link_libraries(diffKin control_robot ${DARWIN_LIBS} ${OPENCV_LIBRARIES})
