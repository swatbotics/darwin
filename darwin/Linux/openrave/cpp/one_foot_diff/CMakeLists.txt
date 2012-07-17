
add_executable(one_foot_diff main.cpp)
target_link_libraries(one_foot_diff control_robot ${DARWIN_LIBS} ${SDL_LIBRARIES} ${OPENCV_LIBRARIES})

