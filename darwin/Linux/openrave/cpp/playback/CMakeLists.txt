
add_executable(playback playback.cpp)
target_link_libraries(playback control_robot ${DARWIN_LIBS} ${SDL_LIBRARIES})

