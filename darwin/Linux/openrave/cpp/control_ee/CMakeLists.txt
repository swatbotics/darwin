# swatbotics/darwin/openrave/cpp/control_ee/CMakeLists.txt


add_executable(control_ee main.cpp)
target_link_libraries(control_ee ${DARWIN_LIBS} ${SDL_LIBRARIES} control_robot)

