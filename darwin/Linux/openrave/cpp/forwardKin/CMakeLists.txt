# swatbotics/darwin/openrave/cpp/forwardKin/CMakeLists.txt

add_executable(forwardKin main.cpp)
target_link_libraries(forwardKin ${DARWIN_LIBS} control_robot)
