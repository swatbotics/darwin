# swatbotics/darwin/Linux/project/offset_tuner/CMakeLists.txt

add_executable(offset_tuner main.cpp cmd_process.cpp)
target_link_libraries(offset_tuner ${DARWIN_LIBS} curses)

