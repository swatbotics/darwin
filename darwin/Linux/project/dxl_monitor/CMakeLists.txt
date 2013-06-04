# swatbotics/darwin/Linux/project/dxl_monitor/CMakeLists.txt

add_executable(dxl_monitor main.cpp cmd_process.cpp)
target_link_libraries(dxl_monitor ${DARWIN_LIBS})

