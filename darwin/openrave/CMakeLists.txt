# swatbotics/darwin/openrave/CMakeLists.txt

include_directories(../Framework/include)
include_directories(../Linux/include)
set(DARWIN_LIBS darwin pthread jpeg rt)

add_subdirectory(cpp)
