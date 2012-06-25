# swatbotics/darwin/openrave/cpp/ik_wrapper/CMakeLists.txt

# Add -DIKFAST_NO_MAIN flag for the #include'd ik.cpp file.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIKFAST_NO_MAIN")
add_executable(iktest main.cpp)
