# swatbotics/darwin/openrave/cpp/control_ee/CMakeLists.txt

# Add -DIKFAST_NO_MAIN flag for the #include'd ik.cpp file.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIKFAST_NO_MAIN")
add_executable(control_ee Body.cpp DarwinDynamicDefinitions.cpp 
  ForwardKinematics.cpp Joystick.cpp main.cpp)
target_link_libraries(control_ee ${DARWIN_LIBS})
target_link_libraries(control_ee SDL)