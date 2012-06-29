# swatbotics/darwin/openrave/cpp/control_ee/CMakeLists.txt

include_directories(${SDL_INCLUDE_DIRS})
link_directories(${SDL_LIBRARY_DIRS})

# Add -DIKFAST_NO_MAIN flag for the #include'd ik.cpp file.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIKFAST_NO_MAIN")
<<<<<<< HEAD
add_executable(control_ee Body.cpp DarwinDynamicDefinitions.cpp 
  ForwardKinematics.cpp Joystick.cpp main.cpp)
target_link_libraries(control_ee ${DARWIN_LIBS})
target_link_libraries(control_ee SDL)
=======
add_executable(control_ee Body.cpp Joystick.cpp main.cpp)
target_link_libraries(control_ee ${DARWIN_LIBS} ${SDL_LIBRARIES})
>>>>>>> d0dad7943a8e0b4be7ec505c0aa0c85ede039815
