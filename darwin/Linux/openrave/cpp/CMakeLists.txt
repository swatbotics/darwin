# swatbotics/darwin/openrave/cpp/CMakeLists.txt

if (${SDL_FOUND})
include_directories(${SDL_INCLUDE_DIRS})
link_directories(${SDL_LIBRARY_DIRS})

# Add -DIKFAST_NO_MAIN flag for the #include'd ik.cpp file.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIKFAST_NO_MAIN")

# This always goes first, as the library created in lib
# is used by the other projects
include_directories(./lib_control_robot)
add_subdirectory(lib_control_robot)

add_subdirectory(control_ee)
add_subdirectory(forwardKin)
add_subdirectory(Jacobians)
add_subdirectory(diff_kin)
endif (${SDL_FOUND})


#add_subdirectory(ik_loader)
add_subdirectory(ik_wrapper)
