# swatbotics/darwin/openrave/cpp/CMakeLists.txt


if (${SDL_FOUND})
add_subdirectory(control_ee)
endif (${SDL_FOUND})

#add_subdirectory(ik_loader)
add_subdirectory(ik_wrapper)
#add_subdirectory(forwardKin)
