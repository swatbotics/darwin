include_directories(${OPENCV_INCLUDE_DIRS})

add_library(control_robot
  Body.cpp
  DarwinDynamicDefinitions.cpp
  ForwardKinematics.cpp
  Joystick.cpp
  DarwinKinematics.cpp)