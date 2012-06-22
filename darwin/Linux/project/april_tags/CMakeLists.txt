# swatbotics/darwin/Linux/Project/april_tags/CMakeLists.txt

set(DARWIN_APRILTAG_FILES util.cpp)

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${APRILTAG_INCLUDE_DIRS})

add_executable(run_explorer run_explorer.cpp explorer.cpp
  ${DARWIN_APRILTAG_FILES})
target_link_libraries(run_explorer ${DARWIN_LIBS} apriltagcpp
  ${OPENCV_LIBRARIES})

add_executable(tag_test_tool tag_test_tool.cpp
  ${DARWIN_APRILTAG_FILES})
target_link_libraries(tag_test_tool ${DARWIN_LIBS} apriltagcpp
  ${OPENCV_LIBRARIES})


