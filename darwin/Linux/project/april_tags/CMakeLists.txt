# swatbotics/darwin/Linux/Project/april_tags/CMakeLists.txt

set(DARWIN_APRILTAG_FILES util.cpp)

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${APRILTAG_INCLUDE_DIRS})
include_directories(${GFLAGS_INCLUDE_DIRS})

add_executable(run_explorer run_explorer.cpp explorer.cpp
  ${DARWIN_APRILTAG_FILES})
target_link_libraries(run_explorer ${DARWIN_LIBS} apriltagcpp
  ${OPENCV_LIBRARIES})

add_executable(tag_test_tool tag_test_tool.cpp
  ${DARWIN_APRILTAG_FILES})
target_link_libraries(tag_test_tool ${DARWIN_LIBS} apriltagcpp
  ${OPENCV_LIBRARIES})

find_package(Boost COMPONENTS system thread)
if (Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIR})

  add_executable(localization_server localization_server.cpp util.cpp)
  target_link_libraries(localization_server ${Boost_LIBRARIES}
    apriltagcpp ${OPENCV_LIBRARIES} gflags)

  add_executable(localization_client localization_client.cpp)
  target_link_libraries(localization_client ${Boost_LIBRARIES}
    apriltagcpp ${OPENCV_LIBRARIES} gflags)
else (Boost_FOUND)
  message("Cannot build localization binaries - need boost!")
endif (Boost_FOUND)
