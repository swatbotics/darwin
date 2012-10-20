# swatbotics/darwin/Linux/Project/april_tags/CMakeLists.txt

set(DARWIN_APRILTAG_FILES util.cpp)

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${APRILTAG_INCLUDE_DIRS})
include_directories(${GFLAGS_INCLUDE_DIRS})
include_directories(${LIBCONFIG_INCLUDE_DIRS})

add_executable(run_explorer run_explorer.cpp explorer.cpp
  ${DARWIN_APRILTAG_FILES})
target_link_libraries(run_explorer ${DARWIN_LIBS} apriltagcpp
  ${OPENCV_LIBRARIES})

add_executable(tag_test_tool tag_test_tool.cpp
  ${DARWIN_APRILTAG_FILES})
target_link_libraries(tag_test_tool ${DARWIN_LIBS} apriltagcpp
  ${OPENCV_LIBRARIES})

add_executable(test_camera_latency test_camera_latency.cpp)
target_link_libraries(test_camera_latency ${OPENCV_LIBRARIES} pthread gflags)

add_executable(timestamp_test timestamp_test.cpp timestamp.cpp)
target_link_libraries(timestamp_test rt)

find_package(Boost COMPONENTS system thread)
if (Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIR})

  add_executable(status_server status_server.cpp status_server_main.cpp
    timestamp.cpp)
  target_link_libraries(status_server ${Boost_LIBRARIES} gflags rt)

  add_executable(status_client status_client.cpp status_client_main.cpp
    timestamp.cpp)
  target_link_libraries(status_client ${Boost_LIBRARIES} gflags rt)

  add_executable(localization_server localization_server.cpp
    localization_server_main.cpp status_server.cpp localizer.cpp
    timestamp.cpp util.cpp)
  target_link_libraries(localization_server ${Boost_LIBRARIES}
    apriltagcpp ${OPENCV_LIBRARIES} gflags config++ rt)

  add_executable(run_localized_explorer localized_explorer.cpp
    status_client.cpp pid_controller.cpp timestamp.cpp util.cpp)
  target_link_libraries(run_localized_explorer ${DARWIN_LIBS}
    ${OPENCV_LIBRARIES} ${Boost_LIBRARIES} gflags rt)

else (Boost_FOUND)
  message("Cannot build localization binaries - need boost!")
endif (Boost_FOUND)
