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

find_package(Boost COMPONENTS system thread)
if (Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIR})

  add_executable(localization_server localization_server.cpp
    localization_server_main.cpp status_server.cpp localizer.cpp util.cpp)
  target_link_libraries(localization_server ${Boost_LIBRARIES}
    apriltagcpp ${OPENCV_LIBRARIES} gflags config++ rt)

  add_executable(status_client status_client.cpp status_client_main.cpp)
  target_link_libraries(status_client ${Boost_LIBRARIES} gflags rt)

  add_executable(run_localized_explorer localized_explorer.cpp
    status_client.cpp pid_controller.cpp util.cpp)
  target_link_libraries(run_localized_explorer ${DARWIN_LIBS}
    ${OPENCV_LIBRARIES} ${Boost_LIBRARIES} gflags)

else (Boost_FOUND)
  message("Cannot build localization binaries - need boost!")
endif (Boost_FOUND)
