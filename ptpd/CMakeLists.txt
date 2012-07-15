# swatbotics/ptpd/CMakeLists.txt

# Get rid of the -02 for release builds here; it just makes warnings appear.
set(CMAKE_C_FLAGS_RELEASE "")

# Definitions from the original Makefile
add_definitions("-DRUNTIME_DEBUG")
add_definitions("-DPTPD_DBG")
#add_definitions("-DPTPD_DBG2")
#add_definitions("-DPTPD_DBGV")

#add_definitions("-DPTPD_NO_DAEMON")
#add_definitions("-DDBG_SIGUSR2_CHANGE_DOMAIN")
add_definitions("-DDBG_SIGUSR2_CHANGE_DEBUG")

add_definitions("-DPTP_EXPERIMENTAL")

include_directories(dep)

add_executable(ptpd2 ptpd.c arith.c bmc.c protocol.c display.c
  dep/msg.c dep/net.c dep/servo.c dep/startup.c dep/sys.c dep/timer.c)
target_link_libraries(ptpd2 m rt)
