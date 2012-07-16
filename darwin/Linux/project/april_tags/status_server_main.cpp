#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

#include <gflags/gflags.h>

#include "status_server.hpp"

DEFINE_double(update_interval, 0.1,
              "Update interval of dummy data in seconds.");
DEFINE_bool(quiet, false,
            "Don't show the messages that the server is sending.");

int main(int argc, char* argv[]) {
  std::string usage;
  usage += std::string("Usage: ") + argv[0] + std::string(" [OPTIONS]");
  gflags::SetUsageMessage(usage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  StatusServer server;
  server.Run();
  std::stringstream ss;
  int message_count = 0;
  while (true) {
    ss.str(std::string());
    ss << message_count++;
    server.UpdateData(ss.str());
    if (!FLAGS_quiet) std::cout << "Data = " << ss.str() << "\n";
    usleep(1000 * 1000 * FLAGS_update_interval);
  }
  server.Stop();
  return 0;
}
