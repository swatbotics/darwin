#include <string>

#include <gflags/gflags.h>

#include "localization_server.hpp"

int main(int argc, char* argv[]) {
  std::string usage;
  usage += std::string("Usage: ") + argv[0] + std::string(" [OPTIONS]");
  gflags::SetUsageMessage(usage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  LocalizationServer server;
  server.Run();
  return 0;
}
