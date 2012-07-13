#include <string>

#include <gflags/gflags.h>

#include "localization_server.hpp"

DEFINE_int32(server_port, 9000,
             "Port on which to run UDP localization service.");

int main(int argc, char* argv[]) {
  std::string usage;
  usage += std::string("Usage: ") + argv[0] + std::string(" [OPTIONS]");
  gflags::SetUsageMessage(usage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  LocalizationServer server(FLAGS_server_port);
  server.Run();
  return 0;
}
