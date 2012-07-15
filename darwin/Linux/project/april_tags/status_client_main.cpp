#include <iostream>
#include <string>
#include <unistd.h>

#include <gflags/gflags.h>

#include "status_client.hpp"

DEFINE_string(server_name, "192.168.1.7",
              "IP address or DNS name of the status server to query.");
DEFINE_int32(server_port, 9000,
             "Port on the status server to connect to.");
DEFINE_double(printing_interval, 0.01,
              "Interval in seconds between printing out status data.");
DEFINE_bool(quiet, false,
            "Don't show the data received from the server.");

int main(int argc, char* argv[]) {
  std::string usage;
  usage += std::string("Usage: ") + argv[0] + std::string(" [OPTIONS]");
  gflags::SetUsageMessage(usage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  StatusClient client(FLAGS_server_name, FLAGS_server_port);
  client.Run();
  while (true) {
    if (!FLAGS_quiet) std::cout << client.GetData() << std::endl;
    usleep(1000 * 1000 * FLAGS_printing_interval);
  }
  client.Stop();
  return 0;
}
