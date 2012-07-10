#include <iostream>
#include <string>
#include <unistd.h>

#include <gflags/gflags.h>

#include "localization_client.hpp"

int main(int argc, char* argv[]) {
  std::string usage;
  usage += std::string("Usage: ") + argv[0] + std::string(" [OPTIONS]");
  gflags::SetUsageMessage(usage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  LocalizationClient client;
  client.Run();
  while (true) {
    std::cout << client.GetData() << std::endl;
    usleep(10*1000);
  }
  client.Stop();
  return 0;
}
