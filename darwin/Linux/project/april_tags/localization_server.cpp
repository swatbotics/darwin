#include "localization_server.hpp"

LocalizationServer::LocalizationServer(int server_port) :
    server_(server_port),
    localizer_(),
    callback_(&server_) {
}

void LocalizationServer::Run() {
  // Mostly runs IO requests in its own thread.
  server_.Run();
  // Runs localization in the main thread; does not return.
  localizer_.Run(&callback_);
}

void LocalizationServer::Stop() {
  server_.Stop();
}
