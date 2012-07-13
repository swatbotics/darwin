#ifndef LOCALIZATION_SERVER_HPP
#define LOCALIZATION_SERVER_HPP

#include <string>

#include "localizer.hpp"
#include "status_server.hpp"

class LocalizationServer {
 public:
  LocalizationServer(int server_port);
  ~LocalizationServer() {}
  void Run();
  void Stop();  // Currently unused; Run() never exits.

 private:
  struct LocalizationServerCallback : Localizer::DataCallbackFunc {
    LocalizationServerCallback(StatusServer* obj) : obj_(obj) {}
    ~LocalizationServerCallback() {}
    void operator() (const std::string& data) {
      obj_->SetData(data);
    }
    StatusServer* obj_;
  };

  StatusServer server_;
  Localizer localizer_;
  LocalizationServerCallback callback_;
};

#endif  // LOCALIZATION_SERVER_HPP
