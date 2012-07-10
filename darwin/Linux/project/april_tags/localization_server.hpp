#ifndef LOCALIZATION_SERVER_HPP
#define LOCALIZATION_SERVER_HPP

#include <string>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include "localizer.hpp"

namespace boost {
namespace asio {
typedef boost::system::error_code error_code;
typedef boost::thread thread;
}  // namespace asio
}  // namespace boost

namespace asio = boost::asio;
using asio::ip::udp;

class LocalizationServer {
 public:
  LocalizationServer(int server_port);
  ~LocalizationServer() {}
  void Run();
  void RunLocalizer();
  void Stop();

 private:
  struct LocalizationServerCallback : Localizer::DataCallbackFunc {
    LocalizationServerCallback(LocalizationServer* obj) : obj_(obj) {}
    ~LocalizationServerCallback() {}
    void operator() (const std::string& data) {
      obj_->DataRetrievalCallback(data);
    }
    LocalizationServer* obj_;
  };

  static const int kDefaultServerPort;

  void DataRetrievalCallback(const std::string& data);
  void ReceiveRequest();
  void HandleRequest(const asio::error_code& error,
                     std::size_t /*bytes_transferred*/);
  void HandleResponse(const asio::error_code& /*error*/,
                      std::size_t /*bytes_transferred*/);

  int server_port_;
  Localizer localizer_;
  LocalizationServerCallback callback_;
  boost::mutex data_mutex_;
  std::string localization_data_;
  asio::io_service io_service_;
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<int, 1> recv_buffer_;
  asio::thread io_thread_;
};

#endif  // LOCALIZATION_SERVER_HPP
