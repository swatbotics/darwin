#ifndef STATUS_CLIENT_HPP
#define STATUS_CLIENT_HPP

#include <string>

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

namespace boost {
namespace asio {
typedef boost::system::error_code error_code;
typedef boost::thread thread;
}  // namespace asio
}  // namespace boost

namespace asio = boost::asio;
using asio::ip::udp;

class StatusClient {
 public:
  StatusClient(std::string server_name, int server_port);
  ~StatusClient() {}
  void Run();
  void Stop();
  std::string GetData();

 private:
  static const int kDefaultServerPort;
  static const int kRecvBufferSize;

  void SendRequest();
  void HandleRequest(const asio::error_code& /*error*/,
                     std::size_t /*bytes_transferred*/);
  void HandleResponse(const asio::error_code& error,
                      std::size_t bytes_transferred);
  void MeasureLatency(const std::string& timestamp);

  std::string server_name_;
  int server_port_;
  boost::mutex data_mutex_;
  std::string data_;
  asio::io_service io_service_;
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  std::vector<char> send_buffer_;
  std::vector<char> recv_buffer_;
  asio::thread io_thread_;
};

#endif  // STATUS_CLIENT_HPP
