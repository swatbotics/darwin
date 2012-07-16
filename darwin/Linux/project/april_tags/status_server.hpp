#ifndef STATUS_SERVER_HPP
#define STATUS_SERVER_HPP

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

class StatusServer {
 public:
  StatusServer();
  ~StatusServer() {}
  void SetData(const std::string& data);
  void Run();
  void Stop();

 private:
  void PublishData();
  void RespondData();
  void SendData(const udp::endpoint& destination, udp::socket& socket);
  void ReceiveRequest();
  void HandleRequest(const asio::error_code& error,
                     std::size_t /*bytes_transferred*/);
  void HandleSend(const asio::error_code& error,
                  std::size_t bytes_transferred);

  boost::mutex data_mutex_;
  std::string data_;
  asio::io_service io_service_;
  udp::socket socket_;
  udp::socket multicast_socket_;
  udp::endpoint remote_endpoint_;
  udp::endpoint multicast_endpoint_;
  boost::array<int, 1> recv_buffer_;
  asio::io_service::work* worker_;
  asio::thread io_thread_;
};

#endif  // STATUS_SERVER_HPP
