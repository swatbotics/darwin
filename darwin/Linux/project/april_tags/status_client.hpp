#ifndef STATUS_CLIENT_HPP
#define STATUS_CLIENT_HPP

#include <string>
#include <time.h>

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
  StatusClient();
  ~StatusClient() {}
  std::string GetData(struct timespec* ts=NULL);
  void Run();
  void Stop();

 private:
  static const int kRecvBufferSize;
  void SubscribeData();
  void HandleSubscribe(const asio::error_code& error,
                       std::size_t bytes_transferred);
  void SendRequest();
  void HandleRequest(const asio::error_code& /*error*/,
                     std::size_t /*bytes_transferred*/);
  void ReceiveResponse();
  void HandleResponse(const asio::error_code& error,
                      std::size_t bytes_transferred);
  void ParseDataFromBuffer(const std::vector<char>& buffer,
                           size_t num_bytes);
  void MeasureDelay(const std::string& server_time);

  boost::mutex data_mutex_;
  struct timespec data_timestamp_;
  std::string data_;
  asio::io_service io_service_;
  long packet_seq_num_;
  udp::socket socket_;
  udp::socket multicast_socket_;
  udp::endpoint remote_endpoint_;
  udp::endpoint multicast_endpoint_;
  std::vector<char> send_buffer_;
  std::vector<char> recv_buffer_;
  std::vector<char> multicast_recv_buffer_;
  struct timespec request_send_time_;
  asio::io_service::work* worker_;
  asio::thread io_thread_;
};

#endif  // STATUS_CLIENT_HPP
