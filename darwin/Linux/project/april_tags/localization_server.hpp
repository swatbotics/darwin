#ifndef LOCALIZATION_SERVER_HPP
#define LOCALIZATION_SERVER_HPP

#include <string>

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "april/TagDetector.h"

namespace boost {
namespace asio {
typedef boost::system::error_code error_code;
typedef boost::thread thread;
}  // namespace asio
}  // namespace boost

namespace asio = boost::asio;

class LocalizationServer {
 public:
  LocalizationServer();
  ~LocalizationServer() {}
  void Run();

 private:
  void InitializeVideoDevice();
  void RunLocalization();
  void ReceiveRequest();
  void HandleRequest(const asio::error_code& error,
                     std::size_t /*bytes_transferred*/);
  void HandleResponse(const asio::error_code& /*error*/,
                      std::size_t /*bytes_transferred*/);

  cv::VideoCapture vc_;
  TagFamily tag_family_;
  TagDetector detector_;
  boost::mutex data_mutex_;
  std::string localization_data_;
  asio::io_service io_service_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint remote_endpoint_;
  boost::array<int, 1> recv_buffer_;
};

#endif  // LOCALIZATION_SERVER_HPP
