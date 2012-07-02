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
using asio::ip::udp;

class LocalizationServer {
 public:
  LocalizationServer();
  ~LocalizationServer() {}
  void Run();

 private:
  struct TagInfo {
    size_t id;
    cv::Point2d center;
    cv::Mat_<double> raw_r;
    cv::Mat_<double> raw_t;
    cv::Mat_<double> t;
  };
  typedef std::map<size_t, TagInfo> TagInfoMap;
  struct ReferenceTagCoords {
    size_t id;
    double pos[3];
  };
  static const double kObjectTagSize = 0.10;
  static const double kReferenceTagSize = 0.10;
  static const double kReferenceTagInterval = 0.605;
  static const ReferenceTagCoords reference_tag_coords_[];

  void InitializeVideoDevice();
  void RunLocalization();
  void RunTagDetection();
  TagInfo GetTagInfo(const TagDetection& detection, double tag_size);
  void FindGlobalTransform();
  void LocalizeObjects();
  void GenerateLocalizationData();
  void ReceiveRequest();
  void HandleRequest(const asio::error_code& error,
                     std::size_t /*bytes_transferred*/);
  void HandleResponse(const asio::error_code& /*error*/,
                      std::size_t /*bytes_transferred*/);

  cv::VideoCapture vc_;
  TagFamily tag_family_;
  TagDetector detector_;
  TagDetectionArray detections_;
  TagInfoMap ref_tags_;
  TagInfoMap obj_tags_;
  cv::Mat_<double> global_transform_;
  boost::mutex data_mutex_;
  std::string localization_data_;
  asio::io_service io_service_;
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<int, 1> recv_buffer_;
};

#endif  // LOCALIZATION_SERVER_HPP
