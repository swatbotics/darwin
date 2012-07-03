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
    double size;
    cv::Point2d center;
    cv::Mat_<double> raw_r;
    cv::Mat_<double> raw_t;
    cv::Mat_<double> inv_t;
    cv::Mat_<double> t;
  };
  typedef std::map<size_t, TagInfo> TagInfoMap;
  struct ReferenceTagCoords {
    size_t id;
    double pos[3];
  };
  struct ReferenceTagSystem {
    size_t origin;
    size_t primary;
    size_t secondary;
  };
  static const double kObjectTagSize;
  static const double kReferenceTagSize;
  static const double kReferenceTagInterval = 0.605;
  static const ReferenceTagCoords reference_tag_coords_[];
  static const ReferenceTagSystem reference_tag_system_;

  void InitializeVideoDevice();
  void RunLocalization();
  void RunTagDetection();
  TagInfo GetTagInfo(const TagDetection& detection, double tag_size);
  void FindGlobalTransform();
  cv::Mat_<double> TransformToGlobal(const cv::Mat_<double>& vec);
  cv::Mat_<double> TransformToCamera(const cv::Mat_<double>& vec);
  void LocalizeObjects();
  void GenerateLocalizationData();
  void ReceiveRequest();
  void HandleRequest(const asio::error_code& error,
                     std::size_t /*bytes_transferred*/);
  void HandleResponse(const asio::error_code& /*error*/,
                      std::size_t /*bytes_transferred*/);

  cv::VideoCapture vc_;
  cv::Mat frame_;
  cv::Point2d optical_center_;
  TagFamily tag_family_;
  TagDetector detector_;
  TagDetectionArray detections_;
  TagInfoMap ref_tags_;
  TagInfoMap obj_tags_;
  cv::Mat_<double> global_translation_;
  cv::Mat_<double> global_rotation_;
  boost::mutex data_mutex_;
  std::string localization_data_;
  asio::io_service io_service_;
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<int, 1> recv_buffer_;
};

#endif  // LOCALIZATION_SERVER_HPP
