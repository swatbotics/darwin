#include "localization_server.hpp"

#include <iostream>
#include <sstream>

#include <april/CameraUtil.h>
#include <boost/bind.hpp>
#include <gflags/gflags.h>

#include "util.hpp"

DEFINE_int32(device_num, 0,
             "Device number for video device to parse for tag information.");
DEFINE_double(focal_length, 500,
              "Focal length of the camera represented by the video device.");
DEFINE_string(tag_family, "Tag36h11",
              "Tag family to use for detections.");
DEFINE_int32(frame_width, 640, "Desired video frame width.");
DEFINE_int32(frame_height, 480, "Desired video frame height.");
DEFINE_int32(server_port, 9000,
             "Port on which to run UDP localization service.");

#define DEBUG false

namespace asio = boost::asio;

const LocalizationServer::ReferenceTagCoords
LocalizationServer::reference_tag_coords_[] = {
  {0, {0, 0, 0}},
  {1, {0, kReferenceTagInterval, 0}},
  {2, {kReferenceTagInterval, 0, 0}},
  {3, {kReferenceTagInterval, kReferenceTagInterval, 0}}
};

LocalizationServer::LocalizationServer() :
    vc_(),
    tag_family_(FLAGS_tag_family),
    detector_(tag_family_),
    detections_(),
    ref_tags_(),
    obj_tags_(),
    global_transform_(3, 3),
    data_mutex_(),
    localization_data_(),
    io_service_(),
    socket_(io_service_),
    remote_endpoint_(),
    recv_buffer_()
{
  //  detector_.params.segDecimate = true;
  //  detector_.params.thetaThresh = 25;
  //  detector_.params.refineCornersSubPix = true;
  InitializeVideoDevice();
}

void LocalizationServer::InitializeVideoDevice() {
  vc_.open(FLAGS_device_num);
  if (!vc_.isOpened()) {
    std::cerr << "Could not open video device!\n";
    exit(1);
  }
  vc_.set(CV_CAP_PROP_FRAME_WIDTH, FLAGS_frame_width);
  vc_.set(CV_CAP_PROP_FRAME_HEIGHT, FLAGS_frame_height);
  std::cout << "Camera resolution: "
            << vc_.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
            << vc_.get(CV_CAP_PROP_FRAME_HEIGHT) << "\n";
}

void LocalizationServer::RunLocalization() {
  double lasttime = get_time_as_double();
  while (true) {
    RunTagDetection();
    FindGlobalTransform();
    LocalizeObjects();
    GenerateLocalizationData();
    double thistime = get_time_as_double();
    printf("FPS: %d\n", (int) (1 / (thistime - lasttime)));
    lasttime = thistime;
  }
}

void LocalizationServer::RunTagDetection() {
  cv::Mat frame;
  vc_ >> frame;
  if (frame.empty()) {
    std::cerr << "Got empty frame!\n";
    exit(1);
  }
  const cv::Point2d optical_center(frame.cols * 0.5, frame.rows * 0.5);
  detector_.process(frame, optical_center, detections_);
  const size_t reference_tags_num = (sizeof(reference_tag_coords_) /
                                     sizeof(reference_tag_coords_[0]));
  ref_tags_.clear();
  obj_tags_.clear();
  for (size_t i = 0; i < detections_.size(); ++i) {
    const TagDetection& d = detections_[i];
    bool is_ref_tag = false;
    size_t j = 0;
    for (j = 0; j < reference_tags_num; ++j) {
      if ((is_ref_tag = (reference_tag_coords_[j].id == d.id))) break;
    }
    if (is_ref_tag) {
      TagInfo tag = GetTagInfo(d, kReferenceTagSize);
      tag.t.create(3, 1);
      for (int k = 0; k < tag.t.rows; ++k) {
        tag.t[k][0] = reference_tag_coords_[j].pos[k];
      }
      ref_tags_[tag.id] = tag;
    } else {
      TagInfo tag = GetTagInfo(d, kObjectTagSize);
      obj_tags_[tag.id] = tag;
    }
  }
}

LocalizationServer::TagInfo LocalizationServer::GetTagInfo(
    const TagDetection& detection, double tag_size) {
  TagInfo tag;
  tag.id = detection.id;
  tag.center = detection.cxy;
  const double f = FLAGS_focal_length;
  CameraUtil::homographyToPoseCV(f, f, tag_size,
                                 detection.homography, tag.raw_r, tag.raw_t);
  return tag;
}

void LocalizationServer::FindGlobalTransform() {
  if (ref_tags_.size() < 3) {
    // TODO: Develop strategies for localizing with only 1-2 tags, using
    // the tag rotation matrices. Or:
    // TODO: Implement a "memory" for previously seen reference tags, to
    // avoid having to degrade localization when tags flicker out?
    std::cout << "Cannot localize with less than 3 tags!\n";
    return;
  }
  cv::Mat_<double> ref_points(3, 3);
  cv::Mat_<double> cam_points(3, 3);
  TagInfoMap::const_iterator it = ref_tags_.begin();
  for (size_t i = 0; i < 3; ++i) {
    const TagInfo& ref_tag = (it++)->second;
    for (size_t j = 0; j < 3; ++j) {
      ref_points[j][i] = ref_tag.t[j][0];
      cam_points[j][i] = ref_tag.raw_t[j][0];
    }
  }
  // TODO: This transform generation scheme is broken because all three
  // reference points used have z-coordinate 0, so the resulting transform
  // has no z-information and always outputs a 0 for that coordinate.
  std::cout << "ref_points = \n" << ref_points << "\n";
  std::cout << "cam_points = \n" << cam_points << "\n";
  global_transform_ = ref_points * cam_points.inv();
  std::cout << "global_transform_ = \n" << global_transform_ << "\n";
  std::cout << "new_ref_points = \n"
            << global_transform_ * cam_points << "\n";
}

void LocalizationServer::LocalizeObjects() {
  for (TagInfoMap::iterator it = obj_tags_.begin();
       it != obj_tags_.end(); ++it) {
    TagInfo& tag = it->second;
    std::cout << "raw_t\n" << tag.raw_t << "\n";
    tag.t = global_transform_ * tag.raw_t;
    printf("Object (id #%zd) at (%.2f, %.2f, %.2f)\n",
           tag.id, tag.t[0][0], tag.t[1][0], tag.t[2][0]);
  }
}

void LocalizationServer::GenerateLocalizationData() {
  std::stringstream sstream;
  for (TagInfoMap::iterator it = obj_tags_.begin();
       it != obj_tags_.end(); ++it) {
    TagInfo& tag = it->second;
    sstream << tag.id << " @ "
            << tag.t[0][0] << " "
            << tag.t[1][0] << " "
            << tag.t[2][0] << "\n";
  }
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    localization_data_ = sstream.str();
  }
}

void LocalizationServer::ReceiveRequest() {
  if (DEBUG) std::cout << "Receiving request!\n";
  socket_.async_receive_from(
      asio::buffer(recv_buffer_), remote_endpoint_,
      boost::bind(&LocalizationServer::HandleRequest, this,
                  asio::placeholders::error,
                  asio::placeholders::bytes_transferred));
}

void LocalizationServer::HandleRequest(const asio::error_code& error,
                                       std::size_t /*bytes_transferred*/) {
  if (DEBUG) std::cout << "Handling request!\n";
  boost::shared_ptr<std::string> response_data(
      new std::string(""));
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    *response_data = localization_data_;
  }
  if (!error || error == asio::error::message_size) {
    socket_.async_send_to(
        asio::buffer(*response_data), remote_endpoint_,
        boost::bind(&LocalizationServer::HandleResponse, this,
                    asio::placeholders::error,
                    asio::placeholders::bytes_transferred));
  }
  ReceiveRequest();
}

void LocalizationServer::HandleResponse(const asio::error_code& /*error*/,
                                        std::size_t /*bytes_transferred*/) {
}

void LocalizationServer::Run() {
  if (DEBUG) std::cout << "Opening UDP socket.\n";
  socket_.open(udp::v4());
  if (DEBUG) std::cout << "Binding UDP socket to port "
                       << FLAGS_server_port << ".\n";
  socket_.bind(udp::endpoint(udp::v4(), FLAGS_server_port));
  if (DEBUG) std::cout << "Receiving initial request asynchronously...\n";
  ReceiveRequest();
  if (DEBUG) std::cout << "Launching IO processing thread...\n";
  asio::thread t(boost::bind(&asio::io_service::run, &io_service_));
  if (DEBUG) std::cout << "Running localization in main thread...\n";
  RunLocalization();
  t.join();
}

int main(int argc, char* argv[]) {
  std::string usage;
  usage += std::string("Usage: ") + argv[0] + std::string(" [OPTIONS]");
  gflags::SetUsageMessage(usage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  LocalizationServer server;
  server.Run();
  return 0;
}
