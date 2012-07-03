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
DEFINE_double(obj_tag_size, 0.075, "Size of object tags.");
DEFINE_double(ref_tag_size, 0.075, "Size of reference tags.");
DEFINE_int32(frame_width, 640, "Desired video frame width.");
DEFINE_int32(frame_height, 480, "Desired video frame height.");
DEFINE_int32(server_port, 9000,
             "Port on which to run UDP localization service.");

#define DEBUG false

namespace asio = boost::asio;

const double LocalizationServer::kObjectTagSize = FLAGS_obj_tag_size;
const double LocalizationServer::kReferenceTagSize = FLAGS_ref_tag_size;

const LocalizationServer::ReferenceTagCoords
LocalizationServer::reference_tag_coords_[] = {
  {0, {0, 0, 0}},
  {1, {0, kReferenceTagInterval, 0}},
  {2, {kReferenceTagInterval, 0, 0}},
  {3, {kReferenceTagInterval, kReferenceTagInterval, 0}}
};

const LocalizationServer::ReferenceTagSystem
LocalizationServer::reference_tag_system_ = {0, 2, 1};

LocalizationServer::LocalizationServer() :
    vc_(),
    frame_(),
    optical_center_(),
    tag_family_(FLAGS_tag_family),
    detector_(tag_family_),
    detections_(),
    ref_tags_(),
    obj_tags_(),
    global_translation_(3, 1),
    global_rotation_(3, 3),
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
  vc_ >> frame_;
  if (frame_.empty()) {
    std::cerr << "Got empty frame!\n";
    exit(1);
  }
  optical_center_ = cv::Point2d(frame_.cols * 0.5, frame_.rows * 0.5);
  detector_.process(frame_, optical_center_, detections_);
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
      std::cout << "ref tag " << tag.id << " raw_t = \n" << tag.raw_t << "\n";
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
  tag.size = tag_size;
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
  if (ref_tags_.count(reference_tag_system_.origin) == 0 ||
      ref_tags_.count(reference_tag_system_.primary) == 0 ||
      ref_tags_.count(reference_tag_system_.secondary) == 0) {
    std::cout << "Cannot localize with this tag system!\n";
    return;
  }
  TagInfo origin_ref = ref_tags_[reference_tag_system_.origin];
  TagInfo primary_ref = ref_tags_[reference_tag_system_.primary];
  TagInfo secondary_ref = ref_tags_[reference_tag_system_.secondary];

  cv::Mat_<double> basis(3, 3);
  cv::Mat_<double> primary_vec = primary_ref.raw_t - origin_ref.raw_t;
  cv::Mat_<double> secondary_vec = secondary_ref.raw_t - origin_ref.raw_t;

  std::cout << "primary_vec = \n" << primary_vec << "\n";
  std::cout << primary_vec.rows << "x" << primary_vec.cols << "\n";
  std::cout << "norm = " << cv::norm(primary_vec) << "\n";
  std::cout << "secondary_vec = \n" << secondary_vec << "\n";
  std::cout << secondary_vec.rows << "x" << secondary_vec.cols << "\n";
  std::cout << "norm = " << cv::norm(secondary_vec) << "\n";

  cv::Mat_<double> primary_nvec = primary_vec / cv::norm(primary_vec);
  cv::Mat_<double> secondary_nvec = secondary_vec / cv::norm(secondary_vec);
  std::cout << "primary_nvec = \n" << primary_nvec << "\n";
  std::cout << primary_nvec.rows << "x" << primary_nvec.cols << "\n";
  std::cout << "norm = " << cv::norm(primary_nvec) << "\n";
  std::cout << "secondary_nvec = \n" << secondary_nvec << "\n";
  std::cout << secondary_nvec.rows << "x" << secondary_nvec.cols << "\n";
  std::cout << "norm = " << cv::norm(secondary_nvec) << "\n";

  cv::Mat_<double> zero = cv::Mat_<double>::zeros(3, 1);
  basis.col(0) = primary_vec + zero;
  basis.col(2) = primary_vec.cross(secondary_vec) + zero;
  basis.col(1) = basis.col(2).cross(primary_vec) + zero;
  std::cout << "basis raw = \n" << basis << "\n";
  basis.col(0) /= cv::norm(basis.col(0));
  basis.col(1) /= cv::norm(basis.col(1));
  basis.col(2) /= cv::norm(basis.col(2));
  std::cout << "basis normed = \n" << basis << "\n";

  cv::Mat_<double> oldbasis(3, 3);
  oldbasis.col(0) = primary_nvec + zero;
  oldbasis.col(2) = primary_nvec.cross(secondary_nvec) + zero;
  oldbasis.col(1) = oldbasis.col(2).cross(primary_nvec) + zero;
  std::cout << "basis old = \n" << oldbasis << "\n";

  std::cout << "basis check = \n" << basis * basis.t() << "\n";

  global_rotation_ = basis.t();
  std::cout << "global_rotation_ = \n" << global_rotation_ << "\n";
  global_translation_ = -global_rotation_ * origin_ref.raw_t;
  std::cout << "global_translation_ = \n" << global_translation_ << "\n";

  std::cout << "recompute tag " << reference_tag_system_.origin << " = \n"
            << TransformToGlobal(origin_ref.raw_t) << "\n";
  std::cout << "recompute tag " << reference_tag_system_.primary << " = \n"
            << TransformToGlobal(primary_ref.raw_t) << "\n";
  std::cout << "recompute tag " << reference_tag_system_.secondary << " = \n"
            << TransformToGlobal(secondary_ref.raw_t) << "\n";

  LocalizeObjects();

  cv::Mat_<double> ref_points(3, 3);
  cv::Mat_<double> cam_points(3, 3);
  ref_points.col(0) = primary_ref.t - origin_ref.t + zero;
  ref_points.col(1) = secondary_ref.t - origin_ref.t + zero;
  ref_points.col(2) = ref_points.col(0).cross(ref_points.col(1)) + zero;
  cam_points.col(0) = primary_vec + zero;
  cam_points.col(1) = secondary_vec + zero;
  cam_points.col(2) = cam_points.col(0).cross(cam_points.col(1)) + zero;

  std::cout << "ref_points = \n" << ref_points << "\n";
  std::cout << "cam_points = \n" << cam_points << "\n";
  global_rotation_ = ref_points * cam_points.inv();
  std::cout << "global_rotation_ = \n" << global_rotation_ << "\n";
  global_translation_ = -global_rotation_ * origin_ref.raw_t;
  std::cout << "global_translation_ = \n" << global_translation_ << "\n";

  std::cout << "recompute tag " << reference_tag_system_.origin << " = \n"
            << TransformToGlobal(origin_ref.raw_t) << "\n";
  std::cout << "recompute tag " << reference_tag_system_.primary << " = \n"
            << TransformToGlobal(primary_ref.raw_t) << "\n";
  std::cout << "recompute tag " << reference_tag_system_.secondary << " = \n"
            << TransformToGlobal(secondary_ref.raw_t) << "\n";
}

cv::Mat_<double> LocalizationServer::TransformToGlobal(
    const cv::Mat_<double>& vec) {
  return global_rotation_ * vec + global_translation_;
}

cv::Mat_<double> LocalizationServer::TransformToCamera(
    const cv::Mat_<double>& vec) {
  return global_rotation_.inv() * (vec - global_translation_);
}

void LocalizationServer::LocalizeObjects() {
  for (TagInfoMap::iterator it = obj_tags_.begin();
       it != obj_tags_.end(); ++it) {
    TagInfo& tag = it->second;
    std::cout << "raw_t\n" << tag.raw_t << "\n";
    tag.t = global_rotation_ * tag.raw_t + global_translation_;
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
