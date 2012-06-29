#include "localization_server.hpp"

#include <iostream>
#include <sstream>

#include <boost/bind.hpp>
#include <gflags/gflags.h>

#include "util.hpp"

DEFINE_int32(device_num, 0,
             "Device number for video device to parse for tag information.");
DEFINE_string(tag_family, "Tag36h11",
              "Tag family to use for detections.");
DEFINE_int32(frame_width, 640, "Desired video frame width.");
DEFINE_int32(frame_height, 480, "Desired video frame height.");
DEFINE_int32(server_port, 9000,
             "Port on which to run UDP localization service.");

#define DEBUG false

namespace asio = boost::asio;

LocalizationServer::LocalizationServer() :
    vc_(),
    tag_family_(FLAGS_tag_family),
    detector_(tag_family_),
    data_mutex_(),
    localization_data_(),
    io_service_(),
    socket_(io_service_),
    remote_endpoint_(),
    recv_buffer_()
{
  detector_.params.segDecimate = true;
  detector_.params.thetaThresh = 25;
  detector_.params.refineCornersSubPix = true;
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
  TagDetectionArray detections;
  cv::Mat frame;
  double lasttime = get_time_as_double();
  while (true) {
    vc_ >> frame;
    if (frame.empty()) {
      std::cerr << "Got empty frame!\n";
      exit(1);
    }
    const cv::Point2d optical_center(frame.cols * 0.5, frame.rows * 0.5);
    detector_.process(frame, optical_center, detections);
    for (size_t i = 0; i < detections.size(); ++i) {
      /*
      const TagDetection& d = detections[i];
      std::cout << " - Detection: id = " << d.id << ", "
                << "code = " << d.code << ", "
                << "rotation = " << d.rotation << "\n";
      */
    }
    std::stringstream sstream;
    sstream << detections.size();
    {
      boost::lock_guard<boost::mutex> lock(data_mutex_);
      localization_data_ = sstream.str();
    }
    double thistime = get_time_as_double();
    printf("FPS: %d\n", (int) (1 / (thistime - lasttime)));
    lasttime = thistime;
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
