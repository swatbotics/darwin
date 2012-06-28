#include "localization_server.hpp"

#include <iostream>

#include <boost/bind.hpp>

#define DEFAULT_DEVICE_NUMBER 0
#define DEFAULT_TAG_FAMILY "Tag36h11"
#define DESIRED_FRAME_WIDTH 1280
#define DESIRED_FRAME_HEIGHT 720

#define DEFAULT_SERVER_PORT 9000

namespace asio = boost::asio;

LocalizationServer::LocalizationServer() :
    vc_(),
    tag_family_(DEFAULT_TAG_FAMILY),
    detector_(tag_family_),
    io_service_(),
    socket_(io_service_),
    remote_endpoint_(),
    recv_buffer_(),
    dummy_response_("foo")
{
  detector_.params.segDecimate = true;
  detector_.params.thetaThresh = 25;
  detector_.params.refineCornersSubPix = true;
  InitializeVideoDevice();
}

void LocalizationServer::InitializeVideoDevice() {
  int device = DEFAULT_DEVICE_NUMBER;
  vc_.open(device);
  if (!vc_.isOpened()) {
    std::cerr << "Could not open video device!\n";
    exit(1);
  }
  vc_.set(CV_CAP_PROP_FRAME_WIDTH, DESIRED_FRAME_WIDTH);
  vc_.set(CV_CAP_PROP_FRAME_HEIGHT, DESIRED_FRAME_HEIGHT);
  std::cout << "Camera resolution: "
            << vc_.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
            << vc_.get(CV_CAP_PROP_FRAME_HEIGHT) << "\n";
}

void LocalizationServer::RunLocalization() {
  TagDetectionArray detections;
  cv::Mat frame;
  while (true) {
    vc_ >> frame;
    if (frame.empty()) {
      std::cerr << "Got empty frame!\n";
      exit(1);
    }
    const cv::Point2d optical_center(frame.cols * 0.5, frame.rows * 0.5);
    detector_.process(frame, optical_center, detections);
    for (size_t i = 0; i < detections.size(); ++i) {
      const TagDetection& d = detections[i];
      /*
      std::cout << " - Detection: id = " << d.id << ", "
                << "code = " << d.code << ", "
                << "rotation = " << d.rotation << "\n";
      */
    }
  }
}

void LocalizationServer::ReceiveRequest() {
  std::cout << "Receiving request!\n";
  socket_.async_receive_from(
      asio::buffer(recv_buffer_), remote_endpoint_,
      boost::bind(&LocalizationServer::HandleRequest, this,
                  asio::placeholders::error,
                  asio::placeholders::bytes_transferred));
}

void LocalizationServer::HandleRequest(const asio::error_code& error,
                                       std::size_t /*bytes_transferred*/) {
  std::cout << "Handling request!\n";
  if (!error || error == asio::error::message_size) {
    socket_.async_send_to(
        asio::buffer(dummy_response_), remote_endpoint_,
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
  std::cout << "Opening UDP socket.\n";
  socket_.open(asio::ip::udp::v4());
  std::cout << "Binding UDP socket to port " << DEFAULT_SERVER_PORT << ".\n";
  socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(),
                                       DEFAULT_SERVER_PORT));
  std::cout << "Receiving initial request asynchronously...\n";
  ReceiveRequest();
  std::cout << "Launching IO processing thread...\n";
  asio::thread t(boost::bind(&asio::io_service::run, &io_service_));
  std::cout << "Running localization in main thread...\n";
  RunLocalization();
  t.join();
}

int main(int argc, char* argv[]) {
  LocalizationServer server;
  server.Run();
  return 0;
}
