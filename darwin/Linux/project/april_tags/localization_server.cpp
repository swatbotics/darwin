#include "localization_server.hpp"

#include <iostream>
#include <sstream>

#include <boost/bind.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <gflags/gflags.h>

#include "util.hpp"

#define DEBUG false

namespace asio = boost::asio;

const int LocalizationServer::kDefaultServerPort = 9000;

LocalizationServer::LocalizationServer(int server_port=kDefaultServerPort) :
    server_port_(server_port),
    localizer_(),
    callback_(this),
    data_mutex_(),
    localization_data_(),
    io_service_(),
    socket_(io_service_),
    remote_endpoint_(),
    recv_buffer_()
{
}

void LocalizationServer::DataRetrievalCallback(const std::string& data) {
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    localization_data_ = data;
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
                       << server_port_ << ".\n";
  socket_.bind(udp::endpoint(udp::v4(), server_port_));
  if (DEBUG) std::cout << "Receiving initial request asynchronously...\n";
  ReceiveRequest();
  if (DEBUG) std::cout << "Launching IO processing thread...\n";
  asio::thread t(boost::bind(&asio::io_service::run, &io_service_));
  if (DEBUG) std::cout << "Running localization in main thread...\n";
  localizer_.Run(&callback_);
  t.join();
}
