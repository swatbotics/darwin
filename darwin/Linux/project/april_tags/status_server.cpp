#include "status_server.hpp"

#include <iostream>
#include <sstream>
#include <time.h>

#include <boost/bind.hpp>
#include <gflags/gflags.h>

#include "util.hpp"

#define DEBUG false

namespace asio = boost::asio;

DEFINE_bool(measure_latency, false,
            "Set if measuring client-server latency.");

const int StatusServer::kDefaultServerPort = 9000;

StatusServer::StatusServer(int server_port=kDefaultServerPort) :
    server_port_(server_port),
    data_mutex_(),
    data_(),
    io_service_(),
    socket_(io_service_),
    remote_endpoint_(),
    recv_buffer_()
{
}

void StatusServer::SetData(const std::string& data) {
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    data_ = data;
  }
}

void StatusServer::ReceiveRequest() {
  if (DEBUG) std::cout << "Receiving request!\n";
  socket_.async_receive_from(
      asio::buffer(recv_buffer_), remote_endpoint_,
      boost::bind(&StatusServer::HandleRequest, this,
                  asio::placeholders::error,
                  asio::placeholders::bytes_transferred));
}

void StatusServer::HandleRequest(const asio::error_code& error,
                                       std::size_t /*bytes_transferred*/) {
  if (DEBUG) std::cout << "Handling request!\n";
  std::stringstream sstr;
  if (FLAGS_measure_latency) {
    struct timespec curtime;
    clock_gettime(CLOCK_REALTIME, &curtime);
    sstr << curtime.tv_sec << " " << curtime.tv_nsec << "\n";
  }
  boost::shared_ptr<std::string> response_data(new std::string(sstr.str()));
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    *response_data += data_;
  }
  if (!error || error == asio::error::message_size) {
    socket_.async_send_to(
        asio::buffer(*response_data), remote_endpoint_,
        boost::bind(&StatusServer::HandleResponse, this,
                    asio::placeholders::error,
                    asio::placeholders::bytes_transferred));
  }
  ReceiveRequest();
}

void StatusServer::HandleResponse(const asio::error_code& /*error*/,
                                        std::size_t /*bytes_transferred*/) {
}

void StatusServer::Run() {
  if (DEBUG) std::cout << "Opening UDP socket.\n";
  socket_.open(udp::v4());
  if (DEBUG) std::cout << "Binding UDP socket to port "
                       << server_port_ << ".\n";
  socket_.bind(udp::endpoint(udp::v4(), server_port_));
  if (DEBUG) std::cout << "Receiving initial request asynchronously...\n";
  ReceiveRequest();
  if (DEBUG) std::cout << "Launching IO processing thread...\n";
  io_thread_ = asio::thread(boost::bind(&asio::io_service::run, &io_service_));
}

void StatusServer::Stop() {
  io_thread_.join();
  socket_.close();
}
