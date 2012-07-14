#include "status_client.hpp"

#include <iostream>
#include <sstream>
#include <time.h>

#include <boost/bind.hpp>
#include <gflags/gflags.h>

#define DEBUG false

namespace asio = boost::asio;

DEFINE_bool(measure_latency, false,
            "Set if measuring client-server latency.");

const int StatusClient::kDefaultServerPort = 9000;
const int StatusClient::kRecvBufferSize = 2048;

StatusClient::StatusClient(std::string server_name,
                           int server_port=kDefaultServerPort) :
    server_name_(server_name),
    server_port_(server_port),
    data_mutex_(),
    data_(),
    io_service_(),
    socket_(io_service_),
    remote_endpoint_(),
    send_buffer_(1, '\0'),
    recv_buffer_(kRecvBufferSize),
    io_thread_()
{
}

void StatusClient::SendRequest() {
  if (DEBUG) std::cout << "Sending request!\n";
  socket_.async_send_to(
      asio::buffer(send_buffer_), remote_endpoint_,
      boost::bind(&StatusClient::HandleRequest, this,
                  asio::placeholders::error,
                  asio::placeholders::bytes_transferred));
}

void StatusClient::HandleRequest(const asio::error_code& error,
                                 std::size_t /*bytes_transferred*/) {
  if (DEBUG) std::cout << "Handling request!\n";
  if (!error) {
    socket_.async_receive_from(
        asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&StatusClient::HandleResponse, this,
                    asio::placeholders::error,
                    asio::placeholders::bytes_transferred));
  }
}

void StatusClient::HandleResponse(const asio::error_code& error,
                                  std::size_t bytes_transferred) {
  if (DEBUG) std::cout << "Handling response!\n";
  if (!error) {
    std::string payload(recv_buffer_.begin(),
                        recv_buffer_.begin() + bytes_transferred);
    size_t start_pos = 0;
    if (FLAGS_measure_latency) {
      size_t nl_pos = payload.find('\n');
      start_pos = nl_pos + 1;
      std::string timestamp = payload.substr(0, nl_pos);
      MeasureLatency(timestamp);
    }
    {
      boost::lock_guard<boost::mutex> lock(data_mutex_);
      data_.assign(payload, start_pos, payload.size());
    }
  }
  SendRequest();
}

void StatusClient::MeasureLatency(const std::string& timestamp) {
  std::stringstream sstr(timestamp);
  struct timespec server_time, client_time, diff_time;
  sstr >> server_time.tv_sec >> server_time.tv_nsec;
  clock_gettime(CLOCK_REALTIME, &client_time);

  // Get the difference between the timespecs.
  diff_time.tv_sec = client_time.tv_sec - server_time.tv_sec;
  diff_time.tv_nsec = client_time.tv_nsec - server_time.tv_nsec;
  if (diff_time.tv_nsec < 0) {
    diff_time.tv_sec -= 1;
    diff_time.tv_nsec += 1000 * 1000 * 1000;
  }

  // Compute and print latency.
  double latency = (diff_time.tv_sec +
                    diff_time.tv_nsec / (1000 * 1000 * 1000.0));
  fprintf(stderr, "Latency: ");
  if (latency > 1.0) {
    fprintf(stderr, "%.1fs", latency);
  } else {
    double latency_ms = latency * 1000;
    if (latency_ms > 1.0) {
      fprintf(stderr, "%.1fms", latency_ms);
    } else {
      double latency_us = latency_ms * 1000;
      fprintf(stderr, "%.1fus", latency_us);
    }
  }
  std::cerr << "\n";
}

void StatusClient::Run() {
  if (DEBUG) std::cout << "Resolving status server name.\n";
  udp::resolver resolver(io_service_);
  std::stringstream port_string;
  port_string << server_port_;  // Convert int to string.
  udp::resolver::query query(udp::v4(), server_name_,
                             port_string.str());
  remote_endpoint_ = *resolver.resolve(query);

  if (DEBUG) std::cout << "Opening UDP socket.\n";
  socket_.open(udp::v4());
  if (DEBUG) std::cout << "Sending initial request asynchronously...\n";
  SendRequest();
  if (DEBUG) std::cout << "Launching IO processing thread...\n";
  io_thread_ = asio::thread(boost::bind(&asio::io_service::run, &io_service_));
}

void StatusClient::Stop() {
  io_thread_.join();
  socket_.close();
}

std::string StatusClient::GetData() {
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    return data_;
  }
}
