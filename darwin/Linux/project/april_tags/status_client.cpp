#include "status_client.hpp"

#include <cmath>  // For abs() with floating-point support.
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

struct timespec MakeTimespecFromString(const std::string& str) {
  struct timespec ts;
  std::stringstream ss(str);
  ss >> ts.tv_sec >> ts.tv_nsec;
  return ts;
}

struct timespec GetTimespecDiff(struct timespec first,
                                struct timespec second) {
  struct timespec diff_time;
  diff_time.tv_sec = first.tv_sec - second.tv_sec;
  diff_time.tv_nsec = first.tv_nsec - second.tv_nsec;
  if (diff_time.tv_nsec < 0) {
    diff_time.tv_sec -= 1;
    diff_time.tv_nsec += 1000 * 1000 * 1000;
  }
  return diff_time;
}

struct timespec ScaleTimespec(struct timespec ts, double scale) {
  struct timespec scaled;
  scaled.tv_sec = abs(ts.tv_sec) * scale;
  scaled.tv_nsec = abs(ts.tv_nsec) * scale;
  time_t extra = abs(ts.tv_sec) - scaled.tv_sec * (1 / scale);
  scaled.tv_nsec += extra * 1000 * 1000 * 1000 * scale;
  // Recover sign information.
  scaled.tv_sec *= (ts.tv_sec >= 0 ? 1 : -1);
  scaled.tv_nsec *= (ts.tv_nsec >= 0 ? 1 : -1);
  return scaled;
}

std::string MakeTimespecString(struct timespec ts) {
  double time_in_s = (ts.tv_sec + ts.tv_nsec / (1000 * 1000 * 1000.0));
  std::stringstream ss;
  ss.precision(1);
  ss.width(5);
  ss << std::fixed;
  if (abs(time_in_s) >= 1.0) {
    ss << time_in_s << "s";
  } else {
    double time_in_ms = time_in_s * 1000;
    if (abs(time_in_ms) >= 1.0) {
      ss << time_in_ms << "ms";
    } else {
      double time_in_us = time_in_ms * 1000;
      ss << time_in_us << "us";
    }
  }
  return ss.str();
}


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
    request_send_time_(),
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
  clock_gettime(CLOCK_REALTIME, &request_send_time_);
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
      MeasureDelay(timestamp);
    }
    {
      boost::lock_guard<boost::mutex> lock(data_mutex_);
      data_.assign(payload, start_pos, payload.size());
    }
  }
  SendRequest();
}

void StatusClient::MeasureDelay(const std::string& timestamp) {
  struct timespec server_time = MakeTimespecFromString(timestamp);
  struct timespec client_time;
  clock_gettime(CLOCK_REALTIME, &client_time);
  struct timespec plain_diff = GetTimespecDiff(client_time, server_time);
  struct timespec rt_time_diff = GetTimespecDiff(client_time,
                                                 request_send_time_);
  struct timespec oneway_time_diff = ScaleTimespec(rt_time_diff, 0.5);
  struct timespec clock_diff = GetTimespecDiff(plain_diff,
                                               oneway_time_diff);
  std::cerr << "RT latency: " << MakeTimespecString(rt_time_diff) << " - "
            << "OW latency: " << MakeTimespecString(oneway_time_diff) << " - "
            << "Clock diff: " << MakeTimespecString(clock_diff) << "\n";
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
