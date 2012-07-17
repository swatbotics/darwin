#include "status_client.hpp"

#include <cmath>  // For abs() with floating-point support.
#include <iostream>
#include <sstream>
#include <time.h>

#include <boost/bind.hpp>
#include <gflags/gflags.h>

#define DEBUG false

namespace asio = boost::asio;

DEFINE_bool(multicast, true,
            "Get updates via multicast rather than direct requests.");
DEFINE_string(multicast_address, "239.255.0.1",
              "IP address of multicast group to join for status datagrams.");
DEFINE_string(multicast_listen_address, "0.0.0.0",
              "IP address on which to receive multicast datagrams.");
DEFINE_int32(multicast_port, 30001,
             "Port on which to receive multicast datagrams.");
DEFINE_string(server_name, "192.168.1.7",
              "IP address or DNS name of the status server to query.");
DEFINE_int32(server_port, 9000,
             "Port to connect to on status server to request unicast data.");
DEFINE_bool(include_timestamp, true,
            "Indicates that status datagrams contain a timestamp line of "
            "the form 'seconds nanoseconds' preceeding the payload.");
DEFINE_bool(numbered_packets, true,
            "Emit sequentially numbered packets (separate numbering for "
            "multicast and unicast packets).");
DEFINE_bool(show_packet_errors, true,
            "Report out-of-order, duplicate, or dropped packets.  Only "
            "effective if used with --numbered_packets.");
DEFINE_bool(measure_latency, false,
            "Show statistics about client-server latency.");

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


StatusClient::StatusClient() :
    data_mutex_(),
    data_(),
    io_service_(),
    packet_seq_num_(0),
    socket_(io_service_),
    multicast_socket_(io_service_),
    remote_endpoint_(),
    multicast_endpoint_(),
    send_buffer_(1, '\0'),
    recv_buffer_(kRecvBufferSize),
    multicast_recv_buffer_(kRecvBufferSize),
    request_send_time_(),
    worker_(NULL),
    io_thread_()
{
  if (FLAGS_multicast) {
    // Maybe should just always do this, in initializer list?
    multicast_endpoint_.address(
        asio::ip::address::from_string(FLAGS_multicast_listen_address));
    multicast_endpoint_.port(FLAGS_multicast_port);
  }
}

std::string StatusClient::GetData() {
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    return data_;
  }
}

void StatusClient::SubscribeData() {
  multicast_socket_.async_receive_from(
      asio::buffer(multicast_recv_buffer_), multicast_endpoint_,
      boost::bind(&StatusClient::HandleSubscribe, this,
                  asio::placeholders::error,
                  asio::placeholders::bytes_transferred));
}

void StatusClient::HandleSubscribe(const asio::error_code& error,
                                  std::size_t bytes_transferred) {
  if (DEBUG) std::cout << "Handling subscribe!\n";
  if (!error) {
    ParseDataFromBuffer(multicast_recv_buffer_, bytes_transferred);
  }
  SubscribeData();
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
    // Now it's the server's turn to send us the data back.
    ReceiveResponse();
  }
}

void StatusClient::ReceiveResponse() {
  socket_.async_receive_from(
      asio::buffer(recv_buffer_), remote_endpoint_,
      boost::bind(&StatusClient::HandleResponse, this,
                  asio::placeholders::error,
                  asio::placeholders::bytes_transferred));
}

void StatusClient::HandleResponse(const asio::error_code& error,
                                  std::size_t bytes_transferred) {
  if (DEBUG) std::cout << "Handling response!\n";
  if (!error) {
    ParseDataFromBuffer(recv_buffer_, bytes_transferred);
  }
  // Immediately send another request. TODO: optional delay here?
  SendRequest();
}

void StatusClient::ParseDataFromBuffer(const std::vector<char>& buffer,
                                       size_t num_bytes) {
  std::string payload(buffer.begin(), buffer.begin() + num_bytes);
  size_t start_pos = 0;
  if (FLAGS_numbered_packets) {
    size_t nl_pos = payload.find('\n', start_pos);
    if (FLAGS_show_packet_errors) {
      long packet_num;
      std::istringstream iss(payload.substr(start_pos, nl_pos));
      iss >> packet_num;
      if (packet_num < packet_seq_num_) {
        std::cerr << "PACKET ERROR: Packet " << packet_num << " "
                  << "arrived after packet " << packet_seq_num_ << "!\n";
      } else if (packet_num == packet_seq_num_) {
        std::cerr << "PACKET ERROR: Duplicate packet number "
                  << packet_num << "!\n";
      } else if (packet_num > packet_seq_num_ + 1) {
        std::cerr << "PACKET ERROR: Dropped "
                  << packet_num - packet_seq_num_ << " packets between "
                  << packet_seq_num_ << " and " << packet_num << "!\n";
      }
      packet_seq_num_ = packet_num;
    }
    start_pos = nl_pos + 1;
  }
  if (FLAGS_include_timestamp) {
    size_t nl_pos = payload.find('\n', start_pos);
    if (FLAGS_measure_latency) {
      std::string timestamp = payload.substr(start_pos, nl_pos);
      MeasureDelay(timestamp);
    }
    start_pos = nl_pos + 1;
  }
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    data_.assign(payload, start_pos, payload.size());
  }
}

void StatusClient::MeasureDelay(const std::string& timestamp) {
  struct timespec server_time = MakeTimespecFromString(timestamp);
  struct timespec client_time;
  clock_gettime(CLOCK_REALTIME, &client_time);
  struct timespec plain_diff = GetTimespecDiff(client_time, server_time);

  if (FLAGS_multicast) {
    std::cerr << "Latency + clock offset: "
              << MakeTimespecString(plain_diff) << "\n";
  } else {
    struct timespec rt_time_diff = GetTimespecDiff(client_time,
                                                   request_send_time_);
    struct timespec oneway_time_diff = ScaleTimespec(rt_time_diff, 0.5);
    struct timespec clock_diff = GetTimespecDiff(plain_diff,
                                                 oneway_time_diff);
    std::cerr
        << "RT latency: " << MakeTimespecString(rt_time_diff) << " - "
        << "OW latency: " << MakeTimespecString(oneway_time_diff) << " - "
        << "Clock diff: " << MakeTimespecString(clock_diff) << "\n";
  }
}

void StatusClient::Run() {
  if (FLAGS_multicast) {
    if (DEBUG) std::cout << "Opening UDP multicast socket.\n";
    multicast_socket_.open(udp::v4());
    // Allow multiple processes to bind to the same address.
    multicast_socket_.set_option(asio::ip::udp::socket::reuse_address(true));
    multicast_socket_.bind(multicast_endpoint_);
    asio::ip::address multicast_address =
        asio::ip::address::from_string(FLAGS_multicast_address);
    multicast_socket_.set_option(
        asio::ip::multicast::join_group(multicast_address));
    if (DEBUG) std::cout << "Starting multicast io_service worker.\n";
    worker_ = new asio::io_service::work(io_service_);
    if (DEBUG) std::cout << "Subscribing for initial data via multicast.\n";
    SubscribeData();

  } else {
    if (DEBUG) std::cout << "Resolving status server name.\n";
    udp::resolver resolver(io_service_);
    std::stringstream port_string;
    port_string << FLAGS_server_port;  // Convert int to string.
    udp::resolver::query query(udp::v4(), FLAGS_server_name,
                               port_string.str());
    remote_endpoint_ = *resolver.resolve(query);

    if (DEBUG) std::cout << "Opening UDP socket.\n";
    socket_.open(udp::v4());
    if (DEBUG) std::cout << "Sending initial request asynchronously...\n";
    SendRequest();
  }

  if (DEBUG) std::cout << "Launching IO processing thread...\n";
  io_thread_ = asio::thread(boost::bind(&asio::io_service::run, &io_service_));
}

void StatusClient::Stop() {
  if (worker_ != NULL) {
    delete worker_;
    worker_ = NULL;
  }
  io_thread_.join();
  socket_.close();
  multicast_socket_.close();
}
