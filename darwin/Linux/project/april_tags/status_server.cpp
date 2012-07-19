#include "status_server.hpp"

#include <iostream>
#include <sstream>

#include <boost/bind.hpp>
#include <gflags/gflags.h>

#include "timestamp.hpp"
#include "util.hpp"

#define DEBUG false

namespace asio = boost::asio;

DEFINE_bool(multicast, true,
            "Publish data over multicast as well as to unicast clients.");
DEFINE_string(multicast_address, "239.255.0.1",
              "IP address to use for status server multicast datagrams.");
DEFINE_int32(multicast_port, 30001,
             "Port to use for status server outbound multicast datagrams.");
DEFINE_int32(server_port, 9000,
             "Port to use for status server to receive inbound datagrams "
             "and send outbound unicast datagrams.");
DEFINE_bool(include_timestamp, true,
            "Include a 'seconds nanoseconds' timestamp line at the top of "
            "each outbound datagram.");
DEFINE_bool(numbered_packets, true,
            "Emit sequentially numbered packets (separate numbering for "
            "multicast and unicast packets).");

StatusServer::StatusServer() :
    data_mutex_(),
    data_(),
    io_service_(),
    seq_num_(0),
    multicast_seq_num_(0),
    socket_(io_service_),
    multicast_socket_(io_service_),
    remote_endpoint_(),
    multicast_endpoint_(),
    recv_buffer_(),
    worker_(NULL),
    io_thread_()
{
  if (FLAGS_multicast) {
    // Maybe should just always do this, in initializer list?
    multicast_endpoint_.address(
        asio::ip::address::from_string(FLAGS_multicast_address));
    multicast_endpoint_.port(FLAGS_multicast_port);
  }
}

void StatusServer::UpdateData(const std::string& data) {
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    data_ = data;
  }
  if (FLAGS_multicast) PublishData();
}

void StatusServer::PublishData() {
  SendData(multicast_seq_num_++, multicast_endpoint_, multicast_socket_);
}

void StatusServer::RespondData() {
  SendData(seq_num_++, remote_endpoint_, socket_);
}

void StatusServer::SendData(long seq_num, const udp::endpoint& destination,
                            udp::socket& socket) {
  std::ostringstream sstr;
  if (FLAGS_numbered_packets) {
    sstr << seq_num << "\n";
  }
  if (FLAGS_include_timestamp) {
    Timestamp ts = Timestamp::Now();
    sstr << ts.ToString() << "\n";
  }
  boost::shared_ptr<std::string> response_data(new std::string(sstr.str()));
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    *response_data += data_;
  }
  if (DEBUG) std::cout << "Starting send with data: \n" << *response_data
                       << " to address: " << destination.address()
                       << ":" << destination.port() << "\n";
  socket.async_send_to(
      asio::buffer(*response_data), destination,
      boost::bind(&StatusServer::HandleSend, this,
                  asio::placeholders::error,
                  asio::placeholders::bytes_transferred));
}

void StatusServer::HandleSend(const asio::error_code& error,
                              std::size_t bytes_transferred) {
  if (error) {
    std::cerr << "Error in sending data!\n";
  } else if (DEBUG) {
    std::cout << "Sent " << bytes_transferred << " bytes of data!\n";
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
  if (!error || error == asio::error::message_size) {
    RespondData();
  }
  ReceiveRequest();
}

void StatusServer::Run() {
  if (DEBUG) std::cout << "Opening UDP unicast socket.\n";
  socket_.open(udp::v4());
  if (DEBUG) std::cout << "Binding UDP unicast socket to port "
                       << FLAGS_server_port << ".\n";
  socket_.bind(udp::endpoint(udp::v4(), FLAGS_server_port));
  if (DEBUG) std::cout << "Receiving initial request asynchronously...\n";
  ReceiveRequest();

  if (FLAGS_multicast) {
    if (DEBUG) std::cout << "Opening UDP multicast socket.\n";
    multicast_socket_.open(udp::v4());
    if (DEBUG) std::cout << "Starting multicast io_service worker.\n";
    worker_ = new asio::io_service::work(io_service_);
    if (DEBUG) std::cout << "Publishing initial data via multicast.\n";
    PublishData();
  }

  if (DEBUG) std::cout << "Launching IO processing thread...\n";
  io_thread_ =
      asio::thread(boost::bind(&asio::io_service::run, &io_service_));
}

void StatusServer::Stop() {
  if (worker_ != NULL) {
    delete worker_;
    worker_ = NULL;
  }
  io_thread_.join();
  socket_.close();
  if (FLAGS_multicast) {
    multicast_socket_.close();
  }
}
