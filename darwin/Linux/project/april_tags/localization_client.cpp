#include "localization_client.hpp"

#include <iostream>
#include <sstream>

#include <boost/bind.hpp>

#define DEBUG false

namespace asio = boost::asio;

const int LocalizationClient::kDefaultServerPort = 9000;
const int LocalizationClient::kRecvBufferSize = 2048;

LocalizationClient::LocalizationClient(std::string server_name,
                                       int server_port=kDefaultServerPort) :
    server_name_(server_name),
    server_port_(server_port),
    data_mutex_(),
    localization_data_(),
    io_service_(),
    socket_(io_service_),
    remote_endpoint_(),
    send_buffer_(1, '\0'),
    recv_buffer_(kRecvBufferSize),
    io_thread_()
{
}

void LocalizationClient::SendRequest() {
  if (DEBUG) std::cout << "Sending request!\n";
  socket_.async_send_to(
      asio::buffer(send_buffer_), remote_endpoint_,
      boost::bind(&LocalizationClient::HandleRequest, this,
                  asio::placeholders::error,
                  asio::placeholders::bytes_transferred));
}

void LocalizationClient::HandleRequest(const asio::error_code& error,
                                       std::size_t /*bytes_transferred*/) {
  if (DEBUG) std::cout << "Handling request!\n";
  if (!error) {
    socket_.async_receive_from(
        asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&LocalizationClient::HandleResponse, this,
                    asio::placeholders::error,
                    asio::placeholders::bytes_transferred));
  }
}

void LocalizationClient::HandleResponse(const asio::error_code& error,
                                        std::size_t bytes_transferred) {
  if (DEBUG) std::cout << "Handling response!\n";
  if (!error && bytes_transferred > 0) {
    {
      boost::lock_guard<boost::mutex> lock(data_mutex_);
      localization_data_.assign(recv_buffer_.begin(),
                                recv_buffer_.begin() + bytes_transferred);
    }
  }
  SendRequest();
}

void LocalizationClient::Run() {
  if (DEBUG) std::cout << "Resolving localization server name.\n";
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

void LocalizationClient::Stop() {
  io_thread_.join();
  socket_.close();
}

std::string LocalizationClient::GetData() {
  {
    boost::lock_guard<boost::mutex> lock(data_mutex_);
    return localization_data_;
  }
}
