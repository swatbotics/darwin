#include "localization_client.hpp"

#include <iostream>
#include <sstream>
#include <unistd.h>

#include <boost/bind.hpp>

#define LOCALIZATION_SERVER_NAME "192.168.1.7"
#define LOCALIZATION_SERVER_PORT 9000
#define RECV_BUFFER_SIZE 1024

#define DEBUG false

namespace asio = boost::asio;

LocalizationClient::LocalizationClient() :
    data_mutex_(),
    localization_data_(),
    io_service_(),
    socket_(io_service_),
    remote_endpoint_(),
    send_buffer_(1, '\0'),
    recv_buffer_(RECV_BUFFER_SIZE),
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
  port_string << LOCALIZATION_SERVER_PORT;
  udp::resolver::query query(udp::v4(), LOCALIZATION_SERVER_NAME,
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

int main(int argc, char* argv[]) {
  LocalizationClient client;
  client.Run();
  while (true) {
    std::cout << client.GetData() << std::endl;
    usleep(10*1000);
  }
  client.Stop();
  return 0;
}
