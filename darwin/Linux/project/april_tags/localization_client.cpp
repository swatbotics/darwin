#include "localization_client.hpp"

#include <iostream>
#include <sstream>
#include <unistd.h>

#include <boost/bind.hpp>
#include <gflags/gflags.h>

DEFINE_string(server_name, "192.168.1.7",
              "IP address or DNS name of the localization server to query.");
DEFINE_int32(server_port, 9000,
             "Port on the localization server to connect to.");

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
  port_string << FLAGS_server_port;
  udp::resolver::query query(udp::v4(), FLAGS_server_name,
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
  std::string usage;
  usage += std::string("Usage: ") + argv[0] + std::string(" [OPTIONS]");
  gflags::SetUsageMessage(usage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  LocalizationClient client;
  client.Run();
  while (true) {
    std::cout << client.GetData() << std::endl;
    usleep(10*1000);
  }
  client.Stop();
  return 0;
}
