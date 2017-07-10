//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2017 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

int sendN()
{
  try
  {
    

    boost::asio::io_service io_service;

    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), "localhost", "60001");
    udp::endpoint receiver_endpoint = *resolver.resolve(query);

    udp::socket socket(io_service);
    socket.open(udp::v4());

    boost::array<double, 1> send_buf;
    send_buf[0] = 10.1;
    socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

    boost::array<char, 128> recv_buf;
    udp::endpoint sender_endpoint;
   
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}

int main()
{
  try
  {
    boost::asio::io_service io_service;

    udp::socket socket(io_service, udp::endpoint(udp::v4(), 60000));

    for (;;)
    {
      boost::array<double, 1024> recv_buf;
      udp::endpoint remote_endpoint;
      boost::system::error_code error;
      size_t len = socket.receive_from(boost::asio::buffer(recv_buf),
          remote_endpoint, 0, error);

     // std::cout.write(recv_buf.data(), len);
      std::cout << "Received: " <<  len << "\n" << recv_buf[0] << "\n" << std::flush;
      std::cout <<  recv_buf[1] << "\n" << std::flush;
      std::cout <<  recv_buf[2] << "\n" << std::flush;

      sendN();

      if (error && error != boost::asio::error::message_size)
        throw boost::system::system_error(error);

      std::string message = make_daytime_string();
      boost::array<double, 1> send_buf = {{ 0.0 }};

      boost::system::error_code ignored_error;
      socket.send_to(boost::asio::buffer("dette er en test"),
          remote_endpoint, 0, ignored_error);
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
