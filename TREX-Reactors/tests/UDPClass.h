#pragma once
#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>




class UDPClass{

public:
	UDPClass();
	~UDPClass();

	std::vector<double> receiveFrom(int uav){

		try
  {
    boost::asio::io_service io_service;

    std::vector<double > result = std::vector<double>(18,0.0);

    udp::socket socket(io_service, udp::endpoint(udp::v4(), 60001));

      boost::array<double, 18> recv_buf;
      udp::endpoint remote_endpoint;
      boost::system::error_code error;
      size_t len = socket.receive_from(boost::asio::buffer(recv_buf),
          remote_endpoint, 0, error);

     // std::cout.write(recv_buf.data(), len);
      std::cout << "Received: " <<  len << "\n" << recv_buf[0] << "\n" << std::flush;
      std::cout <<  recv_buf[1] << "\n" << std::flush;
      std::cout <<  recv_buf[2] << "\n" << std::flush;

      
      for( int i = 0; i < recv_buf.size(); i++ )
      	result[i] = recv_buf[i];

      return result;

      /*

      if (error && error != boost::asio::error::message_size)
        throw boost::system::system_error(error);

      std::string message = make_daytime_string();
      boost::array<double, 1> send_buf = {{ 0.0 }};

      boost::system::error_code ignored_error;
      socket.send_to(boost::asio::buffer("dette er en test"),
          remote_endpoint, 0, ignored_error);
    */


  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }


	}
	void sendTo(int uav, std::vector<double > data){

	try
	  {
	    

	    boost::asio::io_service io_service;

	    udp::resolver resolver(io_service);
	    udp::resolver::query query(udp::v4(), "localhost", "60001");
	    udp::endpoint receiver_endpoint = *resolver.resolve(query);

	    udp::socket socket(io_service);
	    socket.open(udp::v4());

	    boost::array<double, 12> send_buf;

	    for (auro it = data.begin(); it != data.end(); it++)
	    	send_buf[]

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

private:




}