#pragma once
#include <iostream>
#include <string>
#include <list>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>

using namespace boost::asio::ip;

class UDPPeer
{

public:
	UDPPeer(std::string host, std::string port);
	~UDPPeer();

	void startRecieve();
	void send(std::string data);
	void send();

	void run_io() {io_service.run(); }

	std::string hasRec;
private:

	void handleRecieve(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
	void handleSend();

	boost::asio::io_service io_service;
	udp::resolver resolver;
	udp::resolver::query query;

	udp::socket socket_;
	udp::endpoint endpoint_;
	udp::endpoint recievedFromEndpoint;

	boost::array<char, 1> test;
	boost::array<char, 1> recvBff;

};