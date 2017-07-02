#include "UDPPeer.h"

using namespace boost::asio::ip;

UDPPeer::UDPPeer(std::string host, std::string port) : io_service(), resolver(io_service), query(udp::v4(), host, port), socket_(io_service)
{
	boost::system::error_code error;
	endpoint_ = *resolver.resolve(query);
	std::cout << endpoint_.address();
	socket_.bind(udp::endpoint(udp::v4(), 30000));
	//socket_.open(udp::v4(), error);
	hasRec = "NO\n";

	if (error)
		std::cout << "ERROR!";
}



UDPPeer::~UDPPeer(){

	//socket_.close();
}

void UDPPeer::startRecieve(){

    std::cout << "s";

	boost::system::error_code error;
	/*
    socket_.async_receive_from(boost::asio::buffer(recvBff), endpoint_,
    	boost::bind(&UDPPeer::handleRecieve, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));

    */
           socket_.receive_from(boost::asio::buffer(recvBff),
          endpoint_);
           std::cout << "\ngot data\n";
}

void UDPPeer::send(std::string data){

	socket_.send_to(boost::asio::buffer(test), endpoint_);
}
void UDPPeer::send(){
	boost::array<char, 1> send_buf  = {{ 0 }};
	socket_.send_to(boost::asio::buffer(send_buf), endpoint_);
}

void UDPPeer::handleRecieve(const boost::system::error_code& error, std::size_t /*bytes_transferred*/){

	hasRec =  "RECIEVE\n";
}

void UDPPeer::handleSend(){

	
}