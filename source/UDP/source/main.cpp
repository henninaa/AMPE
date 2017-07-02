#include "UDPPeer.h"


int main(int argc, char const *argv[])
{
	/* code */
	std::cout << argc;
	UDPPeer peer("localhost", "30000");
	if (argc == 2){
	std::cout << "recieving";
	
	std::cout << std::flush;
	peer.startRecieve();
	}	
	else
		std::cout << "sending";

	std::cout << std::flush;
	while(true){
		if (argc == 1){
			peer.send();
		
			}
		else break;//std::cout << peer.hasRec;
			
	}


	//peer.run_io();

	return 0;
}
