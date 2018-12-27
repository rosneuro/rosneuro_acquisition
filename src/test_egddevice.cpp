#include "rosneuro_acquisition/EGDDevice.hpp"

#include <unistd.h>

int main(int argc, char** argv) {



	rosneuro::EGDDevice egddev;


	if(egddev.Open(argv[1]) == false)
		return -1;
		
	
	if(egddev.Setup(16.0f) == false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}


	egddev.eeg.dump();
	egddev.exg.dump();
	egddev.tri.dump();

	egddev.Close();

	return 0;
}
