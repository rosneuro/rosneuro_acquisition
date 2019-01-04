#include "rosneuro_acquisition/EGDDevice.hpp"
#include "rosneuro_data/NeuroData.hpp"

#include <unistd.h>

int main(int argc, char** argv) {


	rosneuro::NeuroFrame	frame;
	rosneuro::EGDDevice		egddev(&frame);


	if(egddev.Open(argv[1]) == false)
		return -1;
		
	
	if(egddev.Setup(16.0f) == false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}


	frame.eeg.dump();
	frame.exg.dump();
	frame.tri.dump();

	egddev.Close();

	return 0;
}
