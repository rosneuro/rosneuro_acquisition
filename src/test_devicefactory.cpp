#include "rosneuro_acquisition/FactoryDevice.hpp"
#include "rosneuro_data/NeuroData.hpp"
#include <unistd.h>

using namespace rosneuro;

int main(int argc, char** argv) {


	FactoryDevice factory;
	rosneuro::NeuroFrame	frame;

	std::unique_ptr<Device> egddev   = factory.createDevice(&frame, DeviceType::EGDDEV);
	std::unique_ptr<Device> dummydev = factory.createDevice(&frame, DeviceType::DUMMYDEV);
	
	egddev->Who();
	dummydev->Who();

	std::cout<<"\n>>>>>>>>> TEST EGDDEV <<<<<<<<<<<<<<<"<<std::endl;
	if(egddev->Open(argv[1]) == false)
		return -1;
		
	
	if(egddev->Setup(16.0f) == false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}


	frame.eeg.dump();
	frame.exg.dump();
	frame.tri.dump();
	
	std::cout<<"\n>>>>>>>>> TEST DUMMYDEV <<<<<<<<<<<<<<<"<<std::endl;
	if(dummydev->Open("") == false)
		return -1;
		
	
	if(dummydev->Setup(16.0f) == false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}



	return 0;
}
