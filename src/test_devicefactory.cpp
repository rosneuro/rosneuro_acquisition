#include "rosneuro_acquisition/DeviceFactory.hpp"
#include <unistd.h>

using namespace rosneuro;

int main(int argc, char** argv) {


	DeviceFactory factory;

	std::unique_ptr<Device> egddev   = factory.createDevice(DeviceType::EGDDEV);
	std::unique_ptr<Device> dummydev = factory.createDevice(DeviceType::DUMMYDEV);
	
	egddev->Who();
	dummydev->Who();

	std::cout<<">>>>>>>>> TEST EGDDEV <<<<<<<<<<<<<<<"<<std::endl;
	if(egddev->Open(argv[1]) == false)
		return -1;
		
	sleep(1);
	
	if(egddev->Setup(16.0f) == false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}


	egddev->Dump();
	
	std::cout<<">>>>>>>>> TEST DUMMYDEV <<<<<<<<<<<<<<<"<<std::endl;
	if(dummydev->Open("") == false)
		return -1;
		
	sleep(1);
	
	if(dummydev->Setup(16.0f) == false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}


	dummydev->Dump();

	return 0;
}
