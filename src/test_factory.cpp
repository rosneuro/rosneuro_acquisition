#include "rosneuro_acquisition/FactoryDevice.hpp"
#include "rosneuro_acquisition/Device.hpp"
#include <unistd.h>

using namespace rosneuro;

int main(int argc, char** argv) {


	Device* dummydev = FactoryDevice::get().createDevice("DummyDevice");
	Device* lsldev   = FactoryDevice::get().createDevice("LSLDevice");
	Device* egddev   = FactoryDevice::get().createDevice("EGDDevice");
	
	dummydev->Who();
	lsldev->Who();
	egddev->Who();

	delete dummydev;
	delete lsldev;
	delete egddev;

	return 0;
}
