#include "rosneuro_acquisition/SelfFactoryDevice.hpp"
#include "rosneuro_acquisition/Device.hpp"
#include <unistd.h>

using namespace rosneuro;

int main(int argc, char** argv) {


	Device* dummydev = SelfFactoryDevice::get().createDevice("DummyDevice");
	Device* lsldev   = SelfFactoryDevice::get().createDevice("LSLDevice");
	Device* egddev   = SelfFactoryDevice::get().createDevice("EGDDevice");
	
	dummydev->Who();
	lsldev->Who();
	egddev->Who();



	return 0;
}
