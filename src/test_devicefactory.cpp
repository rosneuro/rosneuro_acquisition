#include "rosneuro_acquisition/DeviceFactory.hpp"

using namespace rosneuro::acquisition;

int main(int argc, char** argv) {


	DeviceFactory factory;

	std::unique_ptr<Device> egddev   = factory.createDevice(DeviceType::EGD_DEVICE);
	std::unique_ptr<Device> dummydev = factory.createDevice(DeviceType::DUMMY_DEVICE);
	std::unique_ptr<Device> xxxdev   = factory.createDevice(666);
	

	egddev->Who();
	dummydev->Who();

	egddev->Open("gtec");

	return 0;
}
