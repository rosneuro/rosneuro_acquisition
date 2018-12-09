#ifndef ROSNEURO_ACQUISITION_DEVICEFACTORY_CPP
#define ROSNEURO_ACQUISITION_DEVICEFACTORY_CPP

#include "rosneuro_acquisition/DeviceFactory.hpp"

namespace rosneuro {

std::unique_ptr<Device> DeviceFactory::createDevice(unsigned int type) {

	std::unique_ptr<Device> dev;
	switch(type) {
		case DeviceType::EGDDEV:
			dev = std::unique_ptr<EGDDevice>(new EGDDevice);
			break;
		case DeviceType::DUMMYDEV:
			dev = std::unique_ptr<DummyDevice>(new DummyDevice);
			break;
		default:
			printf("[DeviceFactory] - Unknown device type required: %u\n", type);
			dev = std::unique_ptr<Device>(nullptr);
			break;
	}

	return dev;
}


}

#endif
