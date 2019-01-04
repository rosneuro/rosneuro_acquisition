#ifndef ROSNEURO_ACQUISITION_FACTORY_DEVICE_CPP
#define ROSNEURO_ACQUISITION_FACTORY_DEVICE_CPP

#include "rosneuro_acquisition/FactoryDevice.hpp"

namespace rosneuro {

std::unique_ptr<Device> FactoryDevice::createDevice(NeuroFrame* frame, unsigned int type) {

	std::unique_ptr<Device> dev;
	switch(type) {
		case DeviceType::EGDDEV:
			dev = std::unique_ptr<EGDDevice>(new EGDDevice(frame));
			break;
		case DeviceType::DUMMYDEV:
			dev = std::unique_ptr<DummyDevice>(new DummyDevice(frame));
			break;
		default:
			printf("[FactoryDevice] - Unknown device type required: %u\n", type);
			dev = std::unique_ptr<Device>(nullptr);
			break;
	}

	return dev;
}


}

#endif
