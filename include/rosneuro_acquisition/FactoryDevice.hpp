#ifndef ROSNEURO_ACQUISITION_FACTORY_DEVICE_HPP
#define ROSNEURO_ACQUISITION_FACTORY_DEVICE_HPP

#include <memory>
#include "rosneuro_acquisition/Device.hpp"
#include "rosneuro_acquisition/EGDDevice.hpp"
#include "rosneuro_acquisition/DummyDevice.hpp"

namespace rosneuro {

enum DeviceType {EGDDEV, DUMMYDEV};

class FactoryDevice {

	public:
		std::unique_ptr<Device> createDevice(NeuroFrame* frame, unsigned int type = DeviceType::EGDDEV);

};

}


#endif
