#ifndef ROSNEURO_ACQUISITION_DEVICEFACTORY_HPP
#define ROSNEURO_ACQUISITION_DEVICEFACTORY_HPP

#include <memory>
#include "rosneuro_acquisition/Device.hpp"
#include "rosneuro_acquisition/EGDDevice.hpp"
#include "rosneuro_acquisition/DummyDevice.hpp"

namespace rosneuro {
	namespace acquisition {

enum DeviceType {EGDDEV, DUMMYDEV};

class DeviceFactory {

	public:
		std::unique_ptr<Device> createDevice(unsigned int type = DeviceType::EGDDEV);

};

	}
}


#endif
