#ifndef ROSNEURO_ACQUISITION_FACTORY_DEVICE_CPP
#define ROSNEURO_ACQUISITION_FACTORY_DEVICE_CPP

#include "rosneuro_acquisition/FactoryDevice.hpp"

namespace rosneuro {

FactoryDevice& FactoryDevice::get() {
	static FactoryDevice instance;
	return instance;
}

bool FactoryDevice::Register(const std::string name, const InstanceCreator& funcCreate) {
	return this->m_devices_.insert(std::make_pair(name, funcCreate)).second;
}

Device* FactoryDevice::createDevice(const std::string& name) {
	auto it = this->m_devices_.find(name);

    if (it != this->m_devices_.end()) {
        return it->second();
    }
 
    return nullptr;
}


}

#endif
