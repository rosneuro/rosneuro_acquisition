#ifndef ROSNEURO_ACQUISITION_SELF_FACTORY_DEVICE_CPP
#define ROSNEURO_ACQUISITION_SELF_FACTORY_DEVICE_CPP

#include "rosneuro_acquisition/SelfFactoryDevice.hpp"

namespace rosneuro {

SelfFactoryDevice& SelfFactoryDevice::get() {
	static SelfFactoryDevice instance;
	return instance;
}

bool SelfFactoryDevice::Register(const std::string name, const InstanceCreator& funcCreate) {
	return this->m_devices_.insert(std::make_pair(name, funcCreate)).second;
}

Device* SelfFactoryDevice::createDevice(const std::string& name) {
	auto it = this->m_devices_.find(name);

    if (it != this->m_devices_.end()) {
        return it->second();
    }
 
    return nullptr;
}


}

#endif
