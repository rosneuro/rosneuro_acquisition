#ifndef ROSNEURO_ACQUISITION_DEVICE_CPP
#define ROSNEURO_ACQUISITION_DEVICE_CPP

#include "rosneuro_acquisition/Device.hpp"

namespace rosneuro {

Device::Device(void) {
	this->name_  = "undefined";
}

Device::~Device(void) {}

std::string Device::GetName(void) {
	return this->name_;
}

void Device::Who(void) {
	printf("[%s] - %s device\n", this->GetName().c_str(), this->GetName().c_str());
}

DeviceCapabilities* Device::GetCapabilities(void) {
	return &(this->devcap_);
}

NeuroData* Device::GetData(void) {
	return this->neurodata_;
}


}

#endif
