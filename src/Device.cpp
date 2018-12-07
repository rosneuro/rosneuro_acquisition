#ifndef ROSNEURO_ACQUISITION_DEVICE_CPP
#define ROSNEURO_ACQUISITION_DEVICE_CPP

#include "rosneuro_acquisition/Device.hpp"

namespace rosneuro {
	namespace acquisition {

Device::Device(void) {
	this->name_  = "undefined";
	this->model_ = "undefined";
	this->id_	 = "undefined";
	this->sampling_rate_ = 0;
}

Device::~Device(void) {}

std::string Device::GetModel(void) {
	return this->model_;
}

std::string Device::GetId(void) {
	return this->id_;
}

unsigned int Device::GetSamplingRate(void) {
	return this->sampling_rate_;
}

std::string Device::GetName(void) {
	return this->name_;
}

void Device::Who(void) {
	printf("[%s] - %s device\n", this->GetName().c_str(), this->GetName().c_str());
}


	}
}

#endif
