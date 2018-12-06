#ifndef ROSNEURO_ACQUISITION_DEVICE_CPP
#define ROSNEURO_ACQUISITION_DEVICE_CPP

#include "rosneuro_acquisition/Device.hpp"

namespace rosneuro {
	namespace acquisition {

Device::Device(void) {
	this->devname_ = "undefined";
}

Device::~Device(void) {
	if(this->devcap_ != nullptr)
		delete this->devcap_;
}

char* Device::GetModel(void) {
	return this->devcap_->model;
}

char* Device::GetId(void) {
	return this->devcap_->id;
}

unsigned int Device::GetSamplingRate(void) {
	return this->devcap_->sampling_rate;
}

std::string Device::GetName(void) {
	return this->devname_;
}

void Device::Who(void) {
	printf("[%s] - %s device\n", this->GetName().c_str(), this->GetName().c_str());
}


	}
}

#endif
