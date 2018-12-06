#ifndef ROSNEURO_ACQUISITION_DEVICE_CPP
#define ROSNEURO_ACQUISITION_DEVICE_CPP

#include "rosneuro_acquisition/Device.hpp"

namespace rosneuro {
	namespace acquisition {

Device::Device(void) {
	this->devcap_->model		  = nullptr;
	this->devcap_->id			  = nullptr;
	this->devcap_->sampling_rate  = 0;
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


	}
}

#endif
