#ifndef ROSNEURO_ACQUISITION_DEVICE_CPP
#define ROSNEURO_ACQUISITION_DEVICE_CPP

#include "rosneuro_acquisition/Device.hpp"

namespace rosneuro {

Device::Device(void) : eeg("EEG"), exg("EXG"), tri("TRI") {
	this->name_			 = "undefined";
	this->sampling_rate_ = -1;
}

Device::~Device(void) {}

std::string Device::GetName(void) {
	return this->name_;
}

unsigned int Device::GetSamplingRate(void) {
	return this->sampling_rate_;
}

void Device::Who(void) {
	printf("[%s] - %s device\n", this->GetName().c_str(), this->GetName().c_str());
}


void Device::Dump(void) {
	printf("[Dump] %s info:\n", this->GetName().c_str());
	printf(" |- Model:         %s\n",	this->devinfo.model.c_str());
	printf(" |- Id:            %s\n",	this->devinfo.id.c_str());
}


}

#endif
