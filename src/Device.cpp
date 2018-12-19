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

DeviceCap* Device::GetCapabilities(void) {
	return &(this->devicecap_);
}

NeuroData* Device::GetData(void) {
	return &(this->neurodata_);
}

void Device::Dump(void) {
	printf("[Dump] %s info:\n", this->GetName().c_str());
	printf(" + Device Capabilities:\n");
	printf(" |- Model:         %s\n",	this->devicecap_.model.c_str());
	printf(" |- Id:            %s\n",	this->devicecap_.id.c_str());
	printf(" |- Sampling rate: %d Hz\n", this->devicecap_.sampling_rate);
	printf(" |- Frame size:    %d Hz\n", this->devicecap_.nsamples);
	for(auto it = this->neurodata_.BeginInfo(); it != this->neurodata_.EndInfo(); ++it) {
		printf(" + %s group:\n", (*it).name.c_str());
		printf(" |- unit:         %s\n",	(*it).unit.c_str());
		printf(" |- transducter:  %s\n",	(*it).transducter.c_str());
		printf(" |- prefiltering: %s\n",	(*it).prefiltering.c_str());
		printf(" |- min/max: [%f %f]\n",	(*it).minmax[0], (*it).minmax[1]);
		printf(" |- isint:       %d\n",		(*it).isint);
		printf(" |- nchannels:   %u\n",		(*it).nchannels);
		printf(" |- labels: ");
		for(auto itl = (*it).labels.begin(); itl != (*it).labels.end(); ++itl)
			printf("%s ", (*itl).c_str());
		printf("\n");
	}
}


}

#endif
