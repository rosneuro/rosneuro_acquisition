#ifndef ROSNEURO_ACQUISITION_DUMMYDEVICE_CPP
#define ROSNEURO_ACQUISITION_DUMMYDEVICE_CPP

#include "rosneuro_acquisition/DummyDevice.hpp"

namespace rosneuro {

DummyDevice::DummyDevice(NeuroFrame* frame) : Device(frame) {
	this->name_ = "dummy";
}

DummyDevice::DummyDevice(void) : Device() {
	this->name_ = "dummy";
}

DummyDevice::~DummyDevice(void) {}

bool DummyDevice::Setup(float framerate) {
	printf("[%s] - Setup done\n", this->name_.c_str());
	return true;
}

bool DummyDevice::Open(const std::string& devname, int samplerate) {
	printf("[%s] - Device open\n", this->name_.c_str());
	return true;
}

bool DummyDevice::Close(void) {
	printf("[%s] - Device closed\n", this->name_.c_str());
	return true;
}

bool DummyDevice::Start(void) {
	printf("[%s] - Device started\n", this->name_.c_str());
	return true;
}

bool DummyDevice::Stop(void) {
	printf("[%s] - Device stopped\n", this->name_.c_str());
	return true;
}

size_t DummyDevice::Get(void) {
	printf("[%s] - Get\n", this->name_.c_str());
	return 1;
}

size_t DummyDevice::GetAvailable(void) {
	printf("[%s] - Get available data\n", this->name_.c_str());
	return 0;
}



}

#endif
