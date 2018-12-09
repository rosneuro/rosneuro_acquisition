#ifndef ROSNEURO_ACQUISITION_DUMMYDEVICE_CPP
#define ROSNEURO_ACQUISITION_DUMMYDEVICE_CPP

#include "rosneuro_acquisition/DummyDevice.hpp"

namespace rosneuro {
	namespace acquisition {

DummyDevice::DummyDevice(void) {
	this->name_ = "dummy";
}
DummyDevice::~DummyDevice(void) {}

bool DummyDevice::Setup(float fs) {
	//printf("[%s] - Setup done\n", this->GetName().c_str());
	printf("[%s] - Setup done\n", this->name_.c_str());
	return true;
}

bool DummyDevice::Open(const std::string& devname) {
	printf("[%s] - Device open\n", this->GetName().c_str());
	return true;
}

bool DummyDevice::Close(void) {
	printf("[%s] - Device closed\n", this->GetName().c_str());
	return true;
}

bool DummyDevice::Start(void) {
	printf("[%s] - Device started\n", this->GetName().c_str());
	return true;
}

bool DummyDevice::Stop(void) {
	printf("[%s] - Device stopped\n", this->GetName().c_str());
	return true;
}

size_t DummyDevice::GetData(DeviceData* data) {
	printf("[%s] - Get data\n", this->GetName().c_str());
	return 1;
}

size_t DummyDevice::GetAvailable(void) {
	printf("[%s] - Get available data\n", this->GetName().c_str());
	return 0;
}

void DummyDevice::Dump(void) {
	printf("[Dump] DummyDevice info:\n");
	printf(" + Capabilities:\n");
	printf(" |- Device:       %s\n",	"DUMMY");
	printf(" |- Id:           %s\n",	"0.0.0");
	printf(" |- Sf:           %d Hz\n", 0);
	printf(" |- Channels:     %d\n",	0);
	printf(" |- Sensors:      %d\n",	0);
	printf(" |- Triggers:     %d\n",	0);
	printf(" |- Prefiltering: %s\n",	"None");

}


	}
}

#endif
