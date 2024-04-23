#ifndef ROSNEURO_ACQUISITION_DEVICE_CPP
#define ROSNEURO_ACQUISITION_DEVICE_CPP
#include "Device.hpp"

namespace rosneuro {
    Device::Device(void) {
        this->name_  = "undefined";
        this->frame_ = nullptr;
    }

    Device::Device(NeuroFrame* frame)  {
        this->name_	 = "undefined";
        this->frame_ = frame;
    }

    Device::~Device(void) {}


    std::string Device::GetName(void) {
        return this->name_;
    }


    void Device::Who(void) {
        printf("[%s] - %s device\n", this->name_.c_str(), this->name_.c_str());
    }


    void Device::Dump(void) {
        printf("[Dump] %s info:\n", this->name_.c_str());
        printf(" |- Model:         %s\n",	this->devinfo.model.c_str());
        printf(" |- Id:            %s\n",	this->devinfo.id.c_str());
    }
}

#endif
