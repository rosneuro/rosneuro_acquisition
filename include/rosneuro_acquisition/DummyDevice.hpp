#ifndef ROSNEURO_ACQUISITION_DUMMYDEVICE_HPP
#define ROSNEURO_ACQUISITION_DUMMYDEVICE_HPP

#include <errno.h>
#include <string.h>
#include "rosneuro_acquisition/Device.hpp"

// Created by L.Tonin  <luca.tonin@epfl.ch> on 06/12/18 16:22:48
// Dummy device to test the device factory

namespace rosneuro {
	namespace acquisition {

class DummyDevice : public Device {

	public:
		DummyDevice(void);
		virtual ~DummyDevice(void);

		bool Setup(float fs);
		bool Open(const std::string& devname);
		bool Close(void);
		bool Start(void);
		bool Stop(void);
		size_t GetData(DeviceData* data);
		size_t GetAvailable(void);
		void Dump(void);
};

	}
}


#endif
