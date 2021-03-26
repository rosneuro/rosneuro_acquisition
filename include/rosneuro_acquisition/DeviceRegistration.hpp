#ifndef ROSNEURO_DEVICE_REGISTRATION_HPP
#define ROSNEURO_DEVICE_REGISTRATION_HPP

#include "rosneuro_acquisition/FactoryDevice.hpp"

namespace rosneuro {

	template <typename T>
	class DeviceRegistration {

		public:
			DeviceRegistration(std::string name) {
				FactoryDevice::get().Register(name, []() {
						return static_cast<Device*>(new T()); }
						);
			}

	};


}

#endif

