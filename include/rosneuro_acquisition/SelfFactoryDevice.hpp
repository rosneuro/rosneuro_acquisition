#ifndef ROSNEURO_ACQUISITION_SELF_FACTORY_DEVICE_HPP
#define ROSNEURO_ACQUISITION_SELF_FACTORY_DEVICE_HPP

#include <memory>
#include <map>

namespace rosneuro {

class Device;
typedef Device*(*InstanceCreator)();

class SelfFactoryDevice {

	public:
		static SelfFactoryDevice& get();
		bool Register(const std::string name, const InstanceCreator& funcCreate);
		Device* createDevice(const std::string& name);

	private:
		SelfFactoryDevice() {};
		SelfFactoryDevice(const SelfFactoryDevice&) {};
		~SelfFactoryDevice() {};

	private:
		std::map<std::string, InstanceCreator> m_devices_;

};


}

#endif

