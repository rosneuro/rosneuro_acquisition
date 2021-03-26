#ifndef ROSNEURO_ACQUISITION_FACTORY_DEVICE_HPP
#define ROSNEURO_ACQUISITION_FACTORY_DEVICE_HPP

#include <memory>
#include <map>

namespace rosneuro {

class Device;
typedef Device*(*InstanceCreator)();

class FactoryDevice {

	public:
		static FactoryDevice& get();
		bool Register(const std::string name, const InstanceCreator& funcCreate);
		Device* createDevice(const std::string& name);

	private:
		FactoryDevice() {};
		FactoryDevice(const FactoryDevice&) {};
		~FactoryDevice() {};

	private:
		std::map<std::string, InstanceCreator> m_devices_;

};


}

#endif

