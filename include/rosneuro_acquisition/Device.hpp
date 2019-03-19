#ifndef ROSNEURO_ACQUISITION_DEVICE_HPP
#define ROSNEURO_ACQUISITION_DEVICE_HPP

#include <string>
#include <vector>

#include "rosneuro_data/NeuroData.hpp"

namespace rosneuro {

struct DeviceInfo {
	std::string		model;
	std::string  	id;
};

class Device {
	
	public:
		Device(NeuroFrame* frame);
		virtual ~Device(void);

		virtual bool   Setup(float framerate) = 0;
		virtual bool   Open(const std::string& devname) = 0;
		virtual bool   Close(void)	= 0;
		virtual bool   Start(void)	= 0;
		virtual bool   Stop(void)	= 0;
		virtual size_t Get(void)	= 0;
		virtual size_t GetAvailable(void) = 0;


		virtual std::string GetName(void);
		virtual void Who(void);
		virtual void Dump(void);

	protected:
		std::string	name_;
		NeuroFrame* frame_;

	public:
		DeviceInfo	devinfo;

};


}


#endif
