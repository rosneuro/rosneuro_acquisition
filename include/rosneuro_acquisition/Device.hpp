#ifndef ROSNEURO_ACQUISITION_DEVICE_HPP
#define ROSNEURO_ACQUISITION_DEVICE_HPP

#include <string>
#include <vector>

#include "rosneuro_acquisition/NeuroData.hpp"

namespace rosneuro {

struct DeviceCap {
	std::string		model;
	std::string  	id;
	unsigned int 	sampling_rate;
	unsigned int	nsamples;
};

class Device {
	
	public:
		Device(void);
		virtual ~Device(void);

		virtual bool   Setup(float fs) = 0;
		virtual bool   Open(const std::string& devname) = 0;
		virtual bool   Close(void)	= 0;
		virtual bool   Start(void)	= 0;
		virtual bool   Stop(void)	= 0;
		virtual size_t Get(void)	= 0;
		virtual size_t GetAvailable(void) = 0;

		virtual NeuroData* GetData(void);
		virtual DeviceCap* GetCapabilities(void);

		virtual std::string GetName(void);
		virtual void Who(void);
		virtual void Dump(void);

	protected:
		std::string	name_;
		NeuroData	neurodata_;
		DeviceCap	devicecap_;

};


}


#endif
