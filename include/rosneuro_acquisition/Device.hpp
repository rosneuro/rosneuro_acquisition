#ifndef ROSNEURO_ACQUISITION_DEVICE_HPP
#define ROSNEURO_ACQUISITION_DEVICE_HPP

#include <string>

namespace rosneuro {
	namespace acquisition {

struct DevCapabilities {
	char*		 model;
	char*		 id;
	unsigned int sampling_rate;
};

class Device {
	
	public:
		Device(void);
		virtual ~Device(void);

		virtual bool   Setup(float fs) = 0;
		virtual bool   Open(const std::string& devname) = 0;
		virtual bool   Close(void) = 0;
		virtual bool   Start(void) = 0;
		virtual bool   Stop(void) = 0;
		virtual size_t GetData(void) = 0;
		virtual size_t GetAvailable(void) = 0;

		virtual char* GetModel(void);
		virtual char* GetId(void);
		virtual unsigned int GetSamplingRate(void);
		virtual std::string GetName(void);
		
		virtual void Who(void);

	protected:
		DevCapabilities* devcap_;
		std::string		 devname_;

};


	}
}


#endif
