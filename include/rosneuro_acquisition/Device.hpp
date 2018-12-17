#ifndef ROSNEURO_ACQUISITION_DEVICE_HPP
#define ROSNEURO_ACQUISITION_DEVICE_HPP

#include <string>
#include <vector>

namespace rosneuro {

struct DeviceData {
	size_t			seeg;
	size_t			sexg;
	size_t			stri;
	void*			eeg;
	void* 			exg;
	void* 			tri;
};

struct DeviceCapabilities {
	std::string					model;
	std::string  				id;
	std::string	 				prefiltering;
	unsigned int 				sampling_rate;
	unsigned int 				neeg;
    unsigned int 				nexg;
    unsigned int 				ntri;
	unsigned int				nsamples;
	std::vector<std::string>	leeg;
	std::vector<std::string>	lexg;
	std::vector<std::string>	ltri;
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

		virtual DeviceData* GetData(void);
		virtual DeviceCapabilities* GetCapabilities(void);
		virtual std::string GetName(void);
		
		virtual void Who(void);
		virtual void Dump(void) {};

	protected:
		std::string			name_;
		DeviceData			data_;
		DeviceCapabilities	cap_;

};


}


#endif
