#ifndef ROSNEURO_ACQUISITION_DEVICE_HPP
#define ROSNEURO_ACQUISITION_DEVICE_HPP

#include <string>

namespace rosneuro {
	namespace acquisition {

struct DeviceData {
	unsigned int	neeg;
	unsigned int 	nexg;
	unsigned int 	ntri;
	size_t			seeg;
	size_t			sexg;
	size_t			stri;
	void*			eeg;
	void* 			exg;
	void* 			tri;
	char**			leeg;
	char**			lexg;

	size_t			sframe;
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
		virtual size_t GetData(DeviceData* data) = 0;
		virtual size_t GetAvailable(void) = 0;

		virtual std::string GetModel(void);
		virtual std::string GetId(void);
		virtual unsigned int GetSamplingRate(void);
		virtual std::string GetName(void);
		
		virtual void Who(void);

		virtual void Dump(void) {};

	protected:
		std::string		name_;
		std::string		model_;
		std::string		id_;
		unsigned int	sampling_rate_;
		DeviceData*		data_;

};


	}
}


#endif
