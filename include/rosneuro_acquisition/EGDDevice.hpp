#ifndef ROSNEURO_ACQUISITION_EGDDEVICE_HPP
#define ROSNEURO_ACQUISITION_EGDDEVICE_HPP

#include <errno.h>
#include <string.h>
#include <iostream>
#include <cstring>
#include <eegdev.h>
#include "rosneuro_acquisition/Device.hpp"

namespace rosneuro {
	namespace acquisition {

struct EGDCapabilities : DevCapabilities {
	unsigned int eeg_nmax;
    unsigned int sensor_nmax;
    unsigned int trigger_nmax;
	char*		 prefiltering;
};

class EGDDevice : public Device {

	public:
		EGDDevice(void);
		virtual ~EGDDevice(void);

		bool Setup(float fs);
		bool Open(const std::string& devname);
		bool Close(void);
		bool Start(void);
		bool Stop(void);
		size_t GetData(void);
		size_t GetAvailable(void);

		const char* GetPrefiltering(void);
		const char*** GetLabels(void);
			
	protected:
		void InitCapabilities(void);
		void InitGroups(void);
		void InitBuffers(void);
		void InitFrame(float hz);
		size_t SizeEGD(int egdtype);

	private:
		struct  eegdev*	egddev_;
		struct	grpconf grp_[3];
		size_t	strides_[3];
		void*	eeg_;
		void*	exg_;
		void*	tri_;
		size_t	frames_;
		char**	labels_[3];
		EGDCapabilities* egdcap_;
};

	}
}


#endif
