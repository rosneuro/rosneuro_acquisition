#ifndef ROSNEURO_ACQUISITION_EGDDEVICE_HPP
#define ROSNEURO_ACQUISITION_EGDDEVICE_HPP

#include <errno.h>
#include <string.h>
#include <iostream>
#include <cstring>
#include <eegdev.h>
#include "rosneuro_acquisition/Device.hpp"

#define EGD_DEFAULT_GROUP_NUMBER 3
#define EGD_MAXSIZE_CHANNEL_NAME 32

namespace rosneuro {
	namespace acquisition {

struct EGDCapabilities {
	std::string  model;
	std::string  id;
	unsigned int sampling_rate;
	unsigned int eeg_nmax;
    unsigned int sensor_nmax;
    unsigned int trigger_nmax;
	std::string	 prefiltering;
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

		const std::string GetPrefiltering(void);
		const char*** GetLabels(void);
		
		void Dump(void);
			
	private:
		void init_egd_capabilities(void);
		void init_egd_groups(void);
		void init_egd_strides(void);
		void init_egd_labels(void);
		void init_egd_data(void);

		bool setup_egd_capabilities(void);
		bool setup_egd_groups(void);
		bool setup_egd_strides(void);
		bool setup_egd_data(void);
		bool setup_egd_labels(void);
		bool setup_egd_frame(float hz);

		void destroy_egd_data(void);
		void destroy_egd_cababilities(void);
		void destroy_egd_strides(void);
		void destroy_egd_labels(void);
		void destroy_egd_groups(void);

		size_t get_egd_size(int egdtype);

	protected:
		struct  eegdev*	egddev_;
		struct	grpconf* grp_;
		size_t*	strides_;
		char***	labels_;
		void*	eeg_;
		void*	exg_;
		void*	tri_;
		size_t	frames_;
		unsigned int ngrp_;
		EGDCapabilities* egdcap_;
};

	}
}


#endif
