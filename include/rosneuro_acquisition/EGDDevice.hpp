#ifndef ROSNEURO_ACQUISITION_EGDDEVICE_HPP
#define ROSNEURO_ACQUISITION_EGDDEVICE_HPP

#include <errno.h>
#include <string.h>
#include <iostream>
#include <cstring>
#include <eegdev.h>
#include "rosneuro_acquisition/Device.hpp"
#include "rosneuro_acquisition/NeuroData.hpp"

#define EGD_DEFAULT_GROUP_NUMBER 3
#define EGD_MAXSIZE_CHANNEL_NAME 32

namespace rosneuro {


class EGDDevice : public Device {

	public:
		EGDDevice(void);
		virtual ~EGDDevice(void);

		bool Setup(float fs);
		bool Open(const std::string& devname);
		bool Close(void);
		bool Start(void);
		bool Stop(void);
		size_t Get(void);
		size_t GetAvailable(void);
			
	private:
		void init_dev_capabilities(void);
		void init_egd_structures(void);
		void init_neuro_data(void);

		bool setup_dev_capabilities(float hz);
		bool setup_egd_structures(void);
		bool setup_neuro_data(void);
		
		void destroy_egd_structures(void);

		size_t get_egd_size(int egdtype);

	protected:
		struct  eegdev*	egddev_;
		struct	grpconf* grp_;
		size_t*	strides_;
		unsigned int ngrp_;
};

}


#endif
