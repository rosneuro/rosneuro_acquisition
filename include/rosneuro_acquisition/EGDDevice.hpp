#ifndef ROSNEURO_ACQUISITION_EGDDEVICE_HPP
#define ROSNEURO_ACQUISITION_EGDDEVICE_HPP

#include <errno.h>
#include <string.h>
#include <iostream>
#include <cstring>
#include <eegdev.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_acquisition/Device.hpp"

#define EGD_DATA_GROUPS 3

namespace rosneuro {


class EGDDevice : public Device {

	public:
		EGDDevice(NeuroFrame* frame);
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

		bool setup_dev_capabilities(void);
		bool setup_neuro_data(float hz);
		void setup_neuro_info(NeuroDataInfo* data, size_t nch, unsigned int index);
		bool setup_egd_structures(void);
		
		void destroy_egd_structures(void);


	protected:
		struct  eegdev*	egddev_;
		struct	grpconf* grp_;
		size_t* strides_;
};

}


#endif
