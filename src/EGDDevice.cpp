#ifndef ROSNEURO_ACQUISITION_EGDDEVICE_CPP
#define ROSNEURO_ACQUISITION_EGDDEVICE_CPP

#include "rosneuro_acquisition/EGDDevice.hpp"

namespace rosneuro {

EGDDevice::EGDDevice(void) {
	
	this->name_ = "egddev";
	this->ngrp_ = EGD_DEFAULT_GROUP_NUMBER;

	// Initialize device capabilities structure
	this->init_dev_capabilities();

	// Initialize eegdev structures
	this->init_egd_structures();

	// Initialize neuro data
	this->init_neuro_data();

}

EGDDevice::~EGDDevice(void) {
	this->destroy_egd_structures();
}

bool EGDDevice::Setup(float hz) {

	
	// Setup structures
	if(this->setup_dev_capabilities(hz) == false) {
		std::cerr<<"[Error] - Cannot setup capabilities"<<std::endl; 
		return false;
	}
	
	
	if(this->setup_egd_structures() == false) {
		std::cerr<<"[Error] - Cannot setup group"<<std::endl; 
		return false;
	}
	
	if(this->setup_neuro_data() == false) {
		std::cerr<<"[Error] - Cannot setup data"<<std::endl; 
		return false;
	}

	// Setup egd device
	if(egd_acq_setup(this->egddev_, 3, this->strides_, 3, this->grp_) == -1) {
		std::cerr<<"Cannot setup the device: " << std::strerror(errno)<<std::endl;
		return false;
	}
	
	
	return true;
}

bool EGDDevice::Open(const std::string& devname) {

	std::string devnamearg;
	if(devname.find(".bdf") != std::string::npos) 
		devnamearg.assign("datafile|path|");
	else if(devname.find(".gdf") != std::string::npos) 
		devnamearg.assign("datafile|path|");
	devnamearg.append(devname);
	
	this->name_ = devname;

	if(!(this->egddev_ = egd_open(devnamearg.c_str()))) {
		std::cerr<<"[Error] - Cannot open the device '"<< this->GetName() <<"': "
				 << std::strerror(errno) <<std::endl;
		return false;
	}


	return true;
}

bool EGDDevice::Close(void) {
	if(egd_close(this->egddev_) == -1) {
		std::cerr<<"[Error] - Cannot close the device '" << this->GetName() <<"': "
			     << std::strerror(errno)<<std::endl;
		return false;
	}
	
	return true;
}
		
bool EGDDevice::Start(void) {
	if(egd_start(this->egddev_) == -1) { 
		std::cerr<<"[Error] - Cannot start the device '" << this->GetName() <<"': "
			     << std::strerror(errno) << std::endl;
		return false;
	}
	return true;
}

bool EGDDevice::Stop(void) {
	if(egd_stop(this->egddev_) == -1) { 
		std::cerr<<"[Error] - Cannot stop the device '" << this->GetName() << "': "
			     << std::strerror(errno) <<std::endl;
		return false;
	}
	return true;
}

size_t EGDDevice::Get(void) {
	size_t size;
	size = egd_get_data(this->egddev_, this->devicecap_.nsamples, 
						this->neurodata_.data[0], 
						this->neurodata_.data[1], 
						this->neurodata_.data[2]);
	if (size == -1) {
		std::cerr<<"Error reading data: " << std::strerror(errno) << std::endl;
	}
	
	return size;
}
		
size_t EGDDevice::GetAvailable(void) {
	return egd_get_available(this->egddev_);
}





/*********************************************************************/
/*					Private/protected methods						 */
/*********************************************************************/

void EGDDevice::init_dev_capabilities(void) {
	this->devicecap_.model			= "";
	this->devicecap_.id				= "";
	this->devicecap_.sampling_rate	= 0;
	this->devicecap_.nsamples		= 0;
}

void EGDDevice::init_neuro_data(void) {
	this->neurodata_.SetGroups(this->ngrp_);
}

void EGDDevice::init_egd_structures(void) {

	// Group structure
	this->grp_ = (grpconf*)malloc(this->ngrp_ * sizeof(grpconf));
	memset(this->grp_, 0, this->ngrp_*sizeof(struct grpconf));
	
	// Strides
	this->strides_ = (size_t*)malloc(this->ngrp_ * sizeof(size_t));
	memset(this->strides_, 0, this->ngrp_ *sizeof(size_t));
}



bool EGDDevice::setup_dev_capabilities(float hz) {

	char* model			= nullptr;
	char* id			= nullptr;
	unsigned int fs		= 0; 

	// Getting sampling rate
	if(egd_get_cap(this->egddev_, EGD_CAP_FS, &fs) == -1) {
		std::cerr<<"[Error] - Cannot get sampling rate: "<<strerror(errno)<<std::endl;
		return false;
	}

	// Getting devtype
	if(egd_get_cap(this->egddev_, EGD_CAP_DEVTYPE, &model) == -1) {
		std::cerr<<"[Error] - Cannot get device type: "<<strerror(errno)<<std::endl;
		return false;
	}

	// Getting devid
	if(egd_get_cap(this->egddev_, EGD_CAP_DEVID, &id) == -1) {
		std::cerr<<"[Error] - Cannot get device id: "<<strerror(errno)<<std::endl;
		return false;
	}


	// Populating device capabilities and data structure
	this->devicecap_.sampling_rate	= fs;
	this->devicecap_.model			= std::string(model);
	this->devicecap_.id				= std::string(id);
	this->devicecap_.nsamples		= (size_t)((float)this->devicecap_.sampling_rate/hz);

	return true;
}

bool EGDDevice::setup_egd_structures(void) {

	int neeg, nexg, ntri;

	/**** Setting up the groups ****/
	if(this->grp_ == nullptr) {
		std::cerr<<"[Error] - Groups are not allocated"<<std::endl;
		return false;
	}

	// Getting number EEG, EXG, TRIGGER channels
	if( (neeg = egd_get_numch(this->egddev_, EGD_EEG)) == -1) {
		std::cerr<<"[Error] - Cannot get number EEG channels: "<<strerror(errno)<<std::endl;
		return false;
	}
	
	if( (nexg = egd_get_numch(this->egddev_, EGD_SENSOR)) == -1) {
		std::cerr<<"[Error] - Cannot get number SENSOR channels: "<<strerror(errno)<<std::endl;
		return false;
	}

	if( (ntri = egd_get_numch(this->egddev_, EGD_TRIGGER)) == -1) {
		std::cerr<<"[Error] - Cannot get number TRIGGER channels: "<<strerror(errno)<<std::endl;
		return false;
	}

	this->grp_[0].sensortype = EGD_EEG;
	this->grp_[0].index		 = 0;
	this->grp_[0].iarray	 = 0;
	this->grp_[0].datatype	 = EGD_FLOAT;
	this->grp_[0].arr_offset = 0;
	this->grp_[0].nch		 = neeg;
	
	this->grp_[1].sensortype = EGD_SENSOR;
	this->grp_[1].index		 = 0; 
	this->grp_[1].iarray	 = 1; 
	this->grp_[1].datatype	 = EGD_FLOAT;
	this->grp_[1].arr_offset = 0;
	this->grp_[1].nch	     = nexg;
	
	this->grp_[2].sensortype = EGD_TRIGGER;
	this->grp_[2].index		 = 0; 
	this->grp_[2].iarray	 = 2;
	this->grp_[2].datatype	 = EGD_INT32;
	this->grp_[2].arr_offset = 0;
	this->grp_[2].nch		 = ntri;

	
	/**** Setting up the strides ****/
	if(this->strides_ == nullptr) {
		std::cerr<<"[Error] - Strides are not allocated"<<std::endl;
		return false;
	}

	// Setup the strides so that we get packed data into the buffers
	this->strides_[0] = this->grp_[0].nch * this->get_egd_size(EGD_FLOAT);
	this->strides_[1] = this->grp_[1].nch * this->get_egd_size(EGD_FLOAT);
	this->strides_[2] = this->grp_[2].nch * this->get_egd_size(EGD_INT32);

	return true;
}

bool EGDDevice::setup_neuro_data(void) {
	
	size_t seeg, sexg, stri;
	char unit[16], transducter[128], filtering[128], label[32];
	double mm[2];
	int isint = 0;

	if(this->strides_ == nullptr) {
		std::cerr<<"[Error] - Strides are not allocated"<<std::endl;
		return false;
	}

	// Compute sizes so not to call malloc if size == 0
	seeg = this->strides_[0]*this->devicecap_.nsamples;
	sexg = this->strides_[1]*this->devicecap_.nsamples;
	stri = this->strides_[2]*this->devicecap_.nsamples;

	this->neurodata_.data[0] = seeg ? (void*)malloc(seeg) : nullptr;
	this->neurodata_.data[1] = sexg ? (void*)malloc(sexg) : nullptr;
	this->neurodata_.data[2] = stri ? (void*)malloc(stri) : nullptr;


	// Get data groups name
	this->neurodata_.info[0].name = "eeg";
	this->neurodata_.info[1].name = "exg";
	this->neurodata_.info[2].name = "tri";

	// Get additional information
	for(auto i = 0; i<this->ngrp_; i++) {
		egd_channel_info(this->egddev_, this->grp_[i].sensortype, 0, 
					 EGD_UNIT, unit, EGD_TRANSDUCTER, transducter, 
					 EGD_PREFILTERING, filtering, EGD_MM_D, mm,
					 EGD_ISINT, &isint, EGD_EOL);

		this->neurodata_.info[i].unit			= std::string(unit);
		this->neurodata_.info[i].transducter	= std::string(transducter);
		this->neurodata_.info[i].prefiltering	= std::string(filtering);
		this->neurodata_.info[i].minmax[0]		= mm[0];
		this->neurodata_.info[i].minmax[1]		= mm[1];
		this->neurodata_.info[i].isint			= isint;
		this->neurodata_.info[i].nchannels		= this->grp_[i].nch;

		for(auto j = 0; j<this->grp_[i].nch; j++) {
			egd_channel_info(this->egddev_, this->grp_[i].sensortype, j,
							 EGD_LABEL, label, EGD_EOL);
			this->neurodata_.info[i].labels.push_back(std::string(label));
		}
	}
	


	return true;
}


void EGDDevice::destroy_egd_structures(void) {
	if(this->strides_ != nullptr)
		free(this->strides_);
	this->strides_ = nullptr;
	
	if(this->grp_ != nullptr)
		free(this->grp_);
	this->grp_ = nullptr;
}


size_t EGDDevice::get_egd_size(int egdtype) {
	size_t size = 0;
	switch(egdtype) {
		case EGD_INT32:
			size = sizeof(int32_t);
			break;
		case EGD_FLOAT:
			size = sizeof(float);
			break;
		case EGD_DOUBLE:
			size = sizeof(double);
			break;
		default:
			std::cout << "[Warning] - EGD type not known (" << egdtype 
				      << ": forcing EGD_FLOAT" << std::endl;
			size = sizeof(float);
	}
	return size;
}


}

#endif
