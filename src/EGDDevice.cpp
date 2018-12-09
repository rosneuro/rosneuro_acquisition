#ifndef ROSNEURO_ACQUISITION_EGDDEVICE_CPP
#define ROSNEURO_ACQUISITION_EGDDEVICE_CPP

#include "rosneuro_acquisition/EGDDevice.hpp"

namespace rosneuro {
	namespace acquisition {

EGDDevice::EGDDevice(void) {
	
	this->ngrp_ = EGD_DEFAULT_GROUP_NUMBER;
	this->init_egd_capabilities();
	this->init_egd_groups();
	this->init_egd_strides();
	this->init_egd_labels();
	this->init_egd_data();
}

EGDDevice::~EGDDevice(void) {
	this->destroy_egd_data();
	this->destroy_egd_cababilities();
	this->destroy_egd_strides();
	this->destroy_egd_labels();
	this->destroy_egd_groups();
}

bool EGDDevice::Setup(float hz) {

	// Setup structures
	if(this->setup_egd_capabilities() == false) {
		std::cerr<<"[Error] - Cannot setup capabilities"<<std::endl; 
		return false;
	}
	if(this->setup_egd_groups() == false) {
		std::cerr<<"[Error] - Cannot setup group"<<std::endl; 
		return false;
	}
	if(this->setup_egd_strides() == false) {
		std::cerr<<"[Error] - Cannot setup strides"<<std::endl; 
		return false;
	}
	if(this->setup_egd_data() == false) {
		std::cerr<<"[Error] - Cannot setup data"<<std::endl; 
		return false;
	}
	if(this->setup_egd_labels() == false) {
		std::cerr<<"[Error] - Cannot setup labels"<<std::endl; 
		return false;
	}

	if(this->setup_egd_frame(hz) == false) {
		std::cerr<<"[Error] - Cannot setup frame"<<std::endl; 
		return false;
	}

	// Setup egd device
	if(egd_acq_setup(this->egddev_, 3, this->strides_, 3, this->grp_) == -1) {
		std::cerr<<"Cannot setup the device: " << std::strerror(errno)<<std::endl;
		return false;
	}
	
	// Setup labels
	if(this->setup_egd_labels() == false)
		return false;
	
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

	this->destroy_egd_cababilities();
	this->egdcap_ = new EGDCapabilities();

	return true;
}

bool EGDDevice::Close(void) {
	if(egd_close(this->egddev_) == -1) {
		std::cerr<<"[Error] - Cannot close the device '" << this->GetName() <<"': "
			     << std::strerror(errno)<<std::endl;
		return false;
	}
	
	this->destroy_egd_cababilities();

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

size_t EGDDevice::GetData(void) {
	return egd_get_data(this->egddev_, this->frames_, 
			this->eeg_, this->exg_, this->tri_);
}
		
size_t EGDDevice::GetAvailable(void) {
	return egd_get_available(this->egddev_);
}

const std::string EGDDevice::GetPrefiltering(void) {
	return this->egdcap_->prefiltering;
}

const char*** EGDDevice::GetLabels(void) {
	return (const char***)this->labels_;
}

void EGDDevice::Dump(void) {
	printf("[Dump] Device info:\n");
	printf(" + Capabilities:\n");
	printf(" |- Device:       %s\n",	this->egdcap_->model.c_str());
	printf(" |- Id:           %s\n",	this->egdcap_->id.c_str());
	printf(" |- Sf:           %d Hz\n", this->egdcap_->sampling_rate);
	printf(" |- Channels:     %d\n",	this->egdcap_->eeg_nmax);
	printf(" |- Sensors:      %d\n",	this->egdcap_->sensor_nmax);
	printf(" |- Triggers:     %d\n",	this->egdcap_->trigger_nmax);
	printf(" |- Prefiltering: %s\n",	this->egdcap_->prefiltering.c_str());
	printf(" |- EEG labels: ");
	for(auto i=0; i<this->grp_[0].nch; i++)
		printf("%s ", this->labels_[0][i]);
	printf("\n |- Sensors labels: ");
	for(auto i=0; i<this->grp_[1].nch; i++)
		printf("%s ", this->labels_[1][i]);
	printf("\n");
}


/*********************************************************************/
/*					Private/protected methods						 */
/*********************************************************************/

void EGDDevice::init_egd_capabilities(void) {
	this->egdcap_ = nullptr;
}

void EGDDevice::init_egd_groups(void) {
	this->grp_ = (grpconf*)malloc(this->ngrp_ * sizeof(grpconf));
	memset(this->grp_, 0, this->ngrp_*sizeof(struct grpconf));
}

void EGDDevice::init_egd_labels(void) {
	this->labels_ = (char***)malloc(this->ngrp_ * sizeof(char**));
	memset(this->labels_, 0, this->ngrp_*sizeof(char**));
}

void EGDDevice::init_egd_strides(void) {
	this->strides_ = (size_t*)malloc(this->ngrp_ * sizeof(size_t));
	memset(this->strides_, 0, this->ngrp_ *sizeof(size_t));
}

void EGDDevice::init_egd_data(void) {
	this->eeg_	  = nullptr;
	this->exg_	  = nullptr;
	this->tri_    = nullptr;
	this->frames_ = 0;
}

void EGDDevice::destroy_egd_data(void) {
	if(this->eeg_ != nullptr)
		free(this->eeg_);
	if(this->exg_ != nullptr)
		free(this->exg_);
	if(this->tri_ != nullptr)
		free(this->tri_);

	this->frames_ = 0;
	this->eeg_    = nullptr;
	this->exg_    = nullptr;
	this->tri_    = nullptr;
}

void EGDDevice::destroy_egd_cababilities(void) {
	if(this->egdcap_ != nullptr)
		delete egdcap_;
}


void EGDDevice::destroy_egd_strides(void) {
	if(this->strides_ != nullptr)
		free(this->strides_);
	this->strides_ = nullptr;
}

void EGDDevice::destroy_egd_labels(void) {

	unsigned int igrp, i;
	
	for (igrp=0; igrp<this->ngrp_-1; igrp++) {
		for (i=0; i<this->grp_[igrp].nch; i++) 
			free(this->labels_[igrp][i]);

		free(this->labels_[igrp]);
	}

	free(this->labels_);
	this->labels_ = nullptr;
}

void EGDDevice::destroy_egd_groups(void) {
	if(this->grp_ != nullptr)
		free(this->grp_);
	this->grp_ = nullptr;
}


bool EGDDevice::setup_egd_capabilities(void) {

	char* model			= nullptr;
	char* id			= nullptr;
	unsigned int fs		= 0; 
	unsigned int neeg	= 0;
	unsigned int ntri	= 0;
	unsigned int nsen	= 0;
	char prefiltering[128];

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

	// Getting number EEG
	if( (neeg = egd_get_numch(this->egddev_, EGD_EEG)) == -1) {
		std::cerr<<"[Error] - Cannot get number EEG channels: "<<strerror(errno)<<std::endl;
		return false;
	}

	// Getting number TRIGGER
	if( (ntri = egd_get_numch(this->egddev_, EGD_TRIGGER)) == -1) {
		std::cerr<<"[Error] - Cannot get number TRIGGER channels: "<<strerror(errno)<<std::endl;
		return false;
	}

	// Getting number SENSOR
	if( (nsen = egd_get_numch(this->egddev_, EGD_SENSOR)) == -1) {
		std::cerr<<"[Error] - Cannot get number SENSOR channels: "<<strerror(errno)<<std::endl;
		return false;
	}
	
	// Getting prefiltering
	if( egd_channel_info(this->egddev_, EGD_EEG, 0, EGD_PREFILTERING, &prefiltering, EGD_EOL) == -1) {
		std::cerr<<"[Error] - Cannot get prefiltering: "<<strerror(errno)<<std::endl;
		return false;
	}

	// Populating device capabilities structure
	this->egdcap_->sampling_rate = fs;
	this->egdcap_->eeg_nmax		 = neeg;
	this->egdcap_->trigger_nmax  = ntri;
	this->egdcap_->sensor_nmax   = nsen;
	this->egdcap_->model		 = std::string(model);
	this->egdcap_->id			 = std::string(id);
	this->egdcap_->prefiltering  = std::string(prefiltering);

	return true;
}

bool EGDDevice::setup_egd_strides(void) {

	if(this->grp_ == nullptr) {
		std::cerr<<"[Error] - Groups are not allocated"<<std::endl;
		return false;
	}
	
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

bool EGDDevice::setup_egd_data(void) {

	if(this->strides_ == nullptr) {
		std::cerr<<"[Error] - Strides are not allocated"<<std::endl;
		return false;
	}
	
	// Compute sizes so not to call malloc if size == 0
	size_t seeg = this->strides_[0]*this->frames_;
	size_t sexg = this->strides_[1]*this->frames_;
	size_t stri = this->strides_[2]*this->frames_;

	this->eeg_ = seeg ? (void*)malloc(seeg) : nullptr;
	this->exg_ = sexg ? (void*)malloc(sexg) : nullptr;
	this->tri_ = stri ? (void*)malloc(stri) : nullptr;

	return true;
}

bool EGDDevice::setup_egd_groups(void) {
	
	if(this->grp_ == nullptr) {
		std::cerr<<"[Error] - Groups are not allocated"<<std::endl;
		return false;
	}
	
	if(this->egdcap_ == nullptr) {
		std::cerr<<"[Error] - Capabilities are not allocated"<<std::endl;
		return false;
	}

	this->grp_[0].sensortype = EGD_EEG;
	this->grp_[0].index		 = 0;
	this->grp_[0].iarray	 = 0;
	this->grp_[0].datatype	 = EGD_FLOAT;
	this->grp_[0].arr_offset = 0;
	this->grp_[0].nch		 = this->egdcap_->eeg_nmax;
	
	this->grp_[1].sensortype = EGD_SENSOR;
	this->grp_[1].index		 = 0; 
	this->grp_[1].iarray	 = 1; 
	this->grp_[1].datatype	 = EGD_FLOAT;
	this->grp_[1].arr_offset = 0;
	this->grp_[1].nch	     = this->egdcap_->sensor_nmax;
	
	this->grp_[2].sensortype = EGD_TRIGGER;
	this->grp_[2].index		 = 0; 
	this->grp_[2].iarray	 = 2;
	this->grp_[2].datatype	 = EGD_INT32;
	this->grp_[2].arr_offset = 0;
	this->grp_[2].nch		 = this->egdcap_->trigger_nmax;

	return true;
}

bool EGDDevice::setup_egd_frame(float hz) {
	if(this->egdcap_ == nullptr) {
		std::cerr<<"[Error] - Capabilities are not allocated"<<std::endl;
		return false;
	}
	this->frames_ = (size_t)((float)this->egdcap_->sampling_rate/hz);
	return true;
}

bool EGDDevice::setup_egd_labels(void) {
	
	unsigned int i, igrp;
	int type;
	
	if(this->grp_ == nullptr) {
		std::cerr<<"[Error] - Groups are not allocated"<<std::endl;
		return false;
	}
	
	if(this->labels_ == nullptr) {
		std::cerr<<"[Error] - Labels are not allocated"<<std::endl;
		return false;
	}

	// Allocate and copy labels
	for (igrp=0; igrp<this->ngrp_-1; igrp++) {
		this->labels_[igrp] = (char**)malloc(this->grp_[igrp].nch * sizeof(char*));
		memset(this->labels_[igrp], 0, this->grp_[igrp].nch * sizeof(char*));
		type = this->grp_[igrp].sensortype;
		for (i=0; i<this->grp_[igrp].nch; i++) {
			this->labels_[igrp][i] = (char*)malloc(EGD_MAXSIZE_CHANNEL_NAME*sizeof(char));
			memset(this->labels_[igrp][i], 0, EGD_MAXSIZE_CHANNEL_NAME *sizeof(char));
			egd_channel_info(this->egddev_, type, i,
			               EGD_LABEL, this->labels_[igrp][i], EGD_EOL);
		}
	}	

	return true;
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
}

#endif
