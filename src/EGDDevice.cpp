#ifndef ROSNEURO_ACQUISITION_EGDDEVICE_CPP
#define ROSNEURO_ACQUISITION_EGDDEVICE_CPP

#include "rosneuro_acquisition/EGDDevice.hpp"

namespace rosneuro {
	namespace acquisition {

EGDDevice::EGDDevice(void) {
	memset(this->grp_, 0, 3*sizeof(struct grpconf));
	memset(this->strides_, 0, 3*sizeof(size_t));
	this->eeg_	  = nullptr;
	this->exg_	  = nullptr;
	this->tri_    = nullptr;
	this->frames_ = 0;

	this->egdcap_ = (EGDCapabilities*)this->devcap_;
	this->devname_ = "eegdev";
}

EGDDevice::~EGDDevice(void) {
	if(this->eeg_ != nullptr)
		free(this->eeg_);
	if(this->exg_ != nullptr)
		free(this->exg_);
	if(this->tri_ != nullptr)
		free(this->tri_);

	// Free labels
	unsigned int igrp, i;
	for (igrp=0; igrp<2; igrp++) {
		if (this->labels_[igrp] == nullptr)
			continue;
		for (i=0; i<this->grp_[igrp].nch; i++) 
			free(this->labels_[igrp][i]);

		free(this->labels_[igrp]);
		this->labels_[igrp] = nullptr;
	}
}

void EGDDevice::InitCapabilities(void) {
	// Note by L.Tonin  <luca.tonin@epfl.ch> on 06/12/18 13:48:52	
	// TODO: To handle return error from egd_get functions
	
	egd_get_cap(this->egddev_, EGD_CAP_FS, &this->egdcap_->sampling_rate);
	egd_get_cap(this->egddev_, EGD_CAP_DEVTYPE, &this->egdcap_->model);
	egd_get_cap(this->egddev_, EGD_CAP_DEVID, &this->egdcap_->id);
	this->egdcap_->eeg_nmax = egd_get_numch(this->egddev_, EGD_EEG);
	this->egdcap_->trigger_nmax = egd_get_numch(this->egddev_, EGD_TRIGGER);
	this->egdcap_->sensor_nmax = egd_get_numch(this->egddev_, EGD_SENSOR);
	egd_channel_info(this->egddev_, EGD_EEG, 0, EGD_PREFILTERING, this->egdcap_->prefiltering, EGD_EOL);
}

void EGDDevice::InitBuffers(void) {
	
	// Setup the strides so that we get packed data into the buffers
	this->strides_[0] = this->grp_[0].nch * this->SizeEGD(EGD_FLOAT);
	this->strides_[1] = this->grp_[1].nch * this->SizeEGD(EGD_FLOAT);
	this->strides_[2] = this->grp_[2].nch * this->SizeEGD(EGD_INT32);

	// Compute sizes so not to call malloc if size == 0
	size_t seeg = this->strides_[0]*this->frames_;
	size_t sexg = this->strides_[1]*this->frames_;
	size_t stri = this->strides_[2]*this->frames_;

	this->eeg_ = seeg ? (void*)malloc(seeg) : nullptr;
	this->exg_ = sexg ? (void*)malloc(sexg) : nullptr;
	this->tri_ = stri ? (void*)malloc(stri) : nullptr;
}

void EGDDevice::InitGroups(void) {
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
}

void EGDDevice::InitFrame(float hz) {
	this->frames_ = (size_t)((float)this->egdcap_->sampling_rate/hz);
}

bool EGDDevice::Setup(float hz) {

	// Note by L.Tonin  <luca.tonin@epfl.ch> on 06/12/18 13:57:10
	// TODO: Handle setup error 

	this->InitCapabilities();
	this->InitFrame(hz);
	this->InitGroups();
	this->InitBuffers();

	if(egd_acq_setup(this->egddev_, 3, this->strides_, 3, this->grp_) == -1) {
		std::cerr<<"Cannot setup the device: " << std::strerror(errno)<<std::endl;
		return false;
	}
	
	// Setup channel labels
	unsigned int i, igrp;
	int type;

	// Allocate and copy labels
	for (igrp=0; igrp<3; igrp++) {
		this->labels_[igrp] = (char**)malloc(this->grp_[igrp].nch * sizeof(char*));
		type = this->grp_[igrp].sensortype;
		for (i=0; i<this->grp_[igrp].nch; i++) {
			this->labels_[igrp][i] = (char*)malloc(32*sizeof(char));
			egd_channel_info(this->egddev_, type, i,
			               EGD_LABEL, this->labels_[igrp][i], EGD_EOL);
		}
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
	
	this->devname_ = devnamearg;

	this->egddev_ = egd_open(this->devname_.c_str());
	if(this->egddev_ == nullptr) {
		std::cerr<<"[Error] - Cannot open the device '"<< this->GetName() <<"': "
				 << std::strerror(errno) <<std::endl;
		return false;
	}

	if(this->egdcap_ != nullptr)
		delete this->egdcap_;

	this->devcap_ = new DevCapabilities;
	this->egdcap_ = (EGDCapabilities*)this->devcap_;

	return true;
}

bool EGDDevice::Close(void) {
	if(egd_close(this->egddev_) == -1) {
		std::cerr<<"[Error] - Cannot close the device '" << this->GetName() <<"': "
			     << std::strerror(errno)<<std::endl;
		return false;
	}
	
	if(this->egdcap_ != nullptr)
		delete this->egdcap_;

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

size_t EGDDevice::SizeEGD(int egdtype) {
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

size_t EGDDevice::GetData(void) {
	return egd_get_data(this->egddev_, this->frames_, 
			this->eeg_, this->exg_, this->tri_);
}
		
size_t EGDDevice::GetAvailable(void) {
	return egd_get_available(this->egddev_);
}

const char* EGDDevice::GetPrefiltering(void) {
	return this->egdcap_->prefiltering;
}

const char*** EGDDevice::GetLabels(void) {
	return (const char***)this->labels_;
}

	}
}

#endif
