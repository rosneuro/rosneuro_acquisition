#ifndef ROSNEURO_ACQUISITION_EGDDEVICE_CPP
#define ROSNEURO_ACQUISITION_EGDDEVICE_CPP

#include "rosneuro_acquisition/EGDDevice.hpp"

namespace rosneuro {

EGDDevice::EGDDevice(void) {
	
	this->data_ = new DeviceData;
	this->name_ = "egddev";
	this->ngrp_ = EGD_DEFAULT_GROUP_NUMBER;
	this->init_egd_capabilities();
	this->init_egd_groups();
	this->init_egd_strides();
	this->init_egd_data();

}

EGDDevice::~EGDDevice(void) {
	this->destroy_egd_data();
	this->destroy_egd_cababilities();
	this->destroy_egd_strides();
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

size_t EGDDevice::GetData(DeviceData* data) {
	return egd_get_data(this->egddev_, this->data_->sframe, 
			this->data_->eeg, this->data_->exg, this->data_->tri);
	data = this->data_;
}
		
size_t EGDDevice::GetAvailable(void) {
	return egd_get_available(this->egddev_);
}

const std::string EGDDevice::GetPrefiltering(void) {
	return this->egdcap_->prefiltering;
}


void EGDDevice::Dump(void) {
	printf("[Dump] EGDDevice info:\n");
	printf(" + Capabilities:\n");
	printf(" |- Device:       %s\n",	this->egdcap_->model.c_str());
	printf(" |- Id:           %s\n",	this->egdcap_->id.c_str());
	printf(" |- Sf:           %d Hz\n", this->egdcap_->sampling_rate);
	printf(" |- Channels:     %d\n",	this->data_->neeg);
	printf(" |- Sensors:      %d\n",	this->data_->nexg);
	printf(" |- Triggers:     %d\n",	this->data_->ntri);
	printf(" |- Prefiltering: %s\n",	this->egdcap_->prefiltering.c_str());
	printf(" |- EEG labels: ");
	for(auto i=0; i<this->grp_[0].nch; i++)
		printf("%s ", this->data_->leeg[i]);
	printf("\n |- Sensors labels: ");
	for(auto i=0; i<this->grp_[1].nch; i++)
		printf("%s ", this->data_->lexg[i]);
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


void EGDDevice::init_egd_strides(void) {
	this->strides_ = (size_t*)malloc(this->ngrp_ * sizeof(size_t));
	memset(this->strides_, 0, this->ngrp_ *sizeof(size_t));
}

void EGDDevice::init_egd_data(void) {
	this->data_->neeg	= 0;
	this->data_->nexg	= 0;
	this->data_->ntri	= 0;
	this->data_->seeg	= 0;
	this->data_->sexg	= 0;
	this->data_->stri	= 0;
	this->data_->eeg	= nullptr;
	this->data_->exg  	= nullptr;
	this->data_->tri	= nullptr;
	this->data_->sframe	= 0;
	this->data_->leeg	= nullptr;
	this->data_->lexg 	= nullptr;
}

void EGDDevice::destroy_egd_data(void) {
	if(this->data_->eeg != nullptr)
		free(this->data_->eeg);
	if(this->data_->exg != nullptr)
		free(this->data_->exg);
	if(this->data_->tri != nullptr)
		free(this->data_->tri);

	this->destroy_egd_labels();
	
	this->data_->eeg    = nullptr;
	this->data_->exg    = nullptr;
	this->data_->tri    = nullptr;
	this->data_->sframe = 0;
	

	delete this->data_;
}

void EGDDevice::destroy_egd_labels(void) {

	// destroy eeg labels
	for (auto i=0; i<this->grp_[0].nch; i++) 
		free(this->data_->leeg[i]);
	free(this->data_->leeg);
	this->data_->leeg = nullptr;
	
	// destroy exg labels
	for (auto i=0; i<this->grp_[1].nch; i++) 
		free(this->data_->lexg[i]);
	free(this->data_->lexg);
	this->data_->lexg = nullptr;
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
	unsigned int nexg	= 0;
	unsigned int ntri	= 0;
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
	
	// Getting number EXG
	if( (nexg = egd_get_numch(this->egddev_, EGD_SENSOR)) == -1) {
		std::cerr<<"[Error] - Cannot get number SENSOR channels: "<<strerror(errno)<<std::endl;
		return false;
	}

	// Getting number TRIGGER
	if( (ntri = egd_get_numch(this->egddev_, EGD_TRIGGER)) == -1) {
		std::cerr<<"[Error] - Cannot get number TRIGGER channels: "<<strerror(errno)<<std::endl;
		return false;
	}

	// Getting prefiltering
	if( egd_channel_info(this->egddev_, EGD_EEG, 0, EGD_PREFILTERING, &prefiltering, EGD_EOL) == -1) {
		std::cerr<<"[Error] - Cannot get prefiltering: "<<strerror(errno)<<std::endl;
		return false;
	}

	// Populating device capabilities and data structure
	this->egdcap_->sampling_rate = fs;
	this->egdcap_->model		 = std::string(model);
	this->egdcap_->id			 = std::string(id);
	this->egdcap_->prefiltering  = std::string(prefiltering);

	this->data_->neeg = neeg;
	this->data_->nexg = nexg;
	this->data_->ntri = ntri;

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
	this->data_->seeg = this->strides_[0]*this->data_->sframe;
	this->data_->sexg = this->strides_[1]*this->data_->sframe;
	this->data_->stri = this->strides_[2]*this->data_->sframe;

	this->data_->eeg = this->data_->seeg ? (void*)malloc(this->data_->seeg) : nullptr;
	this->data_->exg = this->data_->sexg ? (void*)malloc(this->data_->sexg) : nullptr;
	this->data_->tri = this->data_->stri ? (void*)malloc(this->data_->stri) : nullptr;

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
	this->grp_[0].nch		 = this->data_->neeg;
	
	this->grp_[1].sensortype = EGD_SENSOR;
	this->grp_[1].index		 = 0; 
	this->grp_[1].iarray	 = 1; 
	this->grp_[1].datatype	 = EGD_FLOAT;
	this->grp_[1].arr_offset = 0;
	this->grp_[1].nch	     = this->data_->nexg;
	
	this->grp_[2].sensortype = EGD_TRIGGER;
	this->grp_[2].index		 = 0; 
	this->grp_[2].iarray	 = 2;
	this->grp_[2].datatype	 = EGD_INT32;
	this->grp_[2].arr_offset = 0;
	this->grp_[2].nch		 = this->data_->ntri;

	return true;
}

bool EGDDevice::setup_egd_frame(float hz) {
	if(this->egdcap_ == nullptr) {
		std::cerr<<"[Error] - Capabilities are not allocated"<<std::endl;
		return false;
	}
	this->data_->sframe = (size_t)((float)this->egdcap_->sampling_rate/hz);
	return true;
}

bool EGDDevice::setup_egd_labels(void) {
	
	int type;
	
	if(this->grp_ == nullptr) {
		std::cerr<<"[Error] - Groups are not allocated"<<std::endl;
		return false;
	}
	
	// Allocate and copy labels for eeg
	this->data_->leeg = (char**)malloc(this->grp_[0].nch * sizeof(char*));
	memset(this->data_->leeg, 0, this->grp_[0].nch * sizeof(char*));
	type = this->grp_[0].sensortype;
	for (auto i=0; i<this->grp_[0].nch; i++) {
		this->data_->leeg[i] = (char*)malloc(EGD_MAXSIZE_CHANNEL_NAME*sizeof(char));
		memset(this->data_->leeg[i], 0, EGD_MAXSIZE_CHANNEL_NAME *sizeof(char));
		egd_channel_info(this->egddev_, type, i, EGD_LABEL, this->data_->leeg[i], EGD_EOL);
	}
	
	// Allocate and copy labels for exg
	this->data_->lexg = (char**)malloc(this->grp_[1].nch * sizeof(char*));
	memset(this->data_->lexg, 0, this->grp_[1].nch * sizeof(char*));
	type = this->grp_[1].sensortype;
	for (auto i=0; i<this->grp_[1].nch; i++) {
		this->data_->lexg[i] = (char*)malloc(EGD_MAXSIZE_CHANNEL_NAME*sizeof(char));
		memset(this->data_->lexg[i], 0, EGD_MAXSIZE_CHANNEL_NAME *sizeof(char));
		egd_channel_info(this->egddev_, type, i, EGD_LABEL, this->data_->lexg[i], EGD_EOL);
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

#endif
