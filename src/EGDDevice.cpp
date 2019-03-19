#ifndef ROSNEURO_ACQUISITION_EGDDEVICE_CPP
#define ROSNEURO_ACQUISITION_EGDDEVICE_CPP

#include "rosneuro_acquisition/EGDDevice.hpp"

namespace rosneuro {

EGDDevice::EGDDevice(NeuroFrame* frame) : Device(frame) {
	
	this->name_ = "egddev";

	this->grp_	   = nullptr;
	this->strides_ = nullptr;

}

EGDDevice::~EGDDevice(void) {
	this->destroy_egd_structures();
}

bool EGDDevice::Setup(float framerate) {

	
	// Setup structures
	if(this->setup_dev_capabilities() == false) {
		std::cerr<<"[Error] - Cannot setup capabilities"<<std::endl; 
		return false;
	}
	
	
	if(this->setup_neuro_data(framerate) == false) {
		std::cerr<<"[Error] - Cannot setup data"<<std::endl; 
		return false;
	}
	
	if(this->setup_egd_structures() == false) {
		std::cerr<<"[Error] - Cannot setup group"<<std::endl; 
		return false;
	}

	// Setup egd device
	if(egd_acq_setup(this->egddev_, EGD_DATA_GROUPS, this->strides_, EGD_DATA_GROUPS, this->grp_) == -1) {
		std::cerr<<"Cannot setup the device: " << std::strerror(errno)<<std::endl;
		return false;
	}
	
	
	return true;
}

bool EGDDevice::Open(const std::string& devname, int samplerate) {

	std::string devnamearg;
	bool isfile = false;

	if(devname.find(".bdf") != std::string::npos) {
		devnamearg.assign("datafile|path|");
		isfile = true;
	} else if(devname.find(".gdf") != std::string::npos) {
		devnamearg.assign("datafile|path|");
		isfile = true;
	}
	devnamearg.append(devname);

	if(isfile == false && samplerate > 0) {
		if(devname.compare("gtec") == 0) {
			devnamearg += "|samplerate|" + std::to_string(samplerate);
		} else if(devname.compare("eego") == 0) {
			devnamearg += "|SR|" + std::to_string(samplerate);
		}
	}

	//this->name_ = devname;

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
	
	this->destroy_egd_structures();
	
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

	size = egd_get_data(this->egddev_, this->frame_->eeg.nsamples(), 
						(void*)this->frame_->eeg.data(), 
						(void*)this->frame_->exg.data(), 
						(void*)this->frame_->tri.data());
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
	this->devinfo.model	= "";
	this->devinfo.id	= "";
}

void EGDDevice::init_egd_structures(void) {

	// Group structure
	this->grp_ = (grpconf*)malloc(EGD_DATA_GROUPS * sizeof(grpconf));
	memset(this->grp_, 0, EGD_DATA_GROUPS*sizeof(struct grpconf));
	
	// Strides array
	this->strides_ = (size_t*)malloc(EGD_DATA_GROUPS * sizeof(size_t));
	memset(this->strides_, 0, EGD_DATA_GROUPS*sizeof(size_t));
}


bool EGDDevice::setup_dev_capabilities(void) {

	char* model			= nullptr;
	char* id			= nullptr;

	// Initialize device capabilities structure
	this->init_dev_capabilities();
	
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
	this->devinfo.model	= std::string(model);
	this->devinfo.id	= std::string(id);

	return true;
}

bool EGDDevice::setup_neuro_data(float framerate) {

	size_t ns;
	size_t neeg, nexg, ntri;
	unsigned int sampling_rate;
	NeuroDataInfo *ieeg, *iexg, *itri;

	// Getting sampling rate and number of samples
	if(egd_get_cap(this->egddev_, EGD_CAP_FS, &sampling_rate) == -1) {
		std::cerr<<"[Error] - Cannot get device sampling rate: "<<strerror(errno)<<std::endl;
		return false;
	}
	ns = (size_t)(float)sampling_rate/framerate;
	this->frame_->sr = sampling_rate;

	// Getting EEG number of channels
	if( (neeg = egd_get_numch(this->egddev_, EGD_EEG)) == -1) {
		std::cerr<<"[Error] - Cannot get number EEG channels: "<<strerror(errno)<<std::endl;
		return false;
	}
	
	// Getting EXG number of channels
	if( (nexg = egd_get_numch(this->egddev_, EGD_SENSOR)) == -1) {
		std::cerr<<"[Error] - Cannot get number EXG channels: "<<strerror(errno)<<std::endl;
		return false;
	}

	// Getting TRI number of channels
	if( (ntri = egd_get_numch(this->egddev_, EGD_TRIGGER)) == -1) {
		std::cerr<<"[Error] - Cannot get number TRIGGER channels: "<<strerror(errno)<<std::endl;
		return false;
	}

	// Setup NeuroData groups
	this->frame_->eeg.reserve(ns, neeg);
	this->frame_->exg.reserve(ns, nexg);
	this->frame_->tri.reserve(ns, ntri);

	// Fill NeuroData Info
	this->setup_neuro_info(this->frame_->eeg.info(), this->frame_->eeg.nchannels(), EGD_EEG);
	this->setup_neuro_info(this->frame_->exg.info(), this->frame_->exg.nchannels(), EGD_SENSOR);
	this->setup_neuro_info(this->frame_->tri.info(), this->frame_->tri.nchannels(), EGD_TRIGGER);


	return true;
}

void EGDDevice::setup_neuro_info(NeuroDataInfo* info, size_t nch, unsigned int index) {

	char unit[16], transducter[128], filtering[128], label[32];
	double mm[2];
	int isint = 0;

	if(info == nullptr)
		return;

	egd_channel_info(this->egddev_, index, 0, EGD_UNIT, unit, 
					EGD_TRANSDUCTER, transducter, EGD_PREFILTERING, filtering,
					EGD_MM_D, mm, EGD_ISINT, &isint, EGD_EOL);
	info->unit			= std::string(unit);
	info->transducter	= std::string(transducter);
	info->prefiltering	= std::string(filtering);
	info->minmax[0]		= mm[0];
	info->minmax[1]		= mm[1];
	info->isint			= isint;

	info->labels.clear();

	for(auto i = 0; i<nch; i++) {
		egd_channel_info(this->egddev_, index, i,
						 EGD_LABEL, label, EGD_EOL);
		info->labels.push_back(std::string(label));
	}
}


bool EGDDevice::setup_egd_structures(void) {



	// Initialize eegdev structures
	this->init_egd_structures();
	
	// Configure 
	this->grp_[0].sensortype = EGD_EEG;
	this->grp_[0].index		 = 0;
	this->grp_[0].iarray	 = 0;
	this->grp_[0].datatype	 = EGD_FLOAT;
	this->grp_[0].arr_offset = 0;
	this->grp_[0].nch		 = this->frame_->eeg.nchannels();
	
	this->grp_[1].sensortype = EGD_SENSOR;
	this->grp_[1].index		 = 0; 
	this->grp_[1].iarray	 = 1; 
	this->grp_[1].datatype	 = EGD_FLOAT;
	this->grp_[1].arr_offset = 0;
	this->grp_[1].nch		 = this->frame_->exg.nchannels();
	
	this->grp_[2].sensortype = EGD_TRIGGER;
	this->grp_[2].index		 = 0; 
	this->grp_[2].iarray	 = 2;
	this->grp_[2].datatype	 = EGD_INT32;
	this->grp_[2].arr_offset = 0;
	this->grp_[2].nch		 = this->frame_->tri.nchannels();

	this->strides_[0] = this->frame_->eeg.stride();
	this->strides_[1] = this->frame_->exg.stride();
	this->strides_[2] = this->frame_->tri.stride();

	return true;
}



void EGDDevice::destroy_egd_structures(void) {
	
	if(this->grp_ != nullptr)
		free(this->grp_);
	this->grp_ = nullptr;

	if(this->strides_ != nullptr)
		free(this->strides_);
	this->strides_ = nullptr;
}


}

#endif
