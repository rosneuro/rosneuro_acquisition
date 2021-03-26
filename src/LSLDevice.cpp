#ifndef ROSNEURO_ACQUISITION_LSLDEVICE_CPP
#define ROSNEURO_ACQUISITION_LSLDEVICE_CPP

#include "rosneuro_acquisition/LSLDevice.hpp"

namespace rosneuro {

LSLDevice::LSLDevice(void) : Device() {
	this->name_ = "lsldev";
	this->stream_ = nullptr;
	this->info_   = nullptr;
	
	this->stream_name_ = "unknown";
	this->stream_type_ = "unknown";
}

LSLDevice::LSLDevice(NeuroFrame* frame) : Device(frame) {
	
	this->name_ = "lsldev";
	this->stream_ = nullptr;
	this->info_   = nullptr;
	
	this->stream_name_ = "unknown";
	this->stream_type_ = "unknown";
}

LSLDevice::~LSLDevice(void) {
	this->destroy_lsl_structures();
}

bool LSLDevice::Open(const std::string& devarg, int samplerate) {

	std::vector<lsl::stream_info> results;
	std::vector<std::string> devparams;

	// Escaping devarg string
	devparams = this->escape_device_string(devarg, "\\|");
	
	if(devparams.size() != 3) {
		std::cout<<"[Error] - devarg must be in the format 'lsl|TYPE|NAME' of the stream"<<std::endl;
		return false;
	}

	// Associating type and name of the stream
	this->stream_type_ = devparams[1];
	this->stream_name_ = devparams[2];

	// Resolving the stream (with name) and checking the type
	results = lsl::resolve_stream("name", this->stream_name_, 1, 1);

	if (results.empty()) {
		std::cerr<<"[Error] - Stream with name '"<< this->stream_name_
				 <<"' not found in the current stream"<< std::endl;
		return false;
	}

	if(this->stream_type_.compare(results[0].type()) != 0) {
		std::cerr<<"[Error] - Stream with type '"<<this->stream_type_
				 <<"' not found in the current stream"<<std::endl;
		return false;
	}

	// Create stream inlet and info
	this->stream_ = new lsl::stream_inlet(results[0]);
	this->info_   = new lsl::stream_info(this->stream_->info());

	return true;
}

bool LSLDevice::Setup(float framerate) {
	size_t ns;
	size_t neeg, nexg, ntri;
	unsigned int sampling_rate;
	NeuroDataInfo *ieeg, *iexg, *itri;

	// Getting sampling rate from the stream
	sampling_rate = this->info_->nominal_srate();
	this->frame_->sr = sampling_rate;

	if(this->info_->nominal_srate() == lsl::IRREGULAR_RATE) {
		std::cerr<<"[Error] - LSL device does not support irregular rate data stream"<<std::endl;
		return false;
	}

	// Getting number of samples in the frame
	ns = (size_t)(float)sampling_rate/framerate;

	// Getting number of channels from the stream
	neeg = this->info_->channel_count();
	nexg = 0;
	ntri = 0;
	
	// Setup NeuroData groups
	this->frame_->eeg.reserve(ns, neeg);
	this->frame_->exg.reserve(ns, nexg);
	this->frame_->tri.reserve(ns, ntri);

	// Fill NeuroData Info
	this->frame_->eeg.info()->unit = "uV";
	this->frame_->eeg.info()->transducter = "n/a";
	this->frame_->eeg.info()->prefiltering = "n/a";
	this->frame_->eeg.info()->minmax[0] = std::numeric_limits<float>::lowest();
	this->frame_->eeg.info()->minmax[1] = std::numeric_limits<float>::max();
	this->frame_->eeg.info()->isint = 0;

	this->frame_->eeg.info()->labels.clear();
	for (auto i = 0; i<neeg; i++)
		this->frame_->eeg.info()->labels.push_back(std::string("eeg:"+std::to_string(i)));


	return true;

}

bool LSLDevice::Start(void) {

	try {
		this->stream_->open_stream(1);
	} catch (const std::runtime_error& e) {
		std::cerr<<"[Error] - Cannot start the device '" << this->GetName() <<"': "
				 << e.what() << std::endl;
		return false;
	}
	return true;
	
}

bool LSLDevice::Stop(void) {

	/* Not clear how the close stream works. For the time being, let's leave
	 * undefined
	 */
	std::cerr<<"[Warning] - Stop function undefined for '"
		     << this->GetName() <<"' device. The stream will not be closed."<<std::endl;
	/*
	try {
		this->stream_->close_stream();
	} catch (const std::runtime_error& e) {
		std::cerr<<"[Error] - Cannot stop the device '" << this->GetName() <<"': "
				 << e.what() << std::endl;
		return false;
	}*/
	return true;
}

bool LSLDevice::Close(void) {
	//return this->Stop();
	
	this->stream_->close_stream();
	return true;
}

size_t LSLDevice::Get(void) {
	size_t size;
	size_t ns;

	ns = this->frame_->eeg.nsamples() * this->frame_->eeg.nchannels();
	size = this->stream_->pull_chunk_multiplexed(this->frame_->eeg.data(), nullptr, ns, 0, 1);

	if (size != ns) {
		std::cerr<<"[Error] - Corrupted data reading: unexpected chunk size: "<< size << std::endl;
	}

	return size;
}

size_t LSLDevice::GetAvailable(void) {
	return this->stream_->samples_available();
}


void LSLDevice::destroy_lsl_structures(void) {

	if(this->stream_ != nullptr)
		delete this->stream_;

	this->stream_ = nullptr;

	if(this->info_ != nullptr)
		delete this->info_;

	this->info_ = nullptr;
}

std::vector<std::string> LSLDevice::escape_device_string(const std::string& devarg, const std::string& delimiter) {
	std::regex regexz(delimiter);
    std::vector<std::string> list(std::sregex_token_iterator(devarg.begin(), devarg.end(), regexz, -1),
                                  std::sregex_token_iterator());

	return list;
}

}

#endif
