#ifndef ROSNEURO_ACQUISITION_CONVERTER_CPP
#define ROSNEURO_ACQUISITION_CONVERTER_CPP

#include "rosneuro_acquisition/AcquisitionConverter.hpp"

namespace rosneuro {

bool AcquisitionConverter::ToMessage(const DeviceData& data, rosneuro_acquisition_msgs::Acquisition& msg) {

	// Clearing the message before filling it
	AcquisitionConverter::ClearMessage(msg);

	msg.nsamples = data.sframe;
}

bool AcquisitionConverter::FromMessage(const rosneuro_acquisition_msgs::Acquisition& msg, DeviceData& data) {
	
	return true;
}

void AcquisitionConverter::ClearMessage(rosneuro_acquisition_msgs::Acquisition& msg) {

	// Clearing general information
	msg.sampling_rate	= 0;
	msg.nsamples		= 0;
	msg.eeg_nchannels	= 0;
	msg.exg_nchannels	= 0;
	msg.tri_nchannels	= 0;

	// Clearing multilayer message
	msg.eeg_data.clear();
	msg.exg_data.clear();
	msg.tri_data.clear();

	//msg.eeg_data.layout.data_offset = 0;
	//msg.exg_data.layout.data_offset = 0;
	//msg.tri_data.layout.data_offset = 0;
	//
	//msg.eeg_data.data.clear();
	//msg.exg_data.data.clear();
	//msg.tri_data.data.clear();

	// Clearing label vectors
	msg.eeg_labels.clear();
	msg.exg_labels.clear();
	msg.tri_labels.clear();

}


}

#endif
