#ifndef ROSNEURO_ACQUISITION_TOOLS_CPP
#define ROSNEURO_ACQUISITION_TOOLS_CPP

#include "rosneuro_acquisition/AcquisitionTools.hpp"

namespace rosneuro {

bool AcquisitionTools::ToMessage(const DeviceData* data, rosneuro_msgs::NeuroData& msg) {

	/*
	// Clearing the message before filling it
	AcquisitionTools::ClearMessage(msg);

	msg.nsamples	  = data->sframe;
	msg.eeg_nchannels = data->neeg;
	msg.exg_nchannels = data->nexg;
	msg.tri_nchannels = data->ntri;
	msg.eeg_labels.assign(&(data->leeg[0]), &(data->leeg[data->neeg]));
	msg.exg_labels.assign(&(data->lexg[0]), &(data->lexg[data->nexg]));

	float* eeg = (float*)data->eeg;
	float* exg = (float*)data->exg;
	float* tri = (float*)data->tri;
	
	msg.eeg_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.eeg_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.eeg_data.layout.dim[0].label	= "channel";
	msg.eeg_data.layout.dim[0].size		= data->neeg;
	msg.eeg_data.layout.dim[0].stride	= data->neeg*data->sframe;
	msg.eeg_data.layout.dim[1].label	= "sample";
	msg.eeg_data.layout.dim[1].size		= data->sframe;
	msg.eeg_data.layout.dim[1].stride	= data->sframe;
	msg.eeg_data.data.assign(&(eeg[0]), &(eeg[data->neeg*data->sframe]));
	
	msg.exg_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.exg_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.exg_data.layout.dim[0].label	= "channel";
	msg.exg_data.layout.dim[0].size		= data->nexg;
	msg.exg_data.layout.dim[0].stride	= data->nexg*data->sframe;
	msg.exg_data.layout.dim[1].label	= "sample";
	msg.exg_data.layout.dim[1].size		= data->sframe;
	msg.exg_data.layout.dim[1].stride	= data->sframe;
	msg.exg_data.data.assign(&(exg[0]), &(exg[data->nexg*data->sframe]));
	
	msg.tri_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.tri_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.tri_data.layout.dim[0].label	= "channel";
	msg.tri_data.layout.dim[0].size		= data->ntri;
	msg.tri_data.layout.dim[0].stride	= data->ntri*data->sframe;
	msg.tri_data.layout.dim[1].label	= "sample";
	msg.tri_data.layout.dim[1].size		= data->sframe;
	msg.tri_data.layout.dim[1].stride	= data->sframe;
	msg.tri_data.data.assign(&(tri[0]), &(tri[data->ntri*data->sframe]));

	for(auto i = msg.eeg_labels.begin(); i<msg.eeg_labels.end(); ++i)
		std::cout<<(*i)<<std::endl;
	
	for(auto i = msg.exg_labels.begin(); i<msg.exg_labels.end(); ++i)
		std::cout<<(*i)<<std::endl;
*/
	return true;
}


void AcquisitionTools::ClearMessage(rosneuro_msgs::NeuroData& msg) {

	/*
	// Clearing general information
	msg.info.sampling_rate	= 0;
	msg.info.nsamples		= 0;
	msg.info.neeg			= 0;
	msg.info.nexg			= 0;
	msg.info.ntri			= 0;

	// Clearing multilayer message
	msg.eeg_data.layout.dim.clear();
	msg.exg_data.layout.dim.clear();
	msg.tri_data.layout.dim.clear();

	msg.eeg_data.layout.data_offset = 0;
	msg.exg_data.layout.data_offset = 0;
	msg.tri_data.layout.data_offset = 0;
	
	msg.eeg_data.data.clear();
	msg.exg_data.data.clear();
	msg.tri_data.data.clear();

	// Clearing label vectors
	msg.eeg_labels.clear();
	msg.exg_labels.clear();
	*/
}

void AcquisitionTools::ClearInfoMessage(rosneuro_msgs::DeviceInfo& info) {

	info.sampling_rate	= 0;
	info.nsamples		= 0;
	info.neeg			= 0;
	info.nexg			= 0;
	info.ntri			= 0;

	info.model.clear();
	info.id.clear();
	info.prefiltering.clear();
	info.leeg.clear();
	info.lexg.clear();
	info.ltri.clear();
}

//void AcquisitionTools::InitMessage(const DeviceData


}

#endif
