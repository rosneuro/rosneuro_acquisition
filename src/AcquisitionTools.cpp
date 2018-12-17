#ifndef ROSNEURO_ACQUISITION_TOOLS_CPP
#define ROSNEURO_ACQUISITION_TOOLS_CPP

#include "rosneuro_acquisition/AcquisitionTools.hpp"

namespace rosneuro {

bool AcquisitionTools::ToMessage(const DeviceData* data, rosneuro_msgs::NeuroData& msg) {

	// Clearing the message before filling it
	AcquisitionTools::ClearDataMessage(msg);

	float* eeg = (float*)data->eeg;
	float* exg = (float*)data->exg;
	float* tri = (float*)data->tri;
	
	msg.eeg.data.assign(&(eeg[0]), &(eeg[msg.info.neeg*msg.info.nsamples]));
	msg.exg.data.assign(&(exg[0]), &(exg[msg.info.nexg*msg.info.nsamples]));
	msg.tri.data.assign(&(tri[0]), &(tri[msg.info.ntri*msg.info.nsamples]));

	msg.header.stamp = ros::Time::now();

	return true;
}


void AcquisitionTools::ClearDataMessage(rosneuro_msgs::NeuroData& msg) {
	
	msg.eeg.data.clear();
	msg.exg.data.clear();
	msg.tri.data.clear();

}

bool AcquisitionTools::SetMessage(const DeviceCapabilities* cap, rosneuro_msgs::NeuroData& msg) {

	if(cap == nullptr)
		return false;

	// Set the header
	msg.header.frame_id		= "0";

	// Set the DeviceInfo
	msg.info.model			= cap->model;
	msg.info.id				= cap->id;
	msg.info.prefiltering	= cap->prefiltering;
	msg.info.sampling_rate	= cap->sampling_rate;
	msg.info.neeg			= cap->neeg;
	msg.info.nexg			= cap->nexg;
	msg.info.ntri			= cap->ntri;
	msg.info.nsamples		= cap->nsamples;
	msg.info.leeg			= cap->leeg;
	msg.info.lexg			= cap->lexg;
	msg.info.ltri			= cap->ltri;

	// Set the data array
	msg.eeg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.eeg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.eeg.layout.dim[0].label		= "channel";
	msg.eeg.layout.dim[0].size		= cap->neeg;
	msg.eeg.layout.dim[0].stride	= cap->neeg*cap->nsamples;
	msg.eeg.layout.dim[1].label		= "sample";
	msg.eeg.layout.dim[1].size		= cap->nsamples;
	msg.eeg.layout.dim[1].stride	= cap->nsamples;
	msg.eeg.layout.data_offset		= 0;
	
	msg.exg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.exg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.exg.layout.dim[0].label		= "channel";
	msg.exg.layout.dim[0].size		= cap->nexg;
	msg.exg.layout.dim[0].stride	= cap->nexg*cap->nsamples;
	msg.exg.layout.dim[1].label		= "sample";
	msg.exg.layout.dim[1].size		= cap->nsamples;
	msg.exg.layout.dim[1].stride	= cap->nsamples;
	msg.exg.layout.data_offset		= 0;
	
	msg.tri.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.tri.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.tri.layout.dim[0].label		= "channel";
	msg.tri.layout.dim[0].size		= cap->ntri;
	msg.tri.layout.dim[0].stride	= cap->ntri*cap->nsamples;
	msg.tri.layout.dim[1].label		= "sample";
	msg.tri.layout.dim[1].size		= cap->nsamples;
	msg.tri.layout.dim[1].stride	= cap->nsamples;
	msg.tri.layout.data_offset		= 0;

	return true;
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



}

#endif
