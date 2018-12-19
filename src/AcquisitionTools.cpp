#ifndef ROSNEURO_ACQUISITION_TOOLS_CPP
#define ROSNEURO_ACQUISITION_TOOLS_CPP

#include "rosneuro_acquisition/AcquisitionTools.hpp"

namespace rosneuro {

bool AcquisitionTools::ToMessage(const NeuroData* data, rosneuro_msgs::NeuroData& msg) {

	// Clearing the message before filling it
	AcquisitionTools::ClearDataMessage(msg);

	float* eeg = (float*)data->data[0];
	float* exg = (float*)data->data[1];
	int32_t* tri = (int32_t*)data->data[2];
	
	msg.eeg.data.assign(&(eeg[0]), &(eeg[msg.info_eeg.nchannels*msg.info_device.nsamples]));
	msg.exg.data.assign(&(exg[0]), &(exg[msg.info_exg.nchannels*msg.info_device.nsamples]));
	msg.tri.data.assign(&(tri[0]), &(tri[msg.info_tri.nchannels*msg.info_device.nsamples]));

	msg.header.stamp = ros::Time::now();

	return true;
}


void AcquisitionTools::ClearDataMessage(rosneuro_msgs::NeuroData& msg) {
	
	msg.eeg.data.clear();
	msg.exg.data.clear();
	msg.tri.data.clear();

}

bool AcquisitionTools::ConfigureMessage(const DeviceCap* cap, rosneuro_msgs::NeuroData& msg) {

	if(cap == nullptr)
		return false;

	// Set the header
	msg.header.frame_id		= "0";

	// Set the DeviceInfo
	msg.info_device.model			= cap->model;
	msg.info_device.id				= cap->id;
	msg.info_device.sampling_rate	= cap->sampling_rate;
	msg.info_device.nsamples		= cap->nsamples;
	
	return true;
}

bool AcquisitionTools::ConfigureMessage(const NeuroData* data, rosneuro_msgs::NeuroData& msg) {

	if(data == nullptr)
		return false;

	msg.info_eeg.unit			= data->info[0].unit;
	msg.info_eeg.transducter	= data->info[0].transducter;
	msg.info_eeg.prefiltering	= data->info[0].prefiltering;
	msg.info_eeg.isint			= data->info[0].isint;
	msg.info_eeg.nchannels		= data->info[0].nchannels;
	msg.info_eeg.labels			= data->info[0].labels;
	msg.info_eeg.minmax.push_back(data->info[0].minmax[0]);
	msg.info_eeg.minmax.push_back(data->info[0].minmax[1]);

	msg.info_exg.unit			= data->info[1].unit;
	msg.info_exg.transducter	= data->info[1].transducter;
	msg.info_exg.prefiltering	= data->info[1].prefiltering;
	msg.info_exg.isint			= data->info[1].isint;
	msg.info_exg.nchannels		= data->info[1].nchannels;
	msg.info_exg.labels			= data->info[1].labels;
	msg.info_exg.minmax.push_back(data->info[1].minmax[0]);
	msg.info_exg.minmax.push_back(data->info[1].minmax[1]);
	
	msg.info_tri.unit			= data->info[2].unit;
	msg.info_tri.transducter	= data->info[2].transducter;
	msg.info_tri.prefiltering	= data->info[2].prefiltering;
	msg.info_tri.isint			= data->info[2].isint;
	msg.info_tri.nchannels		= data->info[2].nchannels;
	msg.info_tri.labels			= data->info[2].labels;
	msg.info_tri.minmax.push_back(data->info[2].minmax[0]);
	msg.info_tri.minmax.push_back(data->info[2].minmax[1]);

	return true;
}


/*
	
	msg.info_device.neeg			= cap->neeg;
	msg.info_device.nexg			= cap->nexg;
	msg.info_device.ntri			= cap->ntri;
	msg.info_device.leeg			= cap->leeg;
	msg.info_device.lexg			= cap->lexg;
	msg.info_device.ltri			= cap->ltri;

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

*/


}

#endif
