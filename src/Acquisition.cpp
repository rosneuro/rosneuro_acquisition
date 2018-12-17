#ifndef ROSNEURO_ACQUISITION_CPP
#define ROSNEURO_ACQUISITION_CPP

#include "rosneuro_acquisition/Acquisition.hpp"

namespace rosneuro {

Acquisition::Acquisition(void) : p_nh_("~") {
	this->data_		= nullptr;
	this->topic_	= "/neurodata"; 
}

Acquisition::~Acquisition(void) {}

bool Acquisition::configure(void) {

	unsigned int devtype = DeviceType::EGDDEV;

	this->dev_ = factory_.createDevice(devtype);


	if(ros::param::get("~devname", this->devname_) == false) {
		ROS_ERROR("Missing 'devname' in the server. 'devname' is a mandatory parameter");
		return false;
	}
	
	ros::param::param("~fs", this->fs_, 16.0f);
	ros::param::param("~reopen", this->reopen_, true);

	this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroData>(this->topic_, 1);

	return true;
}

bool Acquisition::Run(void) {

	size_t gsize = -1;
	size_t asize = -1;

	// Configure acquisition
	if(this->configure() == false) {
		ROS_ERROR("Cannot configure the acquisition");
		return false;
	}
	ROS_INFO("Acquisition correctly configured");


	// Open the device
	if(this->dev_->Open(this->devname_) == false) {
		ROS_ERROR("Cannot open the device: %s", this->devname_.c_str());
		return false;
	}
	ROS_INFO("Device correctly opened");

	// Configure device
	if(this->dev_->Setup(this->fs_) == false) {
		ROS_ERROR("Cannot setup the device");
		return false;
	}
	ROS_INFO("Device correctly configured");

	// Debug - Dump device configuration
	this->dev_->Dump();

	// Start the device
	if(this->dev_->Start() == false) {
		ROS_ERROR("Cannot start the device");
		return false;
	}
	ROS_INFO("Device started");
	
	DeviceData* tdata;
	
	ros::Rate r(60);
	while(this->nh_.ok()) {

		gsize = this->dev_->Get();
		asize = this->dev_->GetAvailable();
		if(gsize == (size_t)-1) {
			ROS_WARN("The device is down");
			if(this->reopen_ == false) {
				break;
			} else {
				this->dev_->Close();
				if(this->dev_->Open(this->devname_) == false) {
					ROS_ERROR("Cannot re-open the device: %s", this->devname_.c_str());
					return false;
				}
				ROS_INFO("Device correctly re-opened");

				// Configure device
				if(this->dev_->Setup(this->fs_) == false) {
					ROS_ERROR("Cannot re-setup the device");
					return false;
				}
				ROS_INFO("Device correctly re-configured");
				if(this->dev_->Start() == false) {
					ROS_ERROR("Cannot re-start the device");
					return false;
				}
				ROS_INFO("Device re-started");
			}
			continue;
		}


		// Publish DeviceData
		this->data_ = this->dev_->GetData();
		if(AcquisitionTools::ToMessage(this->data_, this->msg_) == true)
			this->pub_.publish(this->msg_);

		if(asize > 0)
			ROS_WARN("Running late: Get/Available=%zd/%zd", gsize, asize);

		ros::spinOnce();
		r.sleep();
	}

	return true;
}

}



#endif
