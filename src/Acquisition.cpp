#ifndef ROSNEURO_ACQUISITION_CPP
#define ROSNEURO_ACQUISITION_CPP

#include "rosneuro_acquisition/Acquisition.hpp"

namespace rosneuro {

Acquisition::Acquisition(void) : p_nh_("~") {
	this->data_		= nullptr;
	this->topic_	= "/neurodata"; 
	this->run_      = false;
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
	ros::param::param("~autostart", this->autostart_, true);

	this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroData>(this->topic_, 1);
	this->srv_start_ = this->p_nh_.advertiseService("start", &Acquisition::on_acquisition_start, this);
	this->srv_stop_  = this->p_nh_.advertiseService("stop",  &Acquisition::on_acquisition_stop, this);


	return true;
}

bool Acquisition::Run(void) {

	size_t gsize = -1;
	size_t asize = -1;
	ros::Rate r(60);
	
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

	// Configure the NeuroData message
	if(AcquisitionTools::SetMessage(this->dev_->GetCapabilities(), this->msg_) == false) {
		ROS_ERROR("Cannot configure the NeuroData message");
		return false;
	}
	ROS_INFO("NeuroData message correctly configured");


	// Debug - Dump device configuration
	this->dev_->Dump();

	// Waiting for starting OR start
	if(this->autostart_ == false) {
		while(this->nh_.ok() && (this->IsRunning() == false)) {
			ROS_WARN_ONCE("Device idle. Waiting for start");
			ros::spinOnce();
			r.sleep();
		}
	} else {
		if(this->Start() == false)
			return false;
	}
	
	DeviceData* tdata;
	
	while(this->nh_.ok()) {
		ros::spinOnce();
		r.sleep();

		if(this->IsRunning() == false)
			continue;

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
				
				if(this->Start() == false)
					return false;
			}
			continue;
		}


		// Publish DeviceData
		this->data_ = this->dev_->GetData();
		if(AcquisitionTools::ToMessage(this->data_, this->msg_) == true)
			this->pub_.publish(this->msg_);

		if(asize > 0)
			ROS_WARN("Running late: Get/Available=%zd/%zd", gsize, asize);

	}

	return true;
}

bool Acquisition::IsRunning(void) {
	return this->run_;
}

bool Acquisition::on_acquisition_start(std_srvs::Empty::Request& req,
									   std_srvs::Empty::Response& res) {

	ROS_INFO("Requested acquisition to start");
	this->Start();

	return true;
}

bool Acquisition::on_acquisition_stop(std_srvs::Empty::Request& req,
									  std_srvs::Empty::Response& res) {

	ROS_INFO("Requested acquisition to stop");
	this->Stop();

	return true;
}

bool Acquisition::Start(void) {

	bool retcod;

	if(this->IsRunning() == true) {
		ROS_INFO("Device '%s' is already started", this->dev_->GetName().c_str());
		return true;
	}

	if(this->dev_->Start() == true) {
		ROS_INFO("Device '%s' started", this->dev_->GetName().c_str());
		this->run_ = true;
		retcod = true;
	} else {
		ROS_ERROR("Cannot start the device '%s'", this->dev_->GetName().c_str());
		this->run_ = false;
		retcod = false;
	}

	return retcod;
}

bool Acquisition::Stop(void) {
	
	bool retcod;
	
	if(this->IsRunning() == false) {
		ROS_INFO("Device '%s' is already stopped", this->dev_->GetName().c_str());
		return true;
	}

	if(this->dev_->Stop() == true) {
		ROS_INFO("Device '%s' stopped", this->dev_->GetName().c_str());
		this->run_ = false;
		retcod = true;
	} else {
		ROS_ERROR("Cannot stop the device '%s'", this->dev_->GetName().c_str());
		this->run_ = true;
		retcod = false;
	}

	return retcod;
}

}



#endif
