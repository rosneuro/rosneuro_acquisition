#ifndef ROSNEURO_ACQUISITION_CPP
#define ROSNEURO_ACQUISITION_CPP

#include "rosneuro_acquisition/Acquisition.hpp"

namespace rosneuro {

Acquisition::Acquisition(void) : p_nh_("~") {
	this->topic_	 = "/neurodata"; 
	this->autostart_ = false;
	this->state_	 = Acquisition::IS_IDLE;
}

Acquisition::~Acquisition(void) {}

bool Acquisition::configure(void) {

	unsigned int devtype = DeviceType::EGDDEV;

	this->dev_	   = factory_.createDevice(devtype);
	this->devname_ = this->dev_->GetName();


	if(ros::param::get("~devarg", this->devarg_) == false) {
		ROS_ERROR("Missing 'devarg' in the server. 'devarg' is a mandatory parameter");
		return false;
	}
	
	ros::param::param("~fs", this->fs_, 16.0f);
	ros::param::param("~reopen", this->reopen_, true);
	ros::param::param("~autostart", this->autostart_, true);

	this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroDataSet>(this->topic_, this->fs_);
	this->srv_start_ = this->p_nh_.advertiseService("start", &Acquisition::on_request_start, this);
	this->srv_stop_  = this->p_nh_.advertiseService("stop",  &Acquisition::on_request_stop, this);
	this->srv_quit_  = this->p_nh_.advertiseService("quit",  &Acquisition::on_request_quit, this);
	this->srv_info_  = this->p_nh_.advertiseService("get_info",  &Acquisition::on_request_info, this);


	return true;
}

bool Acquisition::Run(void) {

	bool quit = false;
	ros::Rate r(60);
	
	// Configure acquisition
	if(this->configure() == false) {
		ROS_ERROR("Cannot configure the acquisition");
		return false;
	}
	ROS_INFO("Acquisition correctly configured");

	// Open the device
	if(this->dev_->Open(this->devarg_) == false) {
		ROS_ERROR("Cannot open the '%s' device with arg=%s", this->devname_.c_str(), this->devarg_.c_str());
		return false;
	}
	ROS_INFO("'%s' device correctly opened with arg=%s", this->devname_.c_str(), this->devarg_.c_str());

	// Configure device
	if(this->dev_->Setup(this->fs_) == false) {
		ROS_ERROR("Cannot setup the '%s' device", this->devname_.c_str());
		return false;
	}
	ROS_INFO("'%s' device correctly configured", this->devname_.c_str());

	// Configure the message
	if(NeuroDataTools::ConfigureMessage(this->dev_->eeg.info(), this->msg_.eeg.info) == false)
		ROS_WARN("Cannot configure eeg info because group is not set. Skipping");
	if(NeuroDataTools::ConfigureMessage(this->dev_->exg.info(), this->msg_.exg.info) == false)
		ROS_WARN("Cannot configure exg info because group is not set. Skipping");
	if(NeuroDataTools::ConfigureMessage(this->dev_->tri.info(), this->msg_.tri.info) == false)
		ROS_WARN("Cannot configure tri info because group is not set. Skipping");
		
	ROS_INFO("NeuroData message correctly configured");

	// Debug - Dump device configuration
	this->dev_->eeg.dump();
	this->dev_->exg.dump();
	this->dev_->tri.dump();

	ROS_INFO("Acquisition started");
	while(this->nh_.ok() && quit == false) {
		
		ros::spinOnce();
		r.sleep();

		switch(this->state_) {
			case Acquisition::IS_IDLE:
				this->state_ = this->on_device_idle();
				break;
			case Acquisition::IS_STARTED:
				this->state_ = this->on_device_started();
				break;
			case Acquisition::IS_STOPPED:
				this->state_ = this->on_device_stopped();
				break;
			case Acquisition::IS_DOWN:
				this->state_ = this->on_device_down();
				break;
			case Acquisition::IS_QUIT:
				quit = true;
				break;
			default:
				break;
		}

	}
	ROS_INFO("Acquisition closed");

	return true;
}

unsigned int Acquisition::on_device_idle(void) {

	if(this->autostart_ == false) {
		ROS_WARN_ONCE("'%s' device idle. Waiting for start", this->devname_.c_str());
		return Acquisition::IS_IDLE;
	}

	if(this->dev_->Start() == false) {
		ROS_ERROR("Cannot start the '%s' device", this->devname_.c_str());
		return Acquisition::IS_QUIT;
	}
	ROS_INFO("'%s' device correctly started", this->devname_.c_str());
	return Acquisition::IS_STARTED;
}

unsigned int Acquisition::on_device_started(void) {
	size_t gsize = -1;
	size_t asize = -1;

	gsize = this->dev_->Get();
	asize = this->dev_->GetAvailable();

	if(gsize == (size_t)-1) {
		return Acquisition::IS_DOWN;
	} 
		
	if( NeuroDataTools::ToMessage(this->dev_->eeg, this->msg_.eeg) == true && 
		NeuroDataTools::ToMessage(this->dev_->exg, this->msg_.exg) == true && 
		NeuroDataTools::ToMessage(this->dev_->tri, this->msg_.tri) == true) {
		this->pub_.publish(this->msg_);
	}

	if(asize > 0)
		ROS_WARN("'%s' device running late: Get/Available=%zd/%zd", this->devname_.c_str(), gsize, asize);
		
	return Acquisition::IS_STARTED;
}

unsigned int Acquisition::on_device_stopped(void) {
	return Acquisition::IS_STOPPED;
}

unsigned int Acquisition::on_device_down(void) {

	ROS_WARN("'%s' device is down", this->devname_.c_str());

	if(this->reopen_ == false) {
		return Acquisition::IS_QUIT;
	}

	// Closing the device
	this->dev_->Close();

	// Re-opening the device
	if(this->dev_->Open(this->devarg_) == false) {
		ROS_ERROR("Cannot re-open the '%s' device with arg=%s", this->devname_.c_str(), this->devarg_.c_str());
		return Acquisition::IS_QUIT;
	}
	ROS_INFO("'%s' device correctly re-opened with arg=%s", this->devname_.c_str(), this->devarg_.c_str());

	// Re-configuring device
	if(this->dev_->Setup(this->fs_) == false) {
		ROS_ERROR("Cannot re-setup the '%s' device", this->devname_.c_str());
		return Acquisition::IS_QUIT;
	}
	ROS_INFO("'%s' device correctly re-configured", this->devname_.c_str());
	
	// Re-starting the device
	if(this->dev_->Start() == false) {
		ROS_ERROR("Cannot re-start the '%s' device", this->devname_.c_str());
		return Acquisition::IS_QUIT;
	}
	ROS_INFO("'%s' device correctly re-started", this->devname_.c_str());

	return Acquisition::IS_STARTED;
}


bool Acquisition::on_request_start(std_srvs::Empty::Request& req,
								   std_srvs::Empty::Response& res) {

	ROS_WARN("Requested '%s' device to start",  this->devname_.c_str());

	if(this->state_ == Acquisition::IS_STARTED) {
		ROS_INFO("'%s' device already started", this->devname_.c_str());
		return true;
	} 
	
	if( this->dev_->Start() == false) {
		ROS_ERROR("Cannot start the '%s' device", this->devname_.c_str());
		this->state_ = Acquisition::IS_QUIT;
		return false;
	}
				
	ROS_INFO("'%s' device correctly started", this->devname_.c_str());
	this->state_ = Acquisition::IS_STARTED;

	return true;
}

bool Acquisition::on_request_stop(std_srvs::Empty::Request& req,
								  std_srvs::Empty::Response& res) {

	ROS_WARN("Requested '%s' device to stop", this->devname_.c_str());
	
	if(this->state_ == Acquisition::IS_STOPPED) {
		ROS_INFO("'%s' device already stopped", this->devname_.c_str());
		return true;
	} 

	if(this->state_ == Acquisition::IS_IDLE) {
		ROS_INFO("'%s' device is idle. device is already stopped", this->devname_.c_str());
		return true;
	}

	if( this->dev_->Stop() == false) {
		ROS_ERROR("Cannot stop the '%s' device", this->devname_.c_str());
		this->state_ = Acquisition::IS_QUIT;
		return false;
	}
	
	ROS_INFO("'%s' device correctly stopped", this->devname_.c_str());
	this->state_ = Acquisition::IS_STOPPED;

	return true;
}

bool Acquisition::on_request_quit(std_srvs::Empty::Request& req,
								  std_srvs::Empty::Response& res) {

	ROS_WARN("Requested '%s' device to quit", this->devname_.c_str());

	if(this->dev_->Close() == false) {
		ROS_ERROR("Cannot close the '%s' device", this->devname_.c_str());
		this->state_ = Acquisition::IS_QUIT;
	}
	ROS_INFO("'%s' device correctly closed", this->devname_.c_str());
	this->state_ = Acquisition::IS_QUIT;

	return true;
}

bool Acquisition::on_request_info(rosneuro_msgs::GetAcquisitionInfo::Request& req,
								  rosneuro_msgs::GetAcquisitionInfo::Response& res) {
	
	
	// Configure info messages
	NeuroDataTools::ConfigureMessage(this->dev_->eeg.info(), res.ieeg); 
	NeuroDataTools::ConfigureMessage(this->dev_->exg.info(), res.iexg); 
	NeuroDataTools::ConfigureMessage(this->dev_->tri.info(), res.itri); 
	

	res.samples      = this->dev_->eeg.nsamples();
	res.eeg_channels = this->dev_->eeg.nchannels();
	res.exg_channels = this->dev_->exg.nchannels();
	res.tri_channels = this->dev_->tri.nchannels();
	res.sampling_rate = this->dev_->GetSamplingRate();

	res.device_model = this->dev_->devinfo.model;
	res.device_id    = this->dev_->devinfo.id;

	res.result = true;
	return true;

}

}



#endif
