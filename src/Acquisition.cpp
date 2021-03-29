#ifndef ROSNEURO_ACQUISITION_CPP
#define ROSNEURO_ACQUISITION_CPP

#include "rosneuro_acquisition/Acquisition.hpp"

namespace rosneuro {

Acquisition::Acquisition(void) : p_nh_("~") {
	this->topic_	 = "/neurodata"; 
	this->autostart_ = false;
	this->state_	 = Acquisition::IS_IDLE;
	this->loader_.reset(new pluginlib::ClassLoader<Device>("rosneuro_acquisition", "rosneuro::Device"));
}

Acquisition::~Acquisition(void) {
	this->dev_.reset();
	this->loader_.reset();
}

bool Acquisition::configure(void) {

	unsigned int devtypeId;
	std::string  plugin;

	// Getting mandatory parameters from ROS
	if(ros::param::get("~plugin", this->plugin_) == false) {
		ROS_ERROR("Missing 'plugin' in the server. 'plugin' is a mandatory parameter");
		return false;
	}
	if(ros::param::get("~framerate", this->framerate_) == false) {
		ROS_ERROR("Missing 'framerate' in the server. 'framerate' is a mandatory parameter");
		return false;
	}

	// Getting optional parameters from ROS
	ros::param::param("~reopen", this->reopen_, true);
	ros::param::param("~autostart", this->autostart_, true);

	// Dynamically load the plugin
	try {
		this->dev_ = this->loader_->createInstance(this->plugin_);
	} catch (pluginlib::PluginlibException& ex) {
		ROS_ERROR("'%s' plugin failed to load: %s", this->plugin_.c_str(), ex.what());
	}
		

	this->devname_ = this->dev_->GetName();
	
	if(this->dev_->Configure(&this->frame_, this->framerate_) == false) {
		ROS_ERROR("Cannot configure the device");
		return false;
	}

	ROS_INFO("Acquisition correctly created the device: %s", this->devname_.c_str());


	
	this->pub_ = this->p_nh_.advertise<rosneuro_msgs::NeuroFrame>(this->topic_, 1);
	
	this->srv_start_ = this->p_nh_.advertiseService("start", &Acquisition::on_request_start, this);
	this->srv_stop_  = this->p_nh_.advertiseService("stop",  &Acquisition::on_request_stop, this);
	this->srv_quit_  = this->p_nh_.advertiseService("quit",  &Acquisition::on_request_quit, this);
	this->srv_info_  = this->p_nh_.advertiseService("get_info",  &Acquisition::on_request_info, this);


	return true;
}

bool Acquisition::Run(void) {

	bool quit = false;
	
	// Created by L.Tonin  <luca.tonin@epfl.ch> on 07/02/19 14:23:39
	// Removed the sleep to not delay the acquisition
	// //ros::Rate r(4096);
	
	// Configure acquisition
	if(this->configure() == false) {
		ROS_ERROR("Cannot configure the acquisition");
		return false;
	}
	ROS_INFO("Acquisition correctly configured");

	// Open the device
	if(this->dev_->Open() == false) {
		return false;
	}

	// Configure device
	if(this->dev_->Setup() == false) {
		return false;
	}

	// Store samplerate in the frame
	//this->frame_.sr = this->samplerate_;

	// Configure the message
	if(NeuroDataTools::ConfigureNeuroMessage(this->frame_, this->msg_) == false) {
		ROS_WARN("Cannot configure NeuroFrame message");
		return false;
	}
	ROS_INFO("NeuroFrame message correctly configured");

	// Debug - Dump device configuration
	this->frame_.eeg.dump();
	this->frame_.exg.dump();
	this->frame_.tri.dump();

	ROS_INFO("Acquisition started");
	while(this->nh_.ok() && quit == false) {
	
		ros::spinOnce();

		// Created by L.Tonin  <luca.tonin@epfl.ch> on 07/02/19 14:23:01	
		// Removed the sleep to not delay the acquisition
		//r.sleep();

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

	this->msg_.header.stamp = ros::Time::now();
	
	if(gsize == (size_t)-1) {
		return Acquisition::IS_DOWN;
	} 
		
	if( NeuroDataTools::FromNeuroFrame(this->frame_, this->msg_) == true ) {
		this->pub_.publish(this->msg_);
	}

	if(asize > 0)
		//ROS_WARN("'%s' device running late: Get/Available=%zd/%zd", this->devname_.c_str(), gsize, asize);
		
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
	if(this->dev_->Open() == false) {
		return Acquisition::IS_QUIT;
	}

	// Re-configuring device
	if(this->dev_->Setup() == false) {
		return Acquisition::IS_QUIT;
	}
	
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
	
	
	//// Configure info messages
	NeuroDataTools::ConfigureNeuroMessage(this->frame_, res.frame); 

	res.device_model = this->dev_->devinfo.model;
	res.device_id    = this->dev_->devinfo.id;

	res.result = true;
	return true;

}

}



#endif
