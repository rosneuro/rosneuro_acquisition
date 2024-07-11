#ifndef ROSNEURO_ACQUISITION_CPP
#define ROSNEURO_ACQUISITION_CPP

#include "Acquisition.hpp"

namespace rosneuro {

Acquisition::Acquisition(void) : p_nh_("~") {
	this->topic_	 = "/neurodata"; 
	this->autostart_ = false;
	this->state_	 = Acquisition::IS_IDLE;
	this->loader_.reset(new pluginlib::ClassLoader<Device>("rosneuro_acquisition", "rosneuro::Device"));
	this->neuroseq_  = 0;
}

Acquisition::~Acquisition(void) {
	this->dev_->Close();
	this->dev_.reset();
	this->loader_.reset();
}

bool Acquisition::configure(void) {
	if(!this->setParams()) return false;

	if(!this->configureDevice()) return false;

    this->advertise();

	return true;
}

bool Acquisition::setParams(void) {
    if(!ros::param::get("~plugin", this->plugin_)) {
        ROS_ERROR("Missing 'plugin' in the server. 'plugin' is a mandatory parameter");
        return false;
    }

    if(!ros::param::get("~framerate", this->framerate_)) {
        ROS_ERROR("Missing 'framerate' in the server. 'framerate' is a mandatory parameter");
        return false;
    }

    ros::param::param("~reopen", this->reopen_, true);
    ros::param::param("~autostart", this->autostart_, true);

    return true;
}

bool Acquisition::configureDevice(void) {
    try {
        this->dev_ = this->loader_->createInstance(this->plugin_);
    } catch (pluginlib::PluginlibException& ex) {
        ROS_ERROR("'%s' plugin failed to load: %s", this->plugin_.c_str(), ex.what());
        return false;
    }

    this->devname_ = this->dev_->GetName();

    if(!this->dev_->Configure(&this->frame_, this->framerate_)) {
        ROS_ERROR("Cannot configure the device");
        return false;
    }

    ROS_INFO("Acquisition correctly created the device: %s", this->devname_.c_str());
    return true;
}

void Acquisition::advertise(void) {
    this->pub_ = this->p_nh_.advertise<rosneuro_msgs::NeuroFrame>(this->topic_, 1);
    this->srv_start_ = this->p_nh_.advertiseService("start", &Acquisition::on_request_start, this);
    this->srv_stop_  = this->p_nh_.advertiseService("stop",  &Acquisition::on_request_stop, this);
    this->srv_quit_  = this->p_nh_.advertiseService("quit",  &Acquisition::on_request_quit, this);
    this->srv_info_  = this->p_nh_.advertiseService("get_info",  &Acquisition::on_request_info, this);
}

bool Acquisition::Run(void) {
	if(!this->configure()) {
		ROS_ERROR("Cannot configure the acquisition");
		return false;
	}
	ROS_INFO("Acquisition correctly configured");
	
	if(!this->dev_->Open()) {
		ROS_ERROR("Cannot open the device");
		return false;
	}

	if(!this->dev_->Setup()) {
		ROS_ERROR("Cannot setup the device");
		return false;
	}

	if(!NeuroDataTools::ConfigureNeuroMessage(this->frame_, this->msg_)) {
		ROS_WARN("Cannot configure NeuroFrame message");
		return false;
	}
	ROS_INFO("NeuroFrame message correctly configured");

	this->frame_.eeg.dump();
	this->frame_.exg.dump();
	this->frame_.tri.dump();

	this->startAcquisitionLoop();

	return true;
}

void Acquisition::startAcquisitionLoop(void){
    bool quit = false;
    ROS_INFO("Acquisition started");
    while(this->nh_.ok() && !quit) {
        ros::spinOnce();
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
}

unsigned int Acquisition::on_device_idle(void) {
	if(!this->autostart_) {
		ROS_WARN_ONCE("'%s' device idle. Waiting for start", this->devname_.c_str());
		return Acquisition::IS_IDLE;
	}

	if(!this->dev_->Start()) {
		ROS_ERROR("Cannot start the '%s' device", this->devname_.c_str());
		return Acquisition::IS_QUIT;
	}
	ROS_INFO("'%s' device correctly started", this->devname_.c_str());
	return Acquisition::IS_STARTED;
}

unsigned int Acquisition::on_device_started(void) {
	this->msg_.header.stamp = ros::Time::now();
	
	if(this->dev_->Get() == (size_t)-1) {
		return Acquisition::IS_DOWN;
	} 
		
	if(NeuroDataTools::FromNeuroFrame(this->frame_, this->msg_)) {
		this->neuroseq_++;
		this->msg_.neuroheader.seq = this->neuroseq_;
		this->pub_.publish(this->msg_);
	}
		
	return Acquisition::IS_STARTED;
}

unsigned int Acquisition::on_device_stopped(void) {
	return Acquisition::IS_STOPPED;
}

unsigned int Acquisition::on_device_down(void) {

	ROS_WARN("'%s' device is down", this->devname_.c_str());

	if(!this->reopen_) {
		return Acquisition::IS_QUIT;
	}

	this->dev_->Close();

	if(!this->dev_->Open()) {
		return Acquisition::IS_QUIT;
	}

	if(!this->dev_->Setup()) {
		return Acquisition::IS_QUIT;
	}
	
	if(!this->dev_->Start()) {
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
	
	if(!this->dev_->Start()) {
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

	if(!this->dev_->Stop()) {
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

	if(!this->dev_->Close()) {
		ROS_ERROR("Cannot close the '%s' device", this->devname_.c_str());
		this->state_ = Acquisition::IS_QUIT;
	}
	ROS_INFO("'%s' device correctly closed", this->devname_.c_str());
	this->state_ = Acquisition::IS_QUIT;

	return true;
}

bool Acquisition::on_request_info(rosneuro_msgs::GetAcquisitionInfo::Request& req,
								  rosneuro_msgs::GetAcquisitionInfo::Response& res) {
	
	NeuroDataTools::ConfigureNeuroMessage(this->frame_, res.frame);

	res.device_model = this->dev_->devinfo.model;
	res.device_id    = this->dev_->devinfo.id;

	res.result = true;
	return true;
}

}



#endif
