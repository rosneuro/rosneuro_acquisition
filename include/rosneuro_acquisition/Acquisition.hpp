#ifndef ROSNEURO_ACQUISITION_HPP
#define ROSNEURO_ACQUISITION_HPP

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "rosneuro_acquisition/FactoryDevice.hpp"
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_data/NeuroDataTools.hpp"
#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/GetAcquisitionInfo.h"



namespace rosneuro {

class Acquisition {
	public:
		Acquisition(void);
		virtual ~Acquisition(void);

		bool configure(void);
		bool Run(void);

	public:
		enum {IS_IDLE, IS_STARTED, IS_STOPPED, IS_DOWN, IS_QUIT};

	private:
		bool on_request_start(std_srvs::Empty::Request& req,
							  std_srvs::Empty::Response& res);
		bool on_request_stop(std_srvs::Empty::Request& req,
							 std_srvs::Empty::Response& res);
		bool on_request_quit(std_srvs::Empty::Request& req,
							 std_srvs::Empty::Response& res);
		bool on_request_info(rosneuro_msgs::GetAcquisitionInfo::Request& req,
							 rosneuro_msgs::GetAcquisitionInfo::Response& res);

		unsigned int on_device_idle(void);
		unsigned int on_device_started(void);
		unsigned int on_device_stopped(void);
		unsigned int on_device_requesting(void);
		unsigned int on_device_down(void);


	private:
		ros::NodeHandle		nh_;
		ros::NodeHandle		p_nh_;
		ros::Publisher		pub_;
		ros::ServiceServer	srv_start_;
		ros::ServiceServer	srv_stop_;
		ros::ServiceServer	srv_quit_;
		ros::ServiceServer	srv_info_;
		std::string			topic_;
		unsigned int		state_;


		FactoryDevice	factory_;
		std::unique_ptr<Device>	dev_;

		std::string		devarg_;
		std::string		devname_;
		float			framerate_;
		int				samplerate_;
		bool			reopen_;
		bool			autostart_;
		
		rosneuro_msgs::NeuroFrame msg_;
		NeuroFrame	frame_;



};

}

#endif
