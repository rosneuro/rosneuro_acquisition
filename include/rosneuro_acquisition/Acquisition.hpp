#ifndef ROSNEURO_ACQUISITION_HPP
#define ROSNEURO_ACQUISITION_HPP

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "rosneuro_acquisition/DeviceFactory.hpp"
#include "rosneuro_acquisition/NeuroData.hpp"
#include "rosneuro_msgs/NeuroData.h"
#include "rosneuro_msgs/DeviceInfo.h"
#include "rosneuro_acquisition/AcquisitionTools.hpp"



namespace rosneuro {

class Acquisition {
	public:
		Acquisition(void);
		virtual ~Acquisition(void);


		bool configure(void);

		bool Run(void);

		bool IsRunning(void);

		bool Start(void);
		bool Stop(void);

	private:
		bool on_acquisition_start(std_srvs::Empty::Request& req,
								  std_srvs::Empty::Response& res);
		bool on_acquisition_stop(std_srvs::Empty::Request& req,
								 std_srvs::Empty::Response& res);

	private:
		ros::NodeHandle		nh_;
		ros::NodeHandle		p_nh_;
		ros::Publisher		pub_;
		ros::ServiceServer	srv_start_;
		ros::ServiceServer	srv_stop_;
		std::string			topic_;


		DeviceFactory	factory_;
		std::unique_ptr<Device>	dev_;

		std::string		devname_;
		float			fs_;
		bool			reopen_;
		bool			run_;
		bool			autostart_;
		rosneuro_msgs::NeuroData msg_;



};

}

#endif
