#ifndef ROSNEURO_ACQUISITION_HPP
#define ROSNEURO_ACQUISITION_HPP

#include <ros/ros.h>
#include "rosneuro_acquisition/DeviceFactory.hpp"
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

	private:
		ros::NodeHandle	nh_;
		ros::NodeHandle	p_nh_;
		ros::Publisher	pub_;
		std::string		topic_;

		DeviceData* 	data_;
		DeviceFactory	factory_;
		std::unique_ptr<Device>	dev_;

		std::string		devname_;
		float			fs_;
		bool			reopen_;
		rosneuro_msgs::NeuroData msg_;



};

}

#endif
