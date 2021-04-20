#include <ros/ros.h>
#include "rosneuro_acquisition/Acquisition.hpp"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "acquisition");

	rosneuro::Acquisition acquisition;

	if(acquisition.Run() == false)
		ROS_ERROR("Acquisition interrupted while running");

	ros::shutdown();
	return 0;
}
