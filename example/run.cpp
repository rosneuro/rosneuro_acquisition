#include <ros/ros.h>
#include "Acquisition.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "acquisition");

	rosneuro::Acquisition acquisition;

	if(!acquisition.Run())
		ROS_ERROR("Acquisition interrupted while running");

	ros::shutdown();
	return 0;
}
