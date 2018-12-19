#ifndef ROSNEURO_ACQUISITION_TOOLS_HPP
#define ROSNEURO_ACQUISITION_TOOLS_HPP

#include <ros/ros.h>
#include "rosneuro_acquisition/NeuroData.hpp"
#include "rosneuro_acquisition/Device.hpp"
#include "rosneuro_msgs/NeuroData.h"
#include "rosneuro_msgs/DeviceInfo.h"

namespace rosneuro {

class AcquisitionTools {

	public:
		static bool ToMessage(const NeuroData* data, rosneuro_msgs::NeuroData& msg);
		static bool ConfigureMessage(const DeviceCap* cap, rosneuro_msgs::NeuroData& msg);
		static bool ConfigureMessage(const NeuroData* data, rosneuro_msgs::NeuroData& msg);

		static void ClearDataMessage(rosneuro_msgs::NeuroData& msg);
		//static void ClearInfoMessage(rosneuro_msgs::DeviceInfo& info);

	private:
		AcquisitionTools(void) {};

};

}

#endif
