#ifndef ROSNEURO_ACQUISITION_CONVERTER_HPP
#define ROSNEURO_ACQUISITION_CONVERTER_HPP

#include <ros/ros.h>
#include "rosneuro_acquisition/Device.hpp"
#include "rosneuro_msgs/NeuroData.h"
#include "rosneuro_msgs/DeviceInfo.h"

namespace rosneuro {

class AcquisitionConverter {

	public:
		static bool ToMessage(const DeviceData* data, rosneuro_msgs::NeuroData& msg);
		static void ClearMessage(rosneuro_msgs::NeuroData& msg);
		static void ClearInfoMessage(rosneuro_msgs::DeviceInfo& info);

	private:
		AcquisitionConverter(void) {};

};

}

#endif
