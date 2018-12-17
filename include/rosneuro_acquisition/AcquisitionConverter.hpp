#ifndef ROSNEURO_ACQUISITION_CONVERTER_HPP
#define ROSNEURO_ACQUISITION_CONVERTER_HPP

#include <ros/ros.h>
#include "rosneuro_acquisition/Device.hpp"
#include "rosneuro_acquisition_msgs/Acquisition.h"

namespace rosneuro {

class AcquisitionConverter {

	public:
		static bool ToMessage(const DeviceData* data, rosneuro_acquisition_msgs::Acquisition& msg);
		static bool FromMessage(const rosneuro_acquisition_msgs::Acquisition& msg, DeviceData& data);
		static void ClearMessage(rosneuro_acquisition_msgs::Acquisition& msg);

	private:
		AcquisitionConverter(void) {};

};

}

#endif
