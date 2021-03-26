#ifndef ROSNEURO_ACQUISITION_FACTORY_DEVICE_HPP
#define ROSNEURO_ACQUISITION_FACTORY_DEVICE_HPP

#include <memory>
#include "rosneuro_acquisition/Device.hpp"
#include "rosneuro_acquisition/EGDDevice.hpp"
#include "rosneuro_acquisition/LSLDevice.hpp"
#include "rosneuro_acquisition/DummyDevice.hpp"

namespace rosneuro {

/*! \brief      Enum device type
 * 
 * The enum contains the list of the devices for which the plugin is currently implemented
 * 
 */
enum DeviceType {EGDDEV, LSLDEV, DUMMYDEV};


/*! \brief      Factory device class
 * 
 * This class implements the function to create a new virtual device that allows the acquisition
 * of data frames from several commercial amplifiers by means of different plugins. To date, the provided 
 * plugin relies on the open-source libeegdev library (http://neuro.debian.net/pkgs/libeegdev-dev.html) that 
 * already supports devices of the following brands: BioSemi (Netherlands), g.Tec medical engineering GmbH (Austria),
 * NeuroSky (USA), BrainProducts GmbH (Germany), BitBrain Technologies (Spain), Wearable Sensing (USA),
 * Cognionics (USA) and ANTNeuro (Netherlands). Some devices require third-party andproperty drivers to be 
 * installed and correctly recognized by the operating system.
 * 
 * \sa EGDDevice, DummyDevice
 */
class FactoryDevice {

	public:
		/*! \brief      Creates a device
		 *
		 * \param      frame  The data frame to be acquired
		 * \param	   type   The type of the device
		 *
		 * \return     A pointer to the created device
		 */
		std::unique_ptr<Device> createDevice(NeuroFrame* frame, unsigned int type = DeviceType::EGDDEV);

};

 /*! \example test_devicefactory.cpp
 * Example of the use of the device factory with an EGD device and a dummy device.
 *
 */

}


#endif
