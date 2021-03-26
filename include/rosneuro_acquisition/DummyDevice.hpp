#ifndef ROSNEURO_ACQUISITION_DUMMYDEVICE_HPP
#define ROSNEURO_ACQUISITION_DUMMYDEVICE_HPP

#include <memory>
#include <errno.h>
#include <string.h>
#include "rosneuro_acquisition/Device.hpp"
#include "rosneuro_acquisition/DeviceRegistration.hpp"

// Created by L.Tonin  <luca.tonin@epfl.ch> on 06/12/18 16:22:48
// Dummy device to test the device factory

namespace rosneuro {


/*! \brief      Dummy device class
 * 
 * This class implements a dummy device to test the device factory. The class derives from the Device
 * class, and so it shares its public methods.
 * 
 * \sa FactoryDevice
 * 
 */
class DummyDevice : public Device {

	public:
		/*! \brief      Constructor
		 *
		 * \param      frame  Data frame
		 * 
		 */
		DummyDevice(NeuroFrame* frame);

		DummyDevice(void);

		/*! \brief      Destructor
		 */
		virtual ~DummyDevice(void);

		/*! \brief      Set up the device
		 *
		 * \param      framerate  The framerate of data acquisition [Hz]
		 *
		 * \return     True if the set up is correctly performed, false otherwise
		 */
		bool Setup(float framerate);

		/*! \brief      Open the device
		 *
		 * \param      devname     Name of the device
		 * \param      samplerate  Samplerate of the device [Hz]
		 *
		 * \return     True if the device is correctly opened, false otherwise
		 */
		bool Open(const std::string& devname, int samplerate);

		/*! \brief      Close the device
		 *
		 * \return     True if the device is correctly closed, false otherwise
		 */
		bool Close(void);

		/*! \brief      Start the device
		 *
		 * \return     True if the device is correctly started, false otherwise
		 */
		bool Start(void);

		/*! \brief      Stop the device
		 *
		 * \return     True if the device is correctly stopped, false otherwise
		 */
		bool Stop(void);

		/*! \brief      Get data from the device
		 *
		 * \return     Size of the data
		 */
		size_t Get(void);

		/*! \brief      Get available data from the device
		 *
		 * \return     Size of the data
		 */
		size_t GetAvailable(void);


};

DeviceRegistration<DummyDevice> _DummyDevice("DummyDevice");

}


#endif
