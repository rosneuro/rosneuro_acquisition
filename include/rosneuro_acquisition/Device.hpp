#ifndef ROSNEURO_ACQUISITION_DEVICE_HPP
#define ROSNEURO_ACQUISITION_DEVICE_HPP

#include <string>
#include <vector>

#include "rosneuro_data/NeuroData.hpp"

namespace rosneuro {

/*! \brief      Structure containing the main device information
 */
struct DeviceInfo {
	std::string		model;
	std::string  	id;
};

/*! \brief      Device class
 * 
 * This class implements a general virtual device to interface with commercial amplifiers or pre-recorded files.
 * It provides some public methods to allow the user to interact with the device, like opening, closing, starting and
 * stopping the device, as well as getting data from the device. The class provides also protected and public variables
 * to store the main device information.
 * 
 * \sa EGDDevice, DummyDevice
 */
class Device {
	
	public:
		/*! \brief      Constructor
		 *
		 * \param      frame  Data frame
		 * 
		 */
		Device(NeuroFrame* frame);

		/*! \brief      Destructor
		 */
		virtual ~Device(void);

		/*! \brief      Set up the device
		 *
		 * \param      framerate  The framerate of data acquisition [Hz]
		 *
		 * \return     True if the set up is correctly performed, false otherwise
		 */
		virtual bool   Setup(float framerate) = 0;

		/*! \brief      Open the device
		 *
		 * \param      devname     Name of the device
		 * \param      samplerate  Samplerate of the device [Hz]
		 *
		 * \return     True if the device is correctly opened, false otherwise
		 */
		virtual bool   Open(const std::string& devname, int samplerate) = 0;

		/*! \brief      Close the device
		 *
		 * \return     True if the device is correctly closed, false otherwise
		 */
		virtual bool   Close(void)	= 0;

		/*! \brief      Start the device
		 *
		 * \return     True if the device is correctly started, false otherwise
		 */
		virtual bool   Start(void)	= 0;

		/*! \brief      Stop the device
		 *
		 * \return     True if the device is correctly stopped, false otherwise
		 */
		virtual bool   Stop(void)	= 0;

		/*! \brief      Get data from the device
		 *
		 * \return     Size of the data
		 */
		virtual size_t Get(void)	= 0;

		/*! \brief      Get available data from the device
		 *
		 * \return     Size of the data
		 */
		virtual size_t GetAvailable(void) = 0;


		/*! \brief      Gets the name of the device
		 *
		 * \return     The device's name.
		 */
		virtual std::string GetName(void);

		/*! \brief      Print the device's name
		 */
		virtual void Who(void);

		/*! \brief      Print the device's name, model and identifier
		 */
		virtual void Dump(void);

	protected:
		std::string	name_;
		NeuroFrame* frame_;

	public:
		DeviceInfo	devinfo;

};


}


#endif
