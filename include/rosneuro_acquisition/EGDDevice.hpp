#ifndef ROSNEURO_ACQUISITION_EGDDEVICE_HPP
#define ROSNEURO_ACQUISITION_EGDDEVICE_HPP

#include <errno.h>
#include <string.h>
#include <iostream>
#include <cstring>
#include <eegdev.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_acquisition/Device.hpp"

#define EGD_DATA_GROUPS 3

namespace rosneuro {

/*! \brief      EGD device class
 * 
 * This class describes the puglin to a device compatible with the open-source libeegdev library
 * (http://neuro.debian.net/pkgs/libeegdev-dev.html), that can be created by the FactoryDevice.
 * The class allows the user to control the device, such as starting, stopping, opening and closing
 * the device. The framerate of data acquisition can be set up as well. The class derives from the Device
 * class, and so it shares its public methods.
 * 
 * \sa FactoryDevice
 */
class EGDDevice : public Device {

	public:
		/*! \brief      Constructor
		 *
		 * \param      frame  Data frame
		 * 
		 */
		EGDDevice(NeuroFrame* frame);

		/*! \brief      Destructor
		 */
		virtual ~EGDDevice(void);

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
			
	private:
		/*! \brief      Initializes the device capabilities.
		 */
		void init_dev_capabilities(void);

		/*! \brief      Initializes the egd device structures
		 */
		void init_egd_structures(void);

		/*! \brief      Set up the device capabilities
		 *
		 * \return     True if device's info are set up, false otherwise
		 */
		bool setup_dev_capabilities(void);

		/*! \brief      Set up the neuro data format from the device
		 *
		 * \param      framerate  The framerate of data acquisition [Hz]
		 *
		 * \return     True if neuro data format is correctly set up, false otherwise
		 */
		bool setup_neuro_data(float framerate);

		/*! \brief      { function_description }
		 *
		 * \param      data   Information on the acquired data
		 * \param      nch    The number of channels
		 * \param      index  The index
		 */
		void setup_neuro_info(NeuroDataInfo* data, size_t nch, unsigned int index);

		/*! \brief      Set up the egd device structures
		 *
		 * \return     True
		 */
		bool setup_egd_structures(void);
		
		/*! \brief      Destroies the egd device structures
		 */
		void destroy_egd_structures(void);


	protected:
		struct  eegdev*	egddev_;
		struct	grpconf* grp_;
		size_t* strides_;
};

 /*! \example test_egddevice.cpp
 * Example of the use of the EDG device and how to set up the framerate.
 *
 */

}


#endif
