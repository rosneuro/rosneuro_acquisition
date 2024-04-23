#ifndef ROSNEURO_ACQUISITION_HPP
#define ROSNEURO_ACQUISITION_HPP

#include <ros/ros.h>
#include "gtest/gtest_prod.h"
#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>
#include "Device.hpp"
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_data/NeuroDataTools.hpp"
#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/GetAcquisitionInfo.h"



namespace rosneuro {

/*! \brief      Acquisition class
 * 
 * This class implements the acquisition module running in the ROS node acquisition
 * in order to allow ROS-Neuro to interface with several commercial amplifiers.
 * Furthermore, it can play pre-recorded data in the GDF and BDF formats.
 * The class works as a finite state machine to ensure a robust functioning of the system. 
 * The data acquired from the amplifiers are published in frames (a chunck of data of size samples x channels)
 * on the \neurodata topic with a frequency determined by the framerate parameter. 
 * The class allows the user to start, stop, close and ask information about the acquisition on request
 * through the proper ROS service.
 * 
 * \sa FactoryDevice
 */
class Acquisition {
	public:
		/*! \brief      Constructor
		 * 
		 * The constructor set ups the topic name \neurodata for publishing data, 
		 * the autostart to false and the state to IS_IDLE.
		 * 
		 */
		Acquisition(void);
		/*! \brief      Destructor
		 */
		virtual ~Acquisition(void);

		/*! \brief      Configure the acquisition
		 *
		 * The function stores the ROS parameters in in-member variables and sets up 
		 * the ROS services and publisher. 
		 *
		 * \return     True if the configuration is performed correctly, false otherwise
		 * 
		 */
		virtual bool configure(void);

		/*! \brief      Run the acquisition
		 *
		 * The function opens and configures the device, configures the Neuroframe message
		 * and runs the finite state machine.
		 *
		 * \return     True if the acquisition is closed on request without errors, false otherwise.
		 */
		bool Run(void);

	public:
		/*! \brief      Enum describing the possible acquisition states.
		 */
		enum {IS_IDLE, IS_STARTED, IS_STOPPED, IS_DOWN, IS_QUIT};

	private:
		/*! \brief      Called on request start.
		 *
		 * \param      req   The request
		 * \param      res   The response
		 *
		 * \return     True if acquisition is started, false otherwise
		 */
		bool on_request_start(std_srvs::Empty::Request& req,
							  std_srvs::Empty::Response& res);

		/*! \brief      Called on request stop.
		 *
		 * \param      req   The request
		 * \param      res   The response
		 *
		 * \return     True if acquisition is stopped, false otherwise
		 */
		bool on_request_stop(std_srvs::Empty::Request& req,
							 std_srvs::Empty::Response& res);

		/*! \brief      Called on request quit.
		 *
		 * \param      req   The request
		 * \param      res   The response
		 *
		 * \return     True
		 */
		bool on_request_quit(std_srvs::Empty::Request& req,
							 std_srvs::Empty::Response& res);

		/*! \brief      Called on request info.
		 *
		 * \param      req   The request
		 * \param      res   The response
		 *
		 * \return     True
		 */
		bool on_request_info(rosneuro_msgs::GetAcquisitionInfo::Request& req,
							 rosneuro_msgs::GetAcquisitionInfo::Response& res);

		/*! \brief      Called when the device is idle.
		 *
		 * \return      Acquisition::IS_IDLE if autostart is false, Acquisition::IS_QUIT if the
		 * 				device cannot be started, Acquisition::IS_STARTED otherwise
		 */
		unsigned int on_device_idle(void);

		/*! \brief      Called when the device is started.
		 *
		 * \return      Acquisition::IS_DOWN if the device is down, Acquisition::IS_STARTED otherwise
		 */
		unsigned int on_device_started(void);

		/*! \brief      Called when the device is stopped.
		 *
		 * \return      Acquisition::IS_STOPPED
		 */
		unsigned int on_device_stopped(void);

		/*! \brief      Called on device requesting.
		 */
		unsigned int on_device_requesting(void);

		/*! \brief      Called when the device is down.
		 *
		 * \return      Acquisition::IS_QUIT if the device cannot be reopened, Acquisition::IS_STARTED otherwise 
		 */
		unsigned int on_device_down(void);


	private:
        void advertise(void);
        bool configureDevice(void);
        bool setParams(void);
        void startAcquisitionLoop(void);

		ros::NodeHandle		nh_;
		ros::NodeHandle		p_nh_;
		ros::Publisher		pub_;
		ros::ServiceServer	srv_start_;
		ros::ServiceServer	srv_stop_;
		ros::ServiceServer	srv_quit_;
		ros::ServiceServer	srv_info_;
		std::string			topic_;
		unsigned int		state_;
		uint32_t 			neuroseq_;


		boost::shared_ptr<Device>	dev_;

		std::string		devname_;
		std::string		plugin_;
		float			framerate_;
		bool			reopen_;
		bool			autostart_;
		
		rosneuro_msgs::NeuroFrame msg_;
		NeuroFrame	frame_;


		std::unique_ptr<pluginlib::ClassLoader<Device>> loader_;

        FRIEND_TEST(AcquisitionTestSuite, Initialization);
        FRIEND_TEST(AcquisitionTestSuiteConf, Configure);
        FRIEND_TEST(AcquisitionTestSuite, RunSuccess);
        FRIEND_TEST(AcquisitionTestSuite, RunFailure);
        FRIEND_TEST(AcquisitionTestSuite, RunFailureOpen);
};

}

#endif
