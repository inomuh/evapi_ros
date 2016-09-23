#ifdef English_dox
/**
 * \file   evarobot_sonar.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Controls and publishes sonar data.
 * \details
 */
#endif

#ifndef INCLUDE_EVAROBOT_SONAR_H_
#define INCLUDE_EVAROBOT_SONAR_H_

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dynamic_reconfigure/server.h>
#include <evarobot_sonar/SonarParamsConfig.h>

#include <time.h>
#include <vector>
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <exception>

#include <boost/shared_ptr.hpp>

#include <ros/console.h>
#include "ErrorCodes.h"

/**
 * In order to change max number of sonar, MAX_SONAR macros
 * in evarobot_sonar.cpp(ROS) and driver_sonar.h(MODULE) should be modified.
 */

#ifdef English_dox
//! Maximum number of sonar sensors.
#endif
#define MAX_SONAR 7

#ifdef English_dox
//! Contains id and gpio pin number of sonar sensor.
#endif
struct sonar_ioc_transfer 
{
	int i_gpio_pin;
	int i_sonar_id;
};

#ifdef English_dox
//! Structure of sonar data. It contains id of sonar and distance.
#endif
struct sonar_data
{
	int i_sonar_id;
	int i_distance;
};

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code = 0;

#ifdef English_dox
//! Sonar driver base address.
#endif
#define SNRDRIVER_IOCTL_BASE 0xAE

#ifdef English_dox
//! Set param macro
#endif
#define IOCTL_SET_PARAM _IOW(SNRDRIVER_IOCTL_BASE, 0, struct sonar_ioc_transfer) 

#ifdef English_dox
//! Read range macro
#endif
#define IOCTL_READ_RANGE _IOWR(SNRDRIVER_IOCTL_BASE, 1, struct sonar_data) 

#ifdef English_dox
//! GPIO pin numbers of sonar sensors from sonar id 0 to 6.
#endif
int SONAR[] = {23, 24, 25, 26, 16, 20, 21};

#ifdef English_dox
//! Controls new parameters are set or not.
#endif
bool b_is_received_params = false;

#ifdef English_dox
//! Field of view of sonar sensor.
#endif
double g_d_field_of_view;

#ifdef English_dox
//! Minimum range of sonar sensor.
#endif
double g_d_min_range;

#ifdef English_dox
//! Maximum range of sonar sensor.
#endif
double g_d_max_range;

#ifdef English_dox
//! If this variable false; sonar topic is published only there is at least one subscriber. If this variable true; sonar topic is published in all conditions.
#endif
bool g_b_always_on;

#ifdef English_dox
//! Aliveness of sonar sensors.
#endif
int g_i_is_alive[MAX_SONAR];

using namespace std;

#ifdef English_dox
//! Provides setting parameters at runtime.
#endif
class IMDynamicReconfig
{
	public:
		#ifdef English_dox
		//! Constructor.
		#endif
		IMDynamicReconfig();

		#ifdef English_dox
		//! Changes values of variables at runtime.
		/**
		 * \param &config includes values.
		 * \param level is not used.
		 */
		#endif
		void CallbackReconfigure(evarobot_sonar::SonarParamsConfig &config, uint32_t level);

		#ifdef English_dox
		//! Creates ROS node handler.
		#endif
		ros::NodeHandle n_private;
		
		#ifdef English_dox
		//! Minimum range of sonar sensor.
		#endif
		double d_min_range;

		#ifdef English_dox
		//! Maximum range of sonar sensor.
		#endif
		double d_max_range;

		#ifdef English_dox
		//! Field of view of sonar sensor.
		#endif
		double d_fov;

		#ifdef English_dox
		//! Maximum frequency
		#endif
		double d_max_freq;

		#ifdef English_dox
		//! Minimum frequency
		#endif
		double d_min_freq;

		#ifdef English_dox
		//! If this variable false; sonar topic is published only there is at least one subscriber. If this variable true; sonar topic is published in all conditions.
		#endif
		bool b_always_on;

		#ifdef English_dox
		//! Dynamic reconfigure service
		#endif
		dynamic_reconfigure::Server<evarobot_sonar::SonarParamsConfig> srv;

		#ifdef English_dox
		//! Dynamic reconfigure service callback type
		#endif
		dynamic_reconfigure::Server<evarobot_sonar::SonarParamsConfig>::CallbackType f;
};

#ifdef English_dox
//! Represents sonar sensor. Publishes sonar<id> topic.
#endif
class IMSONAR
{
	public:
		#ifdef English_dox
		//! Constructor
		/**
		 * \param fd File descriptor id of sonar driver file
		 * \param id id of sonar sensor
		 * \param pin GPIO pin number of sonar sensor
		 * \param _dynamic_params IMDynamicReconfig class reference
		 */
		#endif
		IMSONAR(int fd,
				int id,
				int pin,
				boost::shared_ptr<IMDynamicReconfig> _dynamic_params);

		#ifdef English_dox
		//! Destructor
		#endif
		~IMSONAR();

		#ifdef English_dox
		//! Writes sonar id to sonar driver file.
		#endif
		void Trigger();

		#ifdef English_dox
		//! Sleeps the execution for 0.3 second.
		#endif
		void Wait();

		#ifdef English_dox
		//! Reads file and gets distance data.
		/**
		 * \retval <0 if there is an error
		 * \retval >=0 no error
		 */
		#endif
		int Echo();
	
		#ifdef English_dox
		//! Checks sonar is alive or not.
		/**
		 * \retval true if sonar is alive
		 * \retval false if sonar is not alive
		 */
		#endif
		bool Check();

		#ifdef English_dox
		//! Publishes data from sonar<id> topic.
		#endif
		void Publish();

		#ifdef English_dox
		//! If an error occurs publishes it. Also publishes aliveness of sonar sensor.
		/**
		 * \param &stat Messages are added to this reference.
		 */
		#endif
		void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

		#ifdef English_dox
		//! Creates ROS node handler.
		#endif
		ros::NodeHandle n;

		#ifdef English_dox
		//! Sonar message publisher
		#endif
		ros::Publisher pub_sonar;

		#ifdef English_dox
		//! Diagnostics updater to handle and publish error.
		#endif
		diagnostic_updater::Updater updater;

	private:
		#ifdef English_dox
		//! File descriptor id of sonar driver file
		#endif
		int i_fd;

		#ifdef English_dox
		//! GPIO pin number of sonar.
		#endif
		int i_pin_no;

		#ifdef English_dox
		//! Id of the sonar [0-6]
		#endif
		int i_id;

		#ifdef English_dox
		//! Aliveness of sonar sensor.
		#endif
		bool b_is_alive;

		#ifdef English_dox
		//! ROS message in type of sensor_msgs::Range.
		#endif
		sensor_msgs::Range sonar_msg;

		#ifdef English_dox
		//! IMDynamicReconfig class reference.
		#endif
		boost::shared_ptr<IMDynamicReconfig> dynamic_params;

		#ifdef English_dox
		//! Minimum frequency.
		#endif
		double min_freq;

		#ifdef English_dox
		//! Maximum frequency.
		#endif
		double max_freq;

		#ifdef English_dox
		//! Diagnostics updater to handle and publish error.
		#endif
		diagnostic_updater::HeaderlessTopicDiagnostic * pub_sonar_freq;

		#ifdef English_dox
		//! Object from structure of sonar data. It contains id of sonar and distance.
		#endif
		struct sonar_data data;
};


#endif
