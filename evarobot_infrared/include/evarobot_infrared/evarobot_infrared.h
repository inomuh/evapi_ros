#ifdef English_dox
/**
 * \file   evarobot_infrared.h
 * \author Mehmet Akcakoca (mehmet.akcakoca@inovasyonmuhendislik.com)
 * \date   Mar, 2015
 * \brief  Controls and publishes infrared data.
 * \details
 */
#endif

#ifndef INCLUDE_EVAROBOT_INFRARED_H_
#define INCLUDE_EVAROBOT_INFRARED_H_

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dynamic_reconfigure/server.h>
#include <evarobot_infrared/ParamsConfig.h>

#include <vector>
#include <sstream>

#include <boost/shared_ptr.hpp>

#include "IMADC.h"

#include <ros/console.h>
#include "ErrorCodes.h"

#ifdef English_dox
//! Maximum number of infrared sensors.
#endif
#define MAX_IR 8

using namespace std;

#ifdef English_dox
//! If an error occurs, shows its code number.
#endif
int i_error_code = 0;

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
		void CallbackReconfigure(evarobot_infrared::ParamsConfig &config, uint32_t level);

		#ifdef English_dox
		//! Creates ROS node handler.
		#endif
		ros::NodeHandle n_private;

		#ifdef English_dox
		//! Minimum range of infrared sensor.
		#endif
		double d_min_range;

		#ifdef English_dox
		//! Maximum range of infrared sensor.
		#endif
		double d_max_range;

		#ifdef English_dox
		//! Field of view of infrared sensor.
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
		//! These parameters are used to calculate distance from voltage value.
		#endif
		double d_par_k, d_par_a, d_par_b;
		
		#ifdef English_dox
		//! If this variable false; infrared topic is published only there is at least one subscriber. If this variable true; infrared topic is published in all conditions.
		#endif
		bool b_always_on;

		#ifdef English_dox
		//! Dynamic reconfigure service
		#endif
		dynamic_reconfigure::Server<evarobot_infrared::ParamsConfig> srv;

		#ifdef English_dox
		//! Dynamic reconfigure service callback type
		#endif
		dynamic_reconfigure::Server<evarobot_infrared::ParamsConfig>::CallbackType f;
};

#ifdef English_dox
//! Represents infrared sensor. Publishes ir<id> topic.
#endif
class IMInfrared
{
	public:
		#ifdef English_dox
		//! Constructor
		/**
		 * \param id id of infrared sensor
		 * \param _adc IMADC class reference
		 * \param _dynamic_params IMDynamicReconfig class reference
		 */
		#endif
		IMInfrared(int id,
				boost::shared_ptr<IMADC> _adc,
				boost::shared_ptr<IMDynamicReconfig> _dynamic_params);

		#ifdef English_dox
		//! Destructor
		#endif
		~IMInfrared();

		#ifdef English_dox
		//! Reads raw data from infrared device using adc. Converts raw data to range. Prepares ROS message with range value.
		/**
		 * \retval true if sensor is alive
		 * \retval false if sensor is not alive
		 */
		#endif
		bool ReadRange();

		#ifdef English_dox
		//! Publishes message which is prepared in ReadRange function.
		#endif
		void Publish();

		#ifdef English_dox
		//! If an error occurs publishes it. Also publishes aliveness of infrared sensor.
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
		//! Infrared message publisher
		#endif
		ros::Publisher pub_inf;

		#ifdef English_dox
		//! Diagnostics updater to handle and publish error.
		#endif
		diagnostic_updater::Updater updater;
  	
	private:
		#ifdef English_dox
		//! id of infrared sensor.
		#endif
		int i_id;

		#ifdef English_dox
		//! Aliveness of infrared sensor.
		#endif
		bool b_is_alive;

		#ifdef English_dox
		//! ROS message in type of sensor_msgs::Range.
		#endif
		sensor_msgs::Range inf_msg;

		#ifdef English_dox
		//! IMADC class reference.
		#endif
		boost::shared_ptr<IMADC> adc;

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
		diagnostic_updater::HeaderlessTopicDiagnostic * pub_inf_freq;
};

#endif
