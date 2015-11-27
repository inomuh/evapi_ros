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

//#ifndef DEBUG
//#define DEBUG
//#endif

#define MAX_IR 7

using namespace std;

class IMDynamicReconfig
{
	public:
	IMDynamicReconfig();
	
	void CallbackReconfigure(evarobot_infrared::ParamsConfig &config, uint32_t level);
	
	ros::NodeHandle n_private;
	
	double d_min_range, d_max_range, d_fov;
	double d_max_freq, d_min_freq;
	double d_par_k, d_par_a, d_par_b;
	
	bool b_always_on;
		
	dynamic_reconfigure::Server<evarobot_infrared::ParamsConfig> srv;
	dynamic_reconfigure::Server<evarobot_infrared::ParamsConfig>::CallbackType f;
};

class IMInfrared
{
	public:
	
	IMInfrared(int id,
					boost::shared_ptr<IMADC> _adc,
				  boost::shared_ptr<IMDynamicReconfig> _dynamic_params);
	~IMInfrared();
	bool ReadRange();
	void Publish();
	void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
	
	ros::NodeHandle n;
  ros::Publisher pub_inf;
  diagnostic_updater::Updater updater;
  	
	private:
	
	int i_id;
	bool b_is_alive;
	
	sensor_msgs::Range inf_msg;
	
	boost::shared_ptr<IMADC> adc;
	boost::shared_ptr<IMDynamicReconfig> dynamic_params;
	
	double min_freq;
	double max_freq;
	diagnostic_updater::HeaderlessTopicDiagnostic * pub_inf_freq;

};

#endif
