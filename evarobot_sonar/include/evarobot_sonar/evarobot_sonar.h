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

#include <unistd.h>		/* exit */
#include <sys/ioctl.h>		/* ioctl */
#include <exception>

#include <boost/shared_ptr.hpp>

#ifndef DEBUG
#define DEBUG
#endif

// In order to change max number of sonar, MAX_SONAR macros 
// in evarobot_sonar.cpp(ROS) and driver_sonar.h(MODULE)
// should be modified.

#define MAX_TRIGGER_TIME 1.2
#define MAX_SONAR 7

struct sonar_ioc_transfer 
{
	int i_gpio_pin;
	int i_sonar_id;
};

struct sonar_data
{
	int i_sonar_id;
	int i_distance;
};

#define SNRDRIVER_IOCTL_BASE 0xAE

/* Set param macro*/
#define IOCTL_SET_PARAM _IOW(SNRDRIVER_IOCTL_BASE, 0, struct sonar_ioc_transfer) 
/* Read range macro*/
#define IOCTL_READ_RANGE _IOWR(SNRDRIVER_IOCTL_BASE, 1, struct sonar_data) 


int SONAR[] = {23, 24, 25, 26, 16, 20, 21};
bool b_is_received_params = false;

double g_d_field_of_view;
double g_d_min_range;
double g_d_max_range;
bool g_b_always_on;

int g_i_is_alive[MAX_SONAR];

using namespace std;

class IMDynamicReconfig
{
	public:
	IMDynamicReconfig();
	
	void CallbackReconfigure(evarobot_sonar::SonarParamsConfig &config, uint32_t level);
	
	ros::NodeHandle n_private;
	
	double d_min_range, d_max_range, d_fov;
	double d_max_freq, d_min_freq;
	bool b_always_on;
		
	dynamic_reconfigure::Server<evarobot_sonar::SonarParamsConfig> srv;
	dynamic_reconfigure::Server<evarobot_sonar::SonarParamsConfig>::CallbackType f;
};

class IMSONAR
{
	public:
	
	IMSONAR(int fd, 
				 int id, 
				 int pin,
				 boost::shared_ptr<IMDynamicReconfig> _dynamic_params);
	~IMSONAR();
	void Trigger();
	void Wait();
	int Echo();
/*	
	void CheckTrigger();
	void CheckWait();
	bool CheckEcho();
*/
	bool Check();
	void Publish();
	void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
	
	ros::NodeHandle n;
	ros::Publisher pub_sonar;
	diagnostic_updater::Updater updater;
  	
	private:
	int i_fd;
	int i_pin_no, i_id;
	bool b_is_alive;
	
	sensor_msgs::Range sonar_msg;
	
	boost::shared_ptr<IMDynamicReconfig> dynamic_params;
	//IMDynamicReconfig * dynamic_params;
	
	double min_freq;
	double max_freq;
	diagnostic_updater::HeaderlessTopicDiagnostic * pub_sonar_freq;

	
	struct sonar_data data;
};


#endif
