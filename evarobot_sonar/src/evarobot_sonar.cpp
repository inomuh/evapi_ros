#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include <time.h>
#include <vector>
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <unistd.h>		/* exit */
#include <sys/ioctl.h>		/* ioctl */

//#ifndef DEBUG
//#define DEBUG
//#endif

// In order to change max number of sonar, MAX_SONAR macros 
// in evarobot_sonar.cpp(ROS) and driver_sonar.h(MODULE)
// should be modified.
#define MAX_SONAR 7
#define MAX_TRIGGER_TIME 1.2

struct sonar_ioc_transfer 
{
	int i_gpio_reflection[MAX_SONAR];
	int	i_gpio_trigger[MAX_SONAR];
	int i_size;

};

struct sonar_data
{
	int i_sonar_no;
	int i_distance;
};

#define SNRDRIVER_IOCTL_BASE 0xAE

/* Set param macro*/
#define IOCTL_SET_PARAM _IOW(SNRDRIVER_IOCTL_BASE, 0, struct sonar_ioc_transfer) 
/* Read range macro*/
#define IOCTL_READ_RANGE _IOWR(SNRDRIVER_IOCTL_BASE, 1, struct sonar_data) 


using namespace std;

int main(int argc, char **argv)
{
	int i_fd;
	struct sonar_ioc_transfer pins;
	struct sonar_data data;
	
	// ROS PARAMS
	vector<int> T_i_trigger_pins;
	vector<int> T_i_reflection_pins;
	vector<double> T_d_posX, T_d_posY, T_d_posZ, T_d_angle_yaw;
	vector<double> T_d_min_ranges, T_d_max_ranges;
	vector<double> T_d_field_of_views;
		
	double d_frequency;
	
	string str_driver_path;
	string str_namespace;
	string str_frame_id;
	
	bool b_always_on;
	// rosparams end
	
	vector<ros::Publisher> T_pub_sonar;
	vector<sensor_msgs::Range> T_sonars;
	
	double d_sleep_time;
	

	unsigned int u_i_delay = 100*1000; // Note: Delay in ns
	
	ros::init(argc, argv, "evarobot_sonar");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_sonar/driverPath", str_driver_path, "/dev/evarobotSonar");
	n.param<string>("evarobot_sonar/namespace", str_namespace, "gazebo");
	n.param<string>("evarobot_sonar/frameNamespace", str_frame_id, "sonar");
	n.param("evarobot_sonar/alwaysOn", b_always_on, false);

	
	if(!n.getParam("evarobot_sonar/frequency", d_frequency))
	{
		ROS_ERROR("Failed to get param 'frequency'");
	} 


	n.getParam("evarobot_sonar/field_of_view", T_d_field_of_views);
	n.getParam("evarobot_sonar/minRange", T_d_min_ranges);
	n.getParam("evarobot_sonar/maxRange", T_d_max_ranges);

	n.getParam("evarobot_sonar/pinTrigger", T_i_trigger_pins);
	n.getParam("evarobot_sonar/pinReflection", T_i_reflection_pins);
	
	n.getParam("evarobot_sonar/posX", T_d_posX);
	n.getParam("evarobot_sonar/posY", T_d_posY);
	n.getParam("evarobot_sonar/posZ", T_d_posZ);
	n.getParam("evarobot_sonar/angleYaw", T_d_angle_yaw);
	
	if( !(T_d_field_of_views.size() == T_d_min_ranges.size()
	&& T_d_min_ranges.size() == T_d_max_ranges.size()
	&& T_d_min_ranges.size() == T_i_trigger_pins.size()
	&& T_i_trigger_pins.size() == T_i_reflection_pins.size() 
	&& T_i_reflection_pins.size() == T_d_posX.size()
	&& T_d_posX.size() == T_d_posY.size()
	&& T_d_posY.size() == T_d_posZ.size()
	&& T_d_posZ.size() == T_d_angle_yaw.size())	)
	{
		ROS_ERROR("Size of params are not same.");
	}
	else if(T_i_trigger_pins.size() > MAX_SONAR)
	{
		ROS_ERROR("Number of sonar devices mustn't be greater than %d", MAX_SONAR);
	}
	
	#ifdef DEBUG
		ROS_INFO("Number of sonars: %d", T_d_posZ.size());
	#endif
	
	/******************************************************************/
	/* Init */
	/******************************************************************/
	
/*	clock_gettime(CLOCK_MONOTONIC, &ts);

	// Lock memory to ensure no swapping is done.
	if(mlockall(MCL_FUTURE|MCL_CURRENT))
	{
		ROS_ERROR("Failed to lock memory");
	}

	// Set our thread to real time priority
	struct sched_param sp;
	
	sp.sched_priority = 30;
	
	if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
	{
		ROS_ERROR("Failed to set stepper thread"
				"to real-time priority");
	}
	*/

	i_fd = open(str_driver_path.c_str(), O_RDWR);
	
	if(i_fd < 0)
	{
		printf("file %s either does not exist or has been locked by another process\n", str_driver_path.c_str());
		exit(-1);
	}
		
	/*****************************************************************/
	
	
	
	
	
	// Set publishers
	for(uint i = 0; i < T_i_trigger_pins.size(); i++)
	{
		stringstream ss_topic;
		ss_topic << str_namespace << "/sonar" << i;
		ros::Publisher dummy_publisher = n.advertise<sensor_msgs::Range>(ss_topic.str().c_str(), 10);
		T_pub_sonar.push_back(dummy_publisher);
	}

	// Define frequency
	ros::Rate loop_rate(d_frequency);

	d_sleep_time = 1 / d_frequency / T_i_reflection_pins.size();
	
/*	
	if(d_sleep_time < CRITIC_TRIGGER_TIME)
	{
		ROS_ERROR("Critical frequency");
	}
*/	
	
	
	pins.i_size = T_i_reflection_pins.size();
	
	// init sonar variables
	for(uint i = 0; i < T_i_reflection_pins.size(); i++)
	{
		pins.i_gpio_reflection[i] = T_i_reflection_pins[i];
		pins.i_gpio_trigger[i] = T_i_trigger_pins[i];
		
		stringstream ss_frame;
		ss_frame << str_frame_id << "/sonar" << i;
		
		sensor_msgs::Range dummy_sonar;
		
		dummy_sonar.header.frame_id = ss_frame.str();
		dummy_sonar.radiation_type = 0;
		dummy_sonar.field_of_view = T_d_field_of_views[i];
		dummy_sonar.min_range = T_d_min_ranges[i];
		dummy_sonar.max_range = T_d_max_ranges[i];
		
		T_sonars.push_back(dummy_sonar);
	}
	
	ioctl(i_fd, IOCTL_SET_PARAM, &pins);
	
	while(ros::ok())
	{		
		for(uint i = 0; i < T_pub_sonar.size(); i++)
		{
			// Read sensors
			data.i_sonar_no = i;
			while(ioctl(i_fd, IOCTL_READ_RANGE, &data) != 1);
			
			T_sonars[i].range = (float)(data.i_distance * 0.0001);
			T_sonars[i].header.stamp = ros::Time::now();
			//printf("Sonar Distance[%d]:  %d \n", i_i, data.i_distance);
			
			// Publish Data
			if(T_pub_sonar[i].getNumSubscribers() > 0 || b_always_on)
			{
				T_pub_sonar[i].publish(T_sonars[i]);
			}
		}
		
		loop_rate.sleep();	
	
	}
	
	close(i_fd);
	
	return 0;
}
