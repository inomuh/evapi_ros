#include "ros/ros.h"

#include "im_msgs/Bumper.h"
#include "im_msgs/BumperState.h"

#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>		/* ioctl */


#define MAX_BUMPER 3

struct bumper_ioc_transfer 
{
	int i_gpio_pin[MAX_BUMPER];
	int i_size; 

};

struct bumper_data
{
	int i_bumper_no;
	int i_value;
};

#define BUMPER_IOCTL_BASE 0xB0

		
#define IOCTL_SET_BUMPERS _IOW(BUMPER_IOCTL_BASE, 0, struct bumper_ioc_transfer) /* Set Param */
#define IOCTL_READ_BUMPERS _IOWR(BUMPER_IOCTL_BASE, 1, struct bumper_data) /* Read range */

using namespace std;



int main(int argc, char **argv)
{
	int i_fd;
	struct bumper_ioc_transfer pins;
	struct bumper_data data;
	
	// ROS PARAMS
	vector<int> T_i_bumper_pins;
			
	double d_frequency;
	
	string str_topic_name;
	string str_driver_path;
	
	bool b_always_on;
	// rosparams end
	
	ros::Publisher pub_bumper;
	im_msgs::Bumper bumpers;
	
		
	ros::init(argc, argv, "evarobot_bumper");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_bumper/driverPath", str_driver_path, "/dev/evarobotBumper");
	n.param<string>("evarobot_bumper/commandTopic", str_topic_name, "bumper");
	n.param("evarobot_bumper/alwaysOn", b_always_on, false);

	
	if(!n.getParam("evarobot_bumper/frequency", d_frequency))
	{
		ROS_ERROR("Failed to get param 'frequency'");
	} 

	n.getParam("evarobot_bumper/pinBumper", T_i_bumper_pins);

	

	if(T_i_bumper_pins.size() > MAX_BUMPER)
	{
		ROS_ERROR("Number of bumper sensors mustn't be greater than %d", MAX_BUMPER);
	}
	
	#ifdef DEBUG
		ROS_INFO("Number of bumpers: %d", T_i_bumper_pins.size());
	#endif
	
		
	
	// Set publisher
	pub_bumper = n.advertise<im_msgs::Bumper>(str_topic_name.c_str(), 10);
	
	// Define frequency
	ros::Rate loop_rate(d_frequency);
	
	i_fd = open(str_driver_path.c_str(), O_RDWR);
	if(i_fd < 0)
	{
		printf("file %s either does not exist or has been locked by another process\n", str_driver_path.c_str());
		exit(-1);
	}
	
	pins.i_size = T_i_bumper_pins.size();
	
	for(uint i = 0; i < T_i_bumper_pins.size(); i++)
	{
		pins.i_gpio_pin[i] = T_i_bumper_pins[i];
	}
	
	ioctl(i_fd, IOCTL_SET_BUMPERS, &pins);

	while(ros::ok())
	{		
		
		for(uint i = 0; i < T_i_bumper_pins.size(); i++)
		{
			im_msgs::BumperState bumper_state;
		
			data.i_bumper_no = i;
			ioctl(i_fd, IOCTL_READ_BUMPERS, &data);
			
			bumper_state.bumper_state = (data.i_value == 1) ? false : true;
			
			bumpers.state.push_back(bumper_state);

		}
		
		bumpers.header.stamp = ros::Time::now();

		// Publish Data
		if(pub_bumper.getNumSubscribers() > 0 || b_always_on)
		{
			pub_bumper.publish(bumpers);
		}
		bumpers.state.clear();
		
		loop_rate.sleep();	
	
	}
	
	
	return 0;
}
