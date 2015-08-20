#include "ros/ros.h"
#include "sensor_msgs/Range"
#include "IMGPIO.h"

#include <time.h>


#define CRITIC_TRIGGER_TIME 1.2


struct timespec ts;

int init()
{

	fd = open("/sys/class/gpio/export", O_WRONLY);
	if(fd < 0)
	{
		printf("export acilamadi\n");
		return -1;
	}
	write(fd,"24",2);
	close(fd);

        fd = open("/sys/class/gpio/gpio24/direction", O_WRONLY);
        if(fd < 0)
        {
                printf("direction acilamadi\n");
                return -1;
        }

        write(fd,"out",3);
        close(fd);

        fd = open("/sys/class/gpio/gpio24/value", O_WRONLY);
        if(fd < 0)
        {
                printf("value acilamadi\n");
                return -1;
        }


        clock_gettime(CLOCK_MONOTONIC, &ts);

         // Lock memory to ensure no swapping is done.
        if(mlockall(MCL_FUTURE|MCL_CURRENT)){
                fprintf(stderr,"WARNING: Failed to lock memory\n");
        }

        // Set our thread to real time priority
        struct sched_param sp;
        sp.sched_priority = 30;
        if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)){
                fprintf(stderr,"WARNING: Failed to set stepper thread"
                        "to real-time priority\n");
        }

	return 0;

}


using namespace std;

static void SleepUntil(struct timespec *ts, int delay)
{
	
	ts->tv_nsec += delay;
	if(ts->tv_nsec >= 1000*1000*1000){
		ts->tv_nsec -= 1000*1000*1000;
		ts->tv_sec++;
	}
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts,  NULL);
}


int main()
{
	
	double d_frequency;
	double d_sleep_time;
	string str_device_path;

	unsigned int u_i_delay = 100*1000; // Note: Delay in ns
	
	ros::init(argc, argv, "evarobot_sonar");
	ros::NodeHandle n;
	
	n.param<string>("evarobot_odometry/driverPath", str_driver_path, "/dev/evarobotSonar");
	n.param<string>("evarobot_odometry/odomTopic", str_topic, "odom");
	n.param<string>("evarobot_odometry/odomFrame", str_frame_id, "odom");
	
	if(!n.getParam("evarobot_odometry/frequency", d_frequency))
	{
		ROS_ERROR("Failed to get param 'frequency'");
	} 
	
	/******************************************************************/
	/* Init */
	/******************************************************************/
	IMGPIO * gpio[]  = {new IMGPIO(IMGPIO::GPIO0),
						new IMGPIO(IMGPIO::GPIO1),
						new IMGPIO(IMGPIO::GPIO2),
						new IMGPIO(IMGPIO::GPIO3),
						new IMGPIO(IMGPIO::GPIO6),
						new IMGPIO(IMGPIO::GPIO5)}
	
	// Set pin direction as output
	for(int i = 0; i < 6; i++)
		gpio[i].SetPinDirection(IMGPIO::OUTPUT);
		
	clock_gettime(CLOCK_MONOTONIC, &ts);

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
	
	/*****************************************************************/
	
	// Set publishers
	ros::Publisher pose_sonar0 = n.advertise<nav_msgs::Odometry>(str_topic_name.c_str(), 10);
	ros::Publisher pose_sonar1 = n.advertise<nav_msgs::Odometry>(str_topic_name.c_str(), 10);
	ros::Publisher pose_sonar2 = n.advertise<nav_msgs::Odometry>(str_topic_name.c_str(), 10);
	ros::Publisher pose_sonar3 = n.advertise<nav_msgs::Odometry>(str_topic_name.c_str(), 10);
	ros::Publisher pose_sonar4 = n.advertise<nav_msgs::Odometry>(str_topic_name.c_str(), 10);
	ros::Publisher pose_sonar5 = n.advertise<nav_msgs::Odometry>(str_topic_name.c_str(), 10);
	
	// Define frequency
	ros::Rate loop_rate(d_frequency);
	
	// Opening the encoder.
	i_fd = open(str_device_path.c_str(), O_RDWR);
	
	// Cleaning the encoder.
	//    write(fd, write_buf, sizeof(write_buf));
	write(i_fd, "clean", 5);

	d_sleep_time = 1 / d_frequency / 6;
	
/*	
	if(d_sleep_time < CRITIC_TRIGGER_TIME)
	{
		ROS_ERROR("Critical frequency");
	}
*/
	
	sensor_msgs::Range sonars[6];
	
	// init sonars variable
	for(int i = 0; i < 6; i++)
	{
		sonars[i].header.frame_id
		sonars[i].radiation_type = 0;
		sonars[i].field_of_view
		sonars[i].min_range
		sonars[i].max_range
	}
	
	while(ros::ok())
	{
		
		
		
		// Trigger sensors
		for(int i = 0; i < 6; i++)
		{
			gpio[i].SetPinValue(IMGPIO::HIGH);
			SleepUntil(&ts, u_i_delay);
			
			gpio[i].SetPinValue(IMGPIO::LOW);
			SleepUntil(&ts, u_i_delay);
			
			sonars[i].range = 
			sonars[i].header.stamp
		}
		
		// Read sensors
		
		// Publish Data
		if(pose_sonar0.getNumSubscribers() > 0)
		{
			pose_pub.publish(msg);
		}
		
		if(pose_sonar1.getNumSubscribers() > 0)
		{
			pose_pub.publish(msg);
		}
		
		if(pose_sonar2.getNumSubscribers() > 0)
		{
			pose_pub.publish(msg);
		}
		
		if(pose_sonar3.getNumSubscribers() > 0)
		{
			pose_pub.publish(msg);
		}
		
		if(pose_sonar4.getNumSubscribers() > 0)
		{
			pose_pub.publish(msg);
		}
		
		if(pose_sonar5.getNumSubscribers() > 0)
		{
			pose_pub.publish(msg);
		}
		
	}
	
	return 0;
}
