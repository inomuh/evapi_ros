#ifndef INCLUDE_EVAROBOT_SONAR_H_
#define INCLUDE_EVAROBOT_SONAR_H_

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
	int i_gpio_pins[MAX_SONAR];
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


int SONAR[] = {23, 24, 25, 7, 16, 20, 21};


using namespace std;

#endif
