#ifndef INCLUDE_EVAROBOT_BUMPER_H_
#define INCLUDE_EVAROBOT_BUMPER_H_

#include "ros/ros.h"

#include "im_msgs/Bumper.h"
#include "im_msgs/BumperState.h"

#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>		/* ioctl */

#include "IMEIO.h"


#define MAX_BUMPER 3

char SEM_NAME[]= "i2c";

#endif
