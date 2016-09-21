//! Bu sınıfa ait dokümantasyon evapi_ros 85rpm altındaki dokümantasyon ile aynıdır.

#ifndef INCLUDE_EVAROBOT_BUMPER_H_
#define INCLUDE_EVAROBOT_BUMPER_H_

#include "ros/ros.h"

#include "im_msgs/Bumper.h"
#include "im_msgs/BumperState.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <vector>
#include <sstream>

#include <fcntl.h>
#include <sys/ioctl.h>		/* ioctl */

#include <ros/console.h>

#include "IMEIO.h"
#include "ErrorCodes.h"
#include "im_msgs/EKBTest.h"

#define MAX_BUMPER 3

char SEM_NAME[]= "i2c";

#endif
